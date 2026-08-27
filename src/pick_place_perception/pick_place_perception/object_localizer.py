"""Locate a red workpiece in 3D using synchronized RGB and depth images."""

from collections import deque

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class ObjectLocalizer(Node):
    """Segment the target colour and back-project its depth into `world`."""

    def __init__(self):
        super().__init__("object_localizer")

        defaults = {
            "rgb_topic": "/camera/image",
            "depth_topic": "/camera/depth_image",
            "camera_info_topic": "/camera/camera_info",
            "target_frame": "world",
            "minimum_area": 180.0,
            "minimum_depth": 0.15,
            "maximum_depth": 3.0,
            "surface_to_center_offset": 0.03,
            "smoothing_window": 7,
            "hue_low_1": 0,
            "hue_high_1": 12,
            "hue_low_2": 170,
            "hue_high_2": 179,
            "saturation_min": 100,
            "value_min": 70,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.target_frame = str(self.parameter("target_frame"))
        self.minimum_area = float(self.parameter("minimum_area"))
        self.minimum_depth = float(self.parameter("minimum_depth"))
        self.maximum_depth = float(self.parameter("maximum_depth"))
        self.surface_to_center_offset = float(
            self.parameter("surface_to_center_offset")
        )
        self.hsv = {
            key: int(self.parameter(key))
            for key in (
                "hue_low_1",
                "hue_high_1",
                "hue_low_2",
                "hue_high_2",
                "saturation_min",
                "value_min",
            )
        }

        self.bridge = CvBridge()
        self.intrinsics = None
        self.positions = deque(
            maxlen=max(1, int(self.parameter("smoothing_window")))
        )
        self.last_log_ns = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CameraInfo,
            str(self.parameter("camera_info_topic")),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        rgb_subscriber = message_filters.Subscriber(
            self,
            Image,
            str(self.parameter("rgb_topic")),
            qos_profile=qos_profile_sensor_data,
        )
        depth_subscriber = message_filters.Subscriber(
            self,
            Image,
            str(self.parameter("depth_topic")),
            qos_profile=qos_profile_sensor_data,
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [rgb_subscriber, depth_subscriber],
            queue_size=10,
            slop=0.12,
        )
        self.synchronizer.registerCallback(self.rgbd_callback)

        self.pose_publisher = self.create_publisher(
            PoseStamped, "/perception/object_pose", 10
        )
        self.valid_publisher = self.create_publisher(
            Bool, "/perception/object_visible", 10
        )
        self.debug_publisher = self.create_publisher(
            Image, "/perception/debug_image", 10
        )
        self.marker_publisher = self.create_publisher(
            Marker, "/perception/object_marker", 10
        )

        self.get_logger().info(
            "RGB-D object localizer ready; waiting for synchronized images."
        )

    def parameter(self, name):
        return self.get_parameter(name).value

    def camera_info_callback(self, message):
        self.intrinsics = (
            float(message.k[0]),
            float(message.k[4]),
            float(message.k[2]),
            float(message.k[5]),
        )

    def rgbd_callback(self, rgb_message, depth_message):
        try:
            image = self.bridge.imgmsg_to_cv2(
                rgb_message, desired_encoding="bgr8"
            ).copy()
            depth = self.bridge.imgmsg_to_cv2(
                depth_message, desired_encoding="passthrough"
            ).astype(np.float32)
        except Exception as exception:  # cv_bridge reports several types
            self.get_logger().error(f"Image conversion failed: {exception}")
            return

        if depth_message.encoding in ("16UC1", "mono16"):
            depth /= 1000.0

        detection = self.detect_target(image)
        if self.intrinsics is None or detection is None:
            self.valid_publisher.publish(Bool(data=False))
            status = "waiting for CameraInfo" if self.intrinsics is None else "target not visible"
            cv2.putText(image, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.publish_debug(image, rgb_message)
            return

        contour, mask = detection
        area = cv2.contourArea(contour)
        if area < self.minimum_area:
            self.valid_publisher.publish(Bool(data=False))
            self.publish_debug(image, rgb_message)
            return

        moments = cv2.moments(contour)
        pixel_u = int(moments["m10"] / moments["m00"])
        pixel_v = int(moments["m01"] / moments["m00"])
        target_depth = self.robust_depth(depth, mask, image.shape[:2])
        if target_depth is None:
            self.valid_publisher.publish(Bool(data=False))
            cv2.putText(image, "depth invalid", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.publish_debug(image, rgb_message)
            return

        fx, fy, cx, cy = self.intrinsics
        point = PointStamped()
        point.header = rgb_message.header
        point.point.x = (pixel_u - cx) * target_depth / fx
        point.point.y = (pixel_v - cy) * target_depth / fy
        point.point.z = target_depth

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                point.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            point_world = do_transform_point(point, transform)
        except TransformException as exception:
            self.valid_publisher.publish(Bool(data=False))
            self.throttled_warning(f"Camera TF is not ready: {exception}")
            self.publish_debug(image, rgb_message)
            return

        # Depth measures the visible top face. The task plans to the cube centre.
        point_world.point.z -= self.surface_to_center_offset
        self.positions.append(
            [point_world.point.x, point_world.point.y, point_world.point.z]
        )
        filtered = np.median(np.asarray(self.positions), axis=0)

        pose = PoseStamped()
        pose.header = point_world.header
        pose.pose.position.x = float(filtered[0])
        pose.pose.position.y = float(filtered[1])
        pose.pose.position.z = float(filtered[2])
        pose.pose.orientation.w = 1.0
        self.pose_publisher.publish(pose)
        self.valid_publisher.publish(Bool(data=True))
        self.publish_marker(pose)

        x, y, width, height = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x + width, y + height), (0, 255, 0), 2)
        cv2.circle(image, (pixel_u, pixel_v), 4, (255, 255, 255), -1)
        text = f"world: {filtered[0]:.3f}, {filtered[1]:.3f}, {filtered[2]:.3f} m"
        cv2.putText(image, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2)
        self.publish_debug(image, rgb_message)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns > 2_000_000_000:
            self.last_log_ns = now_ns
            self.get_logger().info(f"Detected target at {text[7:]}")

    def detect_target(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        low_1 = np.array([self.hsv["hue_low_1"], self.hsv["saturation_min"], self.hsv["value_min"]], dtype=np.uint8)
        high_1 = np.array([self.hsv["hue_high_1"], 255, 255], dtype=np.uint8)
        low_2 = np.array([self.hsv["hue_low_2"], self.hsv["saturation_min"], self.hsv["value_min"]], dtype=np.uint8)
        high_2 = np.array([self.hsv["hue_high_2"], 255, 255], dtype=np.uint8)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv_image, low_1, high_1),
            cv2.inRange(hsv_image, low_2, high_2),
        )
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        contour_mask = np.zeros_like(mask)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
        return contour, contour_mask

    def robust_depth(self, depth, rgb_mask, rgb_shape):
        rgb_height, rgb_width = rgb_shape
        depth_height, depth_width = depth.shape[:2]
        mask = cv2.resize(
            rgb_mask,
            (depth_width, depth_height),
            interpolation=cv2.INTER_NEAREST,
        )
        values = depth[
            (mask > 0)
            & np.isfinite(depth)
            & (depth > self.minimum_depth)
            & (depth < self.maximum_depth)
        ]
        if values.size < 10:
            return None
        # The 30th percentile favours the object's top over background edge pixels.
        return float(np.percentile(values, 30))

    def publish_debug(self, image, source_message):
        message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        message.header = source_message.header
        self.debug_publisher.publish(message)

    def publish_marker(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = "rgbd_detection"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.065
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.05
        marker.color.a = 0.75
        marker.lifetime.sec = 1
        self.marker_publisher.publish(marker)

    def throttled_warning(self, message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns > 2_000_000_000:
            self.last_log_ns = now_ns
            self.get_logger().warning(message)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
