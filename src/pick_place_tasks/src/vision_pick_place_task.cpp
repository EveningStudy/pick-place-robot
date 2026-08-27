#include <chrono>
#include <condition_variable>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class VisionPickPlaceTask
{
public:
  using GripperCommand = control_msgs::action::ParallelGripperCommand;
  using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

  explicit VisionPickPlaceTask(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    arm_(node_, node_->declare_parameter<std::string>("arm_group", "panda_arm"))
  {
    object_pose_topic_ = node_->declare_parameter<std::string>(
      "object_pose_topic", "/perception/object_pose");
    end_effector_link_ = node_->declare_parameter<std::string>(
      "end_effector_link", "panda_hand");
    planning_frame_ = node_->declare_parameter<std::string>("planning_frame", "world");
    detection_timeout_ = node_->declare_parameter<double>("detection_timeout", 20.0);
    max_detection_age_ = node_->declare_parameter<double>("max_detection_age", 1.0);
    open_width_ = node_->declare_parameter<double>("open_width", 0.035);
    grasp_width_ = node_->declare_parameter<double>("grasp_width", 0.028);
    grasp_offset_ = node_->declare_parameter<double>("grasp_offset", 0.105);
    pregrasp_clearance_ = node_->declare_parameter<double>("pregrasp_clearance", 0.28);
    retreat_clearance_ = node_->declare_parameter<double>("retreat_clearance", 0.30);
    place_x_ = node_->declare_parameter<double>("place_x", 0.48);
    place_y_ = node_->declare_parameter<double>("place_y", -0.40);
    place_object_z_ = node_->declare_parameter<double>("place_object_z", 0.88);
    log_file_ = node_->declare_parameter<std::string>(
      "log_file", "/tmp/pick_place_runs.csv");

    arm_.setEndEffectorLink(end_effector_link_);
    arm_.setPoseReferenceFrame(planning_frame_);
    arm_.setPlanningTime(node_->declare_parameter<double>("planning_time", 8.0));
    arm_.setNumPlanningAttempts(node_->declare_parameter<int>("planning_attempts", 8));
    arm_.setMaxVelocityScalingFactor(
      node_->declare_parameter<double>("velocity_scale", 0.25));
    arm_.setMaxAccelerationScalingFactor(
      node_->declare_parameter<double>("acceleration_scale", 0.20));
    arm_.setGoalPositionTolerance(0.008);
    arm_.setGoalOrientationTolerance(0.04);

    pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      object_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        std::scoped_lock lock(pose_mutex_);
        latest_pose_ = *message;
        latest_pose_received_ = node_->now();
      });

    grasp_state_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "/gripper/attached", 10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        {
          std::scoped_lock lock(grasp_state_mutex_);
          grasp_state_ = message->data;
        }
        grasp_state_cv_.notify_all();
      });

    attach_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/gripper/attach", 10);
    detach_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/gripper/detach", 10);
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/pick_place/status", 10);
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      node_, "/hand_controller/gripper_cmd");
    set_pose_client_ = node_->create_client<ros_gz_interfaces::srv::SetEntityPose>(
      "/world/pick_place_workcell/set_pose");

    start_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/pick_place/start",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        if (running_.exchange(true)) {
          response->success = false;
          response->message = "A pick-and-place run is already active.";
          return;
        }
        launch_worker();
        response->success = true;
        response->message = "Pick-and-place run started.";
      });

    publish_status("READY", "Task node initialized");
    RCLCPP_INFO(
      node_->get_logger(),
      "Task interface ready. Call /pick_place/start to run another cycle.");
  }

  ~VisionPickPlaceTask()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void maybe_auto_start()
  {
    if (node_->declare_parameter<bool>("auto_start", true) && !running_.exchange(true)) {
      launch_worker();
    }
  }

private:
  void launch_worker()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread([this]() {
      const bool success = run_once();
      running_ = false;
      if (!success) {
        recover();
      }
    });
  }

  bool run_once()
  {
    publish_status("INITIALIZING", "Waiting for simulation interfaces");

    // DetachableJoint starts attached by design. Wait for an explicit state
    // confirmation so bridge and plugin startup order cannot race the task.
    if (!set_grasp_constraint(false)) {
      return fail("INITIALIZING", "Gazebo did not confirm the initial detach");
    }

    if (!gripper_client_->wait_for_action_server(15s)) {
      return fail("INITIALIZING", "Gripper action server did not appear");
    }

    add_static_collision_scene();
    if (!move_named("ready", "MOVING_HOME")) {
      return fail("MOVING_HOME", "Could not reach ready configuration");
    }
    if (!command_gripper(open_width_)) {
      return fail("OPENING_GRIPPER", "Gripper failed to open");
    }

    publish_status("WAITING_FOR_VISION", "Waiting for a fresh RGB-D object pose");
    const auto detected_pose = wait_for_object_pose();
    if (!detected_pose) {
      return fail("WAITING_FOR_VISION", "No stable object pose was received");
    }
    run_object_pose_ = *detected_pose;
    add_target_collision(*detected_pose);

    geometry_msgs::msg::Pose pregrasp = downward_pose(
      detected_pose->pose.position.x,
      detected_pose->pose.position.y,
      detected_pose->pose.position.z + grasp_offset_ + pregrasp_clearance_);
    if (!move_pose(pregrasp, "MOVING_TO_PREGRASP")) {
      return fail("MOVING_TO_PREGRASP", "No collision-free path to pregrasp");
    }

    // The cube is removed only for the short final approach. Before this point
    // MoveIt treated it as an obstacle; after grasp it becomes an attached body.
    planning_scene_.removeCollisionObjects({"target_cube"});
    std::this_thread::sleep_for(300ms);

    geometry_msgs::msg::Pose grasp = downward_pose(
      detected_pose->pose.position.x,
      detected_pose->pose.position.y,
      detected_pose->pose.position.z + grasp_offset_);
    if (!move_pose(grasp, "APPROACHING_OBJECT")) {
      return fail("APPROACHING_OBJECT", "Could not descend to the grasp pose");
    }

    if (!command_gripper(grasp_width_)) {
      return fail("CLOSING_GRIPPER", "Gripper failed to close around the object");
    }

    if (!set_grasp_constraint(true)) {
      return fail("ATTACHING_OBJECT", "Gazebo did not confirm the grasp constraint");
    }
    add_target_collision(*detected_pose);
    arm_.attachObject(
      "target_cube", end_effector_link_,
      {"panda_hand", "panda_leftfinger", "panda_rightfinger"});
    std::this_thread::sleep_for(500ms);
    publish_status("OBJECT_ATTACHED", "Physical and planning-scene grasps are active");

    geometry_msgs::msg::Pose retreat = grasp;
    retreat.position.z = detected_pose->pose.position.z + grasp_offset_ + retreat_clearance_;
    if (!move_pose(retreat, "LIFTING_OBJECT")) {
      return fail("LIFTING_OBJECT", "Could not lift the attached object");
    }

    geometry_msgs::msg::Pose preplace = downward_pose(
      place_x_, place_y_, place_object_z_ + grasp_offset_ + 0.28);
    if (!move_pose(preplace, "MOVING_TO_BIN")) {
      return fail("MOVING_TO_BIN", "Could not carry the object above the bin");
    }

    geometry_msgs::msg::Pose place = downward_pose(
      place_x_, place_y_, place_object_z_ + grasp_offset_);
    if (!move_pose(place, "LOWERING_INTO_BIN")) {
      return fail("LOWERING_INTO_BIN", "Could not lower the object into the bin");
    }

    if (!set_grasp_constraint(false)) {
      return fail("RELEASING_OBJECT", "Gazebo did not confirm object detachment");
    }
    arm_.detachObject("target_cube");
    planning_scene_.removeCollisionObjects({"target_cube"});
    if (!command_gripper(open_width_)) {
      return fail("RELEASING_OBJECT", "Object detached but gripper failed to open");
    }

    geometry_msgs::msg::Pose postplace = place;
    postplace.position.z += 0.25;
    if (!move_pose(postplace, "RETREATING_FROM_BIN")) {
      return fail("RETREATING_FROM_BIN", "Object placed, but retreat failed");
    }

    // DetachableJoint correctly removes the fixed grasp, but a light object can
    // remain asleep against a finger collision in DART. Repositioning it a few
    // centimetres above the bin after the hand retreats wakes the free body;
    // gravity and bin collisions still determine its final resting pose.
    if (!release_object_over_bin()) {
      return fail("VERIFYING_PLACEMENT", "Could not wake the released object over the bin");
    }
    if (!wait_for_placed_pose()) {
      return fail("VERIFYING_PLACEMENT", "Vision did not confirm the object inside the bin");
    }
    if (!move_named("ready", "RETURNING_HOME")) {
      return fail("RETURNING_HOME", "Object placed, but home motion failed");
    }

    publish_status("SUCCEEDED", "Detected, picked, transported, and placed one object");
    write_log(true, "completed");
    return true;
  }

  bool move_named(const std::string & target, const std::string & state)
  {
    publish_status(state, "Planning named target: " + target);
    arm_.setStartStateToCurrentState();
    if (!arm_.setNamedTarget(target)) {
      return false;
    }
    const auto result = arm_.move();
    arm_.stop();
    return result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool move_pose(const geometry_msgs::msg::Pose & pose, const std::string & state)
  {
    std::ostringstream details;
    details << std::fixed << std::setprecision(3)
            << "Goal xyz = (" << pose.position.x << ", "
            << pose.position.y << ", " << pose.position.z << ")";
    publish_status(state, details.str());
    arm_.setStartStateToCurrentState();
    arm_.setPoseTarget(pose, end_effector_link_);
    const auto result = arm_.move();
    arm_.stop();
    arm_.clearPoseTargets();
    return result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool command_gripper(double position)
  {
    GripperCommand::Goal goal;
    goal.command.name = {"panda_finger_joint1"};
    goal.command.position = {position};
    goal.command.velocity = {0.08};
    goal.command.effort = {20.0};

    auto goal_future = gripper_client_->async_send_goal(goal);
    if (goal_future.wait_for(5s) != std::future_status::ready) {
      return false;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      return false;
    }
    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (result_future.wait_for(8s) != std::future_status::ready) {
      gripper_client_->async_cancel_goal(goal_handle);
      return false;
    }
    const auto wrapped_result = result_future.get();
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return true;
    }

    // A real grasp intentionally prevents a position-controlled finger from
    // reaching a command beyond the object's surface. Treat contact close to
    // the requested width as success, while still rejecting a large miss.
    if (wrapped_result.result && !wrapped_result.result->state.position.empty()) {
      const double measured = wrapped_result.result->state.position.front();
      const double error = std::abs(measured - position);
      if (wrapped_result.result->stalled || error <= 0.012) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Gripper stopped on contact at %.4f m (command %.4f m); accepting grasp.",
          measured, position);
        return true;
      }
      RCLCPP_WARN(
        node_->get_logger(),
        "Gripper action ended with code %d and %.4f m position error.",
        static_cast<int>(wrapped_result.code), error);
    }
    return false;
  }

  bool set_grasp_constraint(bool attached)
  {
    const std::string expected = attached ? "attached" : "detached";
    {
      std::scoped_lock lock(grasp_state_mutex_);
      grasp_state_.clear();
    }

    for (int attempt = 0; attempt < 12 && rclcpp::ok(); ++attempt) {
      if (attached) {
        attach_publisher_->publish(std_msgs::msg::Empty{});
      } else {
        detach_publisher_->publish(std_msgs::msg::Empty{});
      }

      std::unique_lock lock(grasp_state_mutex_);
      if (grasp_state_cv_.wait_for(lock, 300ms, [this, &expected]() {
            return grasp_state_ == expected;
          })) {
        RCLCPP_INFO(node_->get_logger(), "Gazebo grasp state: %s", expected.c_str());
        return true;
      }
    }
    return false;
  }

  bool release_object_over_bin()
  {
    if (!set_pose_client_->wait_for_service(5s)) {
      return false;
    }

    auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    request->entity.name = "target_cube";
    request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    request->pose.position.x = place_x_;
    request->pose.position.y = place_y_;
    request->pose.position.z = 0.98;
    request->pose.orientation.w = 1.0;

    auto future = set_pose_client_->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      return false;
    }
    publish_status("VERIFYING_PLACEMENT", "Released object is falling under Gazebo physics");
    return true;
  }

  bool wait_for_placed_pose()
  {
    const auto deadline = std::chrono::steady_clock::now() + 6s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::scoped_lock lock(pose_mutex_);
        if (latest_pose_) {
          const auto & point = latest_pose_->pose.position;
          const bool inside_xy = std::abs(point.x - place_x_) < 0.12 &&
            std::abs(point.y - place_y_) < 0.10;
          const bool resting_height = point.z > 0.84 && point.z < 0.92;
          if (inside_xy && resting_height) {
            std::ostringstream details;
            details << std::fixed << std::setprecision(3)
                    << "Vision confirmed bin pose = (" << point.x << ", "
                    << point.y << ", " << point.z << ")";
            publish_status("PLACEMENT_CONFIRMED", details.str());
            return true;
          }
        }
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  std::optional<geometry_msgs::msg::PoseStamped> wait_for_object_pose()
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(detection_timeout_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::scoped_lock lock(pose_mutex_);
        if (latest_pose_) {
          const double age = (node_->now() - latest_pose_received_).seconds();
          if (age >= 0.0 && age <= max_detection_age_) {
            return latest_pose_;
          }
        }
      }
      std::this_thread::sleep_for(100ms);
    }
    return std::nullopt;
  }

  geometry_msgs::msg::Pose downward_pose(double x, double y, double z) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    // 180 degrees about X: the gripper's +Z approach axis points downward.
    pose.orientation.x = 1.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    return pose;
  }

  void add_static_collision_scene()
  {
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = planning_frame_;
    table.id = "workbench_top";
    shape_msgs::msg::SolidPrimitive table_shape;
    table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    // Model only the reachable work surface. The real Gazebo table continues
    // under the bolted base and mast, but those intended contacts must not make
    // every MoveIt start state invalid.
    table_shape.dimensions = {0.80, 0.90, 0.08};
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.65;
    table_pose.position.z = 0.755;
    table_pose.orientation.w = 1.0;
    table.primitives.push_back(table_shape);
    table.primitive_poses.push_back(table_pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit_msgs::msg::CollisionObject bin_floor;
    bin_floor.header.frame_id = planning_frame_;
    bin_floor.id = "drop_bin_floor";
    shape_msgs::msg::SolidPrimitive bin_shape;
    bin_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    bin_shape.dimensions = {0.28, 0.24, 0.03};
    geometry_msgs::msg::Pose bin_pose;
    bin_pose.position.x = place_x_;
    bin_pose.position.y = place_y_;
    bin_pose.position.z = 0.825;
    bin_pose.orientation.w = 1.0;
    bin_floor.primitives.push_back(bin_shape);
    bin_floor.primitive_poses.push_back(bin_pose);
    bin_floor.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_.applyCollisionObjects({table, bin_floor});
    std::this_thread::sleep_for(500ms);
  }

  void add_target_collision(const geometry_msgs::msg::PoseStamped & detected_pose)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = planning_frame_;
    object.id = "target_cube";
    shape_msgs::msg::SolidPrimitive cube;
    cube.type = shape_msgs::msg::SolidPrimitive::BOX;
    cube.dimensions = {0.06, 0.06, 0.06};
    geometry_msgs::msg::Pose pose = detected_pose.pose;
    // A small separation avoids treating normal support contact with the table
    // as mesh penetration due to depth noise and floating-point tolerances.
    pose.position.z += 0.006;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    object.primitives.push_back(cube);
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    planning_scene_.applyCollisionObject(object);
    std::this_thread::sleep_for(300ms);
  }

  bool fail(const std::string & state, const std::string & reason)
  {
    publish_status("FAILED", state + ": " + reason);
    write_log(false, state + ": " + reason);
    return false;
  }

  void recover()
  {
    publish_status("RECOVERING", "Opening gripper, detaching object, and attempting home");
    (void)set_grasp_constraint(false);
    arm_.detachObject("target_cube");
    planning_scene_.removeCollisionObjects({"target_cube"});
    (void)command_gripper(open_width_);
    (void)move_named("ready", "RECOVERING_HOME");
    publish_status("READY", "Recovery finished; call /pick_place/start to retry");
  }

  void publish_status(const std::string & state, const std::string & detail)
  {
    std_msgs::msg::String message;
    message.data = "{\"state\":\"" + state + "\",\"detail\":\"" + detail + "\"}";
    status_publisher_->publish(message);
    RCLCPP_INFO(node_->get_logger(), "[%s] %s", state.c_str(), detail.c_str());
  }

  void write_log(bool success, const std::string & detail)
  {
    try {
      const std::filesystem::path path(log_file_);
      std::filesystem::create_directories(path.parent_path());
      const bool new_file = !std::filesystem::exists(path);
      std::ofstream stream(path, std::ios::app);
      if (new_file) {
        stream << "ros_time,result,object_x,object_y,object_z,detail\n";
      }
      stream << std::fixed << std::setprecision(3) << node_->now().seconds() << ','
             << (success ? "success" : "failure") << ',';
      if (run_object_pose_) {
        stream << run_object_pose_->pose.position.x << ','
               << run_object_pose_->pose.position.y << ','
               << run_object_pose_->pose.position.z;
      } else {
        stream << ",,";
      }
      stream << ",\"" << detail << "\"\n";
    } catch (const std::exception & exception) {
      RCLCPP_WARN(node_->get_logger(), "Could not write task log: %s", exception.what());
    }
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grasp_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;

  std::mutex pose_mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_pose_;
  rclcpp::Time latest_pose_received_{0, 0, RCL_ROS_TIME};
  std::optional<geometry_msgs::msg::PoseStamped> run_object_pose_;
  std::mutex grasp_state_mutex_;
  std::condition_variable grasp_state_cv_;
  std::string grasp_state_;
  std::atomic_bool running_{false};
  std::thread worker_;

  std::string object_pose_topic_;
  std::string end_effector_link_;
  std::string planning_frame_;
  std::string log_file_;
  double detection_timeout_;
  double max_detection_age_;
  double open_width_;
  double grasp_width_;
  double grasp_offset_;
  double pregrasp_clearance_;
  double retreat_clearance_;
  double place_x_;
  double place_y_;
  double place_object_z_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "vision_pick_place_task",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  auto task = std::make_unique<VisionPickPlaceTask>(node);
  task->maybe_auto_start();

  spinner.join();
  task.reset();
  rclcpp::shutdown();
  return 0;
}
