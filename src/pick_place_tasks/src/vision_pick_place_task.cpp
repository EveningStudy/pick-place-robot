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
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
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
  using ArmTrajectory = control_msgs::action::FollowJointTrajectory;

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
    arm_controller_client_ = rclcpp_action::create_client<ArmTrajectory>(
      node_, "/panda_arm_controller/follow_joint_trajectory");
    set_pose_client_ = node_->create_client<ros_gz_interfaces::srv::SetEntityPose>(
      "/world/pick_place_workcell/set_pose");
    state_validity_client_ = node_->create_client<moveit_msgs::srv::GetStateValidity>(
      "/check_state_validity");

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
      if (!success) {
        recover();
      }
      running_ = false;
    });
  }

  bool run_once()
  {
    run_object_pose_.reset();
    publish_status("INITIALIZING", "Waiting for simulation interfaces");

    // Launch timers are only a convenience, not a readiness guarantee on a
    // busy machine. Never plan before both hardware action servers exist.
    if (!arm_controller_client_->wait_for_action_server(60s) ||
        !gripper_client_->wait_for_action_server(30s) || !arm_.getCurrentState(10.0)) {
      return fail("INITIALIZING", "Controllers or current robot state did not become ready");
    }

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
    if (!move_linear(grasp, "APPROACHING_OBJECT")) {
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
    if (!move_linear(retreat, "LIFTING_OBJECT")) {
      return fail("LIFTING_OBJECT", "Could not lift the attached object");
    }

    // Place the OBJECT at the requested height, not the hand at a nominal
    // grasp offset. Controller tolerances change the actual grasp transform;
    // assuming a fixed offset can put the attached collision box into the bin.
    const auto place_target = placement_hand_pose();
    if (!place_target) {
      return fail("MOVING_TO_BIN", "Could not read the attached workpiece transform");
    }
    geometry_msgs::msg::Pose place = *place_target;
    geometry_msgs::msg::Pose preplace = place;
    preplace.position.z += 0.28;
    if (!move_pose(preplace, "MOVING_TO_BIN")) {
      return fail("MOVING_TO_BIN", "Could not carry the object above the bin");
    }

    if (!move_linear(place, "LOWERING_INTO_BIN")) {
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
    if (!move_linear(postplace, "RETREATING_FROM_BIN")) {
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

  std::optional<geometry_msgs::msg::Pose> placement_hand_pose()
  {
    const auto objects = planning_scene_.getAttachedObjects({"target_cube"});
    const auto found = objects.find("target_cube");
    if (found == objects.end()) {
      return std::nullopt;
    }
    const auto & attached = found->second;
    if (attached.link_name != end_effector_link_ ||
        attached.object.header.frame_id != end_effector_link_ ||
        attached.object.primitive_poses.size() != 1) {
      return std::nullopt;
    }
    const auto & origin = attached.object.pose;
    const auto & shape = attached.object.primitive_poses.front();
    const Eigen::Quaterniond rotation(origin.orientation.w, origin.orientation.x,
      origin.orientation.y, origin.orientation.z);
    const Eigen::Vector3d offset = Eigen::Vector3d(origin.position.x, origin.position.y,
      origin.position.z) + rotation * Eigen::Vector3d(shape.position.x, shape.position.y,
      shape.position.z);
    if (!offset.allFinite() || offset.norm() > 0.30) {
      return std::nullopt;
    }
    auto target = downward_pose(place_x_, place_y_, place_object_z_);
    const Eigen::Quaterniond hand_rotation(target.orientation.w, target.orientation.x,
      target.orientation.y, target.orientation.z);
    const Eigen::Vector3d world_offset = hand_rotation * offset;
    target.position.x -= world_offset.x();
    target.position.y -= world_offset.y();
    target.position.z -= world_offset.z();
    RCLCPP_INFO(node_->get_logger(),
      "Measured grasp offset=(%.6f, %.6f, %.6f); placement hand=(%.6f, %.6f, %.6f)",
      offset.x(), offset.y(), offset.z(), target.position.x, target.position.y, target.position.z);
    return target;
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

  bool move_linear(const geometry_msgs::msg::Pose & pose, const std::string & state)
  {
    // Short vertical motions should continue the current IK branch. Re-sampling
    // an unconstrained OMPL goal here can choose a different redundant-arm
    // posture or fail to sample the near-contact placement pose at all.
    publish_status(state, "Planning collision-checked Cartesian segment");
    arm_.clearPoseTargets();
    const auto start = arm_.getCurrentState(2.0);
    if (!start) {
      return false;
    }
    arm_.setStartState(*start);
    if (state == "LOWERING_INTO_BIN") {
      const auto attached = planning_scene_.getAttachedObjects({"target_cube"});
      for (const auto & entry : attached) {
        const auto & origin = entry.second.object.pose;
        RCLCPP_INFO(node_->get_logger(), "Attached origin xyz=(%.6f, %.6f, %.6f)",
          origin.position.x, origin.position.y, origin.position.z);
        for (const auto & p : entry.second.object.primitive_poses) {
          RCLCPP_INFO(node_->get_logger(), "Attached cube frame=%s link=%s xyz=(%.6f, %.6f, %.6f)",
            entry.second.object.header.frame_id.c_str(), entry.second.link_name.c_str(),
            p.position.x, p.position.y, p.position.z);
        }
      }
    }
    moveit_msgs::msg::RobotTrajectory message;
    moveit_msgs::msg::MoveItErrorCodes error;
    const double fraction = arm_.computeCartesianPath({pose}, 0.005, message, true, &error);
    RCLCPP_INFO(node_->get_logger(), "%s Cartesian fraction %.6f, error %d",
      state.c_str(), fraction, error.val);
    if (error.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || fraction < 1.0 - 1e-6 ||
        message.joint_trajectory.points.size() < 2) {
      RCLCPP_WARN(node_->get_logger(), "Rejecting incomplete Cartesian path; trying collision-checked OMPL");
      return move_pose(pose, state + "_OMPL_FALLBACK");
    }
    // The modern Cartesian API no longer has a jump-threshold argument. Reject
    // discontinuous IK branches explicitly before any trajectory is executed.
    const auto & points = message.joint_trajectory.points;
    for (std::size_t i = 1; i < points.size(); ++i) {
      for (std::size_t j = 0; j < points[i].positions.size(); ++j) {
        if (std::abs(points[i].positions[j] - points[i - 1].positions[j]) > 0.25) {
          RCLCPP_WARN(node_->get_logger(), "Rejecting Cartesian joint jump at waypoint %zu", i);
          return move_pose(pose, state + "_OMPL_FALLBACK");
        }
      }
    }
    // Cartesian paths require explicit timing; preserve the original velocity
    // and acceleration limits. Recheck the retimed waypoints because TOTG may
    // deviate slightly from the original path during resampling.
    robot_trajectory::RobotTrajectory trajectory(arm_.getRobotModel(), arm_.getName());
    trajectory.setRobotTrajectoryMsg(*start, message);
    trajectory_processing::TimeOptimalTrajectoryGeneration timing(0.001, 0.02);
    if (!timing.computeTimeStamps(trajectory, arm_.getMaxVelocityScalingFactor(),
        arm_.getMaxAccelerationScalingFactor())) {
      return false;
    }
    if (!state_validity_client_->wait_for_service(2s)) {
      return false;
    }
    for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
      auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      request->group_name = arm_.getName();
      moveit::core::robotStateToRobotStateMsg(trajectory.getWayPoint(i), request->robot_state, false);
      request->robot_state.is_diff = true;  // Keep the scene's attached workpiece.
      auto checked = state_validity_client_->async_send_request(request);
      if (checked.wait_for(2s) != std::future_status::ready) {
        return false;
      }
      const auto validity = checked.get();
      if (!validity->valid) {
        RCLCPP_WARN(node_->get_logger(), "Rejecting invalid retimed waypoint %zu", i);
        for (const auto & contact : validity->contacts) {
          RCLCPP_WARN(node_->get_logger(), "Retimed collision: %s / %s depth %.6f",
            contact.contact_body_1.c_str(), contact.contact_body_2.c_str(), contact.depth);
        }
        return false;
      }
    }
    trajectory.getRobotTrajectoryMsg(message);
    const auto result = arm_.execute(message);
    arm_.stop();
    arm_.setStartStateToCurrentState();
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
    // Give physics/sensors six simulation seconds, bounded by 20 wall seconds
    // if the simulator stalls. Software rendering can run below real time.
    const auto started = node_->now();
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    rclcpp::Time last_sample(0, 0, RCL_ROS_TIME);
    int confirmed_frames = 0;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
        (node_->now() - started).seconds() < 6.0) {
      {
        std::scoped_lock lock(pose_mutex_);
        if (latest_pose_) {
          const auto & point = latest_pose_->pose.position;
          const bool inside_xy = std::abs(point.x - place_x_) < 0.12 &&
            std::abs(point.y - place_y_) < 0.10;
          const bool resting_height = point.z > 0.84 && point.z < 0.92;
          const rclcpp::Time stamp(latest_pose_->header.stamp, RCL_ROS_TIME);
          const double age = (node_->now() - stamp).seconds();
          if (stamp > last_sample) {
            confirmed_frames = (stamp > started && age >= 0.0 &&
              age <= max_detection_age_ && inside_xy && resting_height) ? confirmed_frames + 1 : 0;
            last_sample = stamp;
          }
          if (confirmed_frames >= 3) {
            std::ostringstream details;
            details << std::fixed << std::setprecision(3)
                    << "Three fresh frames confirmed bin pose = (" << point.x << ", "
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
  rclcpp_action::Client<ArmTrajectory>::SharedPtr arm_controller_client_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grasp_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_client_;
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
