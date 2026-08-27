# Vision-Guided Pick-and-Place Robot

A ROS 2 Jazzy simulation in which a Franka Emika Panda arm detects a red
workpiece with an overhead RGB-D camera, plans collision-free motions, picks the
object, places it in a bin, and verifies the final placement.

## System

```text
Gazebo RGB + depth + CameraInfo
              |
              v
    RGB-D object localizer -- /perception/object_pose --> task state machine
                                                           |          |
                                            MoveIt 2 plans |          | gripper
                                                           v          v
                                              ros2_control controllers
                                                           |
                                                           v
                                                    Gazebo Panda arm
```

The project uses Gazebo Sim for physics and sensors, `gz_ros2_control` for
simulated hardware, MoveIt 2 with OMPL for collision-aware arm planning, and
OpenCV for HSV segmentation and depth-based 3D localization. The task node is a
C++ state machine with recovery, status reporting, and CSV run logs.

## Packages

- `pick_place_description`: Panda Xacro, controllers, RViz config, and workcell.
- `pick_place_moveit_config`: SRDF, kinematics, limits, OMPL, and controllers.
- `pick_place_perception`: synchronized RGB-D detection and TF transformation.
- `pick_place_tasks`: autonomous C++ pick-and-place state machine.
- `pick_place_bringup`: one-command launch orchestration.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy Desktop
- Gazebo Harmonic / `ros_gz`
- MoveIt 2
- `ros2_control` and `gz_ros2_control`

Install package dependencies after cloning:

```bash
cd ~/project/manipulation_ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init                    # only if rosdep has never been initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python
```

## Build

```bash
cd ~/project/manipulation_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Start the complete system. Gazebo and RViz open, then one autonomous cycle
starts after the controllers, MoveIt, and perception are ready:

```bash
cd ~/project/manipulation_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pick_place_bringup pick_place.launch.py
```

To start without immediately running the task:

```bash
ros2 launch pick_place_bringup pick_place.launch.py auto_start:=false
```

Then trigger one cycle manually:

```bash
ros2 service call /pick_place/start std_srvs/srv/Trigger "{}"
```

Disable RViz on a slower computer with `rviz:=false`.

## Observe the system

```bash
# State transitions and details
ros2 topic echo /pick_place/status

# Detected 3D target pose in the world frame
ros2 topic echo /perception/object_pose

# Confirm the three hardware controllers are active
ros2 control list_controllers
```

Successful and failed runs are appended to `logs/pick_place_runs.csv`.

## Task sequence

1. Detach the simulation grasp constraint and move to `ready`.
2. Synchronize RGB and depth images and estimate the cube centre in `world`.
3. Add the table, bin, and workpiece to the MoveIt planning scene.
4. Plan to pre-grasp, descend, close the fingers, and attach the workpiece.
5. Lift, transport, lower, detach, open, and retreat.
6. Let Gazebo physics settle the object and confirm its bin pose using vision.
7. Return home, publish `SUCCEEDED`, and write the CSV result.

## License

Apache-2.0.
