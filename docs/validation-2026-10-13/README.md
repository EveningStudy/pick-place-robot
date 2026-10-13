# Simulation test

**18 of 20 cycles completed successfully (90%).**

## Setup

Each run started a fresh simulation on Ubuntu 24.04 in WSL2 with ROS 2 Jazzy.
Gazebo ran headless with its camera sensors active. Each run had a separate
ROS domain and Gazebo partition. The workcell, cube position and lighting
were the same throughout the test.

Grasping uses Gazebo's detachable-joint constraint. After release, a helper
repositions the cube above the bin so it can settle before the camera check.
The results cover this assisted simulation setup.

## Success criteria

The camera must confirm the cube is inside the bin in three distinct, fresh
frames. Its centre must be within 0.12 m in X and 0.10 m in Y of (0.48, -0.40),
with Z between 0.84 m and 0.92 m. The check allows six simulation seconds,
capped at 20 wall-clock seconds. The arm must then return home and record
`SUCCEEDED` and a successful CSV cycle.

## Results

All 20 launches count. Run 15 timed out during camera confirmation, and run 16
timed out waiting for the controllers or current robot state. Both need further
investigation. The other 18 cycles passed.

The Wilson 95% confidence interval is 69.9%–97.21%. The sample covers repeated
runs in one scene. Different object positions, lighting and physical hardware
have yet to be tested. The machine was shared during testing, so run times
depend on its load.

See [individual trials](trials.csv) and the [result summary](summary.json).
