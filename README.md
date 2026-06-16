# W102 ROS2 Path Planning and Gazebo Simulation

This repository contains a ROS 2 Python package for simulating robot **W102** navigating in a **10 ft × 12 ft living room** with a chair obstacle between the robot and the user **John**.

The project includes both:

- a 2D top-down path-planning simulation, and
- a 3D Gazebo simulation with RViz visualization.

## Package

- `w102_path_planning`

## Main Nodes

- `w102_path_sim.py` — 2D path-planning / result-generation node
- `w102_gazebo_nav.py` — Gazebo navigation controller using odometry feedback

## Scenario Setup

- Room size: `10 ft × 12 ft`
- Start position `S`: `(0, 0)`
- Chair obstacle center: `(0, 4)`
- Goal position `G` / John: `(0, 10)`
- Robot footprint: approximately `2.25 ft × 2.67 ft`
- Inflated obstacle safety margin: `1.75 ft`

## Graph Nodes

- `S  = (0, 0)`
- `L1 = (-3, 0)`
- `L2 = (-3, 6)`
- `R1 = (3, 0)`
- `R2 = (3, 6)`
- `G  = (0, 10)`

## Edges

- `S -> L1`
- `L1 -> L2`
- `L2 -> G`
- `S -> R1`
- `R1 -> R2`
- `R2 -> G`

Using Euclidean edge cost:

- `S -> R1 = 3 ft`
- `R1 -> R2 = 6 ft`
- `R2 -> G = 5 ft`
- Total right path cost = `14 ft`

The left path has the same total cost. The direct path from `S` to `G` is treated as blocked because it intersects the inflated chair obstacle region.

## Chosen Final Trajectory

The simulation uses the right-side path as the final trajectory:

```text
(0,0) -> (3,0) -> (3,6) -> (0,10)
```

In Gazebo, the waypoints are converted to meters and adjusted for the physical room, robot footprint, wall clearance, and chair clearance.

## Repository Structure

```text
.
├── results/
│   ├── README.md
│   ├── trajectory_points.csv
│   ├── trajectory_result.png
│   └── w102_simulation.gif
└── src/
    └── w102_path_planning/
        ├── config/
        ├── launch/
        ├── resource/
        ├── rviz/
        ├── urdf/
        ├── w102_path_planning/
        └── worlds/
```

## Requirements

- Ubuntu 24.04
- ROS 2 Kilted
- Python 3
- `rclpy`
- `numpy`
- `matplotlib`
- `Pillow`
- Gazebo / `ros_gz_bridge` for the 3D simulation

## Build

From the workspace root:

```bash
source /opt/ros/kilted/setup.bash
colcon build --packages-select w102_path_planning
source install/setup.bash
```

If your workspace is named `CTI_One_interview`, the full sequence is:

```bash
cd ~/CTI_One_interview
source /opt/ros/kilted/setup.bash
colcon build --packages-select w102_path_planning
source install/setup.bash
```

## Run the 2D Path-Planning Node

```bash
export W102_RESULTS_DIR=~/CTI_One_interview/results
ros2 run w102_path_planning w102_path_sim
```

Or run it through the launch file:

```bash
ros2 launch w102_path_planning w102_path_planning.launch.py
```

The node generates the following files in `results/`:

- `trajectory_result.png`
- `trajectory_points.csv`
- `w102_simulation.gif`
- `README.md`

## Run the Gazebo Simulation

```bash
ros2 launch w102_path_planning w102_gazebo.launch.py
```

Launch without RViz:

```bash
ros2 launch w102_path_planning w102_gazebo.launch.py rviz:=false
```

Expected behavior:

1. Gazebo opens the living-room world.
2. RViz opens with markers and robot visualization.
3. W102 starts near the bottom of the room.
4. W102 follows the right-side route around the chair.
5. W102 reaches John near the top of the room.

## Monitor Topics

In another terminal:

```bash
source /opt/ros/kilted/setup.bash
ros2 topic echo /w102/current_waypoint
ros2 topic echo /w102/status
ros2 topic echo /odom
ros2 topic echo /cmd_vel
```

## Troubleshooting

If RViz shows motion but Gazebo does not move, check:

- `ros_gz_bridge` is running,
- `/cmd_vel` is being published,
- `/odom` is being published,
- the Gazebo differential-drive plugin is configured correctly,
- wheel joint axes and orientation match the robot model,
- the room/world frame and odom frame use the same coordinate assumptions.

If Gazebo opens but the robot appears stuck, also inspect the robot spawn pose, yaw direction, wall clearance, and whether the controller is publishing velocity commands to the exact topic consumed by the Gazebo plugin.
