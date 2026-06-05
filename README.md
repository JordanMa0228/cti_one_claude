# W102 ROS2 Path-Planning and Gazebo Simulation

This project implements a ROS 2 simulation for robot **W102** navigating in a **10 ft × 12 ft living room** with a **chair obstacle** between the robot and the user **John**.

The simulation includes:

- graph-based path planning
- waypoint-based navigation
- Gazebo world simulation
- RViz visualization
- obstacle avoidance around the chair
- right-side path execution from start to goal

## Scenario

### Room
- Size: **10 ft × 12 ft**
- Coordinate system: **2D top-down**
- Units in simulation are converted to **meters**

### Key positions
- **Start (S):** `(0, 0)`
- **Chair obstacle center:** `(0, 4)`
- **Goal / John (G):** `(0, 10)`

### Graph nodes
- `S  = (0, 0)`
- `L1 = (-3, 0)`
- `L2 = (-3, 6)`
- `R1 = (3, 0)`
- `R2 = (3, 6)`
- `G  = (0, 10)`

### Path selection
The direct path from `S` to `G` is blocked by the chair and its inflated safety region, so the robot must go around the obstacle.

The intended right-side graph path is:

`(0,0) -> (3,0) -> (3,6) -> (0,10)`

In Gazebo, waypoint/controller adjustments may be used so the robot can physically complete the route without colliding with the wall while still preserving the **right-side obstacle-avoidance behavior**.

## Repository structure

```text
CTI_One_interview/
├── build/
├── install/
├── log/
├── results/
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

## Main components

### Gazebo world
The Gazebo world defines:
- room floor and walls
- chair obstacle
- John goal marker
- robot W102 model
- physics and differential drive plugin

### ROS 2 nodes
The package includes nodes for:
- **Gazebo navigation** of W102 through waypoints
- **RViz marker visualization**
- **robot_state_publisher** for robot TF/URDF visualization
- **ROS ↔ Gazebo topic bridging** using `ros_gz_bridge`

## How to run

This is the command sequence used to build and launch the project:

```bash
cd ~/CTI_One_interview
source /opt/ros/kilted/setup.bash
colcon build --packages-select w102_path_planning
source install/setup.bash
ros2 launch w102_path_planning w102_gazebo.launch.py
```

## Optional launch variants

Launch without RViz:

```bash
ros2 launch w102_path_planning w102_gazebo.launch.py rviz:=false
```

## Expected behavior

When the launch succeeds:

- Gazebo opens the living room world
- RViz opens with markers and robot visualization
- W102 starts from the bottom side of the room
- the robot follows the right-side path around the chair
- the robot reaches John near the top of the room

Typical console output includes waypoint progress such as:
- waypoint reached at `R1`
- waypoint reached at `R2`
- robot reaches `G (John)`
- mission complete

## Notes

- The project is intended for **Ubuntu 24.04** with **ROS 2 Kilted**
- Gazebo GUI behavior under WSL may depend on display / graphics configuration
- RViz may show the path and markers more clearly than Gazebo
- Small controller or waypoint adjustments may be necessary to make the physical robot complete turns cleanly in Gazebo

## Troubleshooting

### If Gazebo opens but the robot does not move
Check:
- `ros_gz_bridge` is running
- `/odom` is being published
- `/cmd_vel` is being sent
- the diff drive plugin is configured correctly

### If RViz works but Gazebo looks wrong
Check:
- wheel joint axes in the SDF
- waypoint clearance near walls
- camera angle in the world file
- WSL / display setup

## Author
Project prepared for a ROS 2 simulation and obstacle-avoidance assignment using **Python**, **rclpy**, **Gazebo**, and **RViz**.
