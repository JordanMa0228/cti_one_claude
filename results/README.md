# W102 Path Planning Simulation — Results

## Room Setup

| Parameter | Value |
|-----------|-------|
| Room size | 10 ft × 12 ft |
| Coordinate system | 2D top-down (origin at robot start) |
| Robot (W102) footprint | 2.25 ft × 2.67 ft (27 in × 32 in) |
| Obstacle safety margin | 1.75 ft (inflated around chair centre) |

| Entity | Position (ft) |
|--------|--------------|
| W102 start | (0, 0) |
| Chair obstacle | (0, 4) |
| John (goal) | (0, 10) |

---

## Graph Nodes and Edges

Six waypoints define the navigation graph:

| Label | Position | Description |
|-------|----------|-------------|
| S | (0, 0) | Robot start |
| L1 | (−3, 0) | Left detour entry |
| L2 | (−3, 6) | Left detour apex |
| R1 | (3, 0) | Right detour entry |
| R2 | (3, 6) | Right detour apex |
| G | (0, 10) | Goal — John's position |

Edges and Euclidean costs:

```
S  -> L1  :  3.00 ft      S  -> R1  :  3.00 ft
L1 -> L2  :  6.00 ft      R1 -> R2  :  6.00 ft
L2 -> G   :  5.00 ft      R2 -> G   :  5.00 ft
```

---

## Why the Direct Path is Blocked

The chair is centred at (0, 4), directly on the straight line from S (0, 0) to G (0, 10).
With a 1.75 ft safety margin the obstacle inflates to a circle of radius 1.75 ft around (0, 4),
completely blocking any path within ±1.75 ft of x = 0 in that y-range.
The direct S → G segment passes through this inflated region, so it is removed from the graph.

---

## Chosen Final Trajectory

```
(0, 0)  →  (3, 0)  →  (3, 6)  →  (0, 10)
  S           R1          R2          G
```

Both the left and right detours share an identical Euclidean cost of **14.00 ft**.
The right-side path is selected as the preferred trajectory by design (right-side bias,
consistent with W102's default behavioural policy).

### Path cost breakdown

| Segment | Distance |
|---------|---------|
| S → R1 | 3.00 ft |
| R1 → R2 | 6.00 ft |
| R2 → G | 5.00 ft |
| **Total** | **14.00 ft** |

---

## Output Files

| File | Description |
|------|-------------|
| `trajectory_result.png` | Static top-down plot: room, obstacle, graph, planned path |
| `trajectory_points.csv` | Waypoints in order (step, label, x_ft, y_ft) |
| `w102_simulation.gif` | Animated simulation of W102 moving along the trajectory |

---

## How to Run the ROS2 Node

### Prerequisites

- ROS 2 Kilted (or later) installed at `/opt/ros/kilted`
- Python packages: `rclpy`, `matplotlib`, `numpy`, `Pillow`

### Build

```bash
cd ~/CTI_One_interview
source /opt/ros/kilted/setup.bash
colcon build --packages-select w102_path_planning
source install/setup.bash
```

### Run (single node)

```bash
export W102_RESULTS_DIR=~/CTI_One_interview/results
ros2 run w102_path_planning w102_path_sim
```

### Run via launch file

```bash
ros2 launch w102_path_planning w102_path_planning.launch.py
```

The node will:
1. Log the room setup, graph nodes, and Dijkstra result to the terminal.
2. Simulate W102 moving waypoint by waypoint, publishing each position on `/w102/current_waypoint`.
3. Save `trajectory_result.png`, `trajectory_points.csv`, and `w102_simulation.gif` to `$W102_RESULTS_DIR`.

### Monitor live topics (in a second terminal)

```bash
source /opt/ros/kilted/setup.bash
ros2 topic echo /w102/current_waypoint
ros2 topic echo /w102/status
```

---

## Package Structure

```
CTI_One_interview/
├── src/
│   └── w102_path_planning/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       │   └── w102_path_planning
│       ├── launch/
│       │   └── w102_path_planning.launch.py
│       └── w102_path_planning/
│           ├── __init__.py
│           └── w102_path_sim.py
└── results/
    ├── trajectory_result.png
    ├── trajectory_points.csv
    ├── w102_simulation.gif
    └── README.md
```
