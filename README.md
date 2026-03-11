# trajectory_control

ROS2 path smoothing and trajectory tracking for a Turtlebot3 Burger differential-drive robot, running in Gazebo Classic via Docker.

Implements three path smoothing algorithms (Catmull-Rom, Bezier, Gradient Descent), a trapezoidal velocity trajectory generator, a Pure Pursuit controller, and LiDAR-based obstacle detection — all visualised simultaneously in RViz2.

---

## Demo

```bash
git clone https://github.com/N1CKX-MU/trajectory_control.git
cd trajectory_control
xhost +local:docker
make docker        # starts container and drops you inside
make demo          # builds + launches everything
```

Gazebo opens with the Turtlebot3 in an obstacle course. RViz2 opens showing all three smooth paths, the reference trajectory, and the robot tracking it in real time.

---

## Environment

| Component | Version |
|-----------|---------|
| Host OS | Ubuntu 24.04 |
| Container OS | Ubuntu 22.04 (Jammy) |
| ROS2 | Humble |
| Gazebo | Classic 11 |
| Robot | Turtlebot3 Burger |
| GPU | Nvidia (passthrough via nvidia-container-toolkit) |

---

## Repository Structure

```
trajectory_control/
├── docker/
│   ├── Dockerfile          # ROS2 Humble + Gazebo Classic + TB3 + Nvidia GL
│   └── entrypoint.sh       # sources ROS2, sets env vars, auto builds workspace
├── docker-compose.yml      # Nvidia passthrough, X11 forwarding, src volume mount
├── Makefile                # host-side commands (docker up/down/build)
└── src/
    └── trajectory_controller/
        ├── CMakeLists.txt
        ├── package.xml
        ├── Makefile                        # container-side commands (build/demo/run)
        ├── config/
        │   └── params.yaml                 # all tunable parameters
        ├── include/trajectory_controller/
        │   ├── types.hpp                   # shared structs + waypoint loader
        |   └── viz_utils.hpp
        ├── msg/
        │   └── segment.msg
        ├── launch/
        │   ├── demo.launch.py              # full demo (Gazebo + all nodes + RViz2)
        │   ├── demo_with_obstacles.launch.py  # demo with obstacle avoidance
        │   └── graph_compare.launch.py     # smoother comparison (no Gazebo)
        ├── rviz/
        │   └── visualize_trajectory.rviz   # pre-configured RViz2 layout
        ├── src/
        │   ├── waypoints.cpp               # publishes waypoints as markers
        │   ├── catmull.cpp                 # Catmull-Rom smoother
        │   ├── bezier_node.cpp             # Cubic Bezier smoother
        │   ├── gradient_smoothing.cpp      # Gradient Descent smoother
        │   ├── trajectory_generator.cpp    # trapezoidal velocity profile
        │   ├── controller.cpp              # Pure Pursuit controller
        │   ├── obstacle_detector.cpp       # LiDAR clustering → obstacle positions
        │   └── obstacle_avoider.cpp        # path deformation around obstacles
        ├── test/
        │   └──test_smoother.cpp
        └── worlds/
            └── obstacle_course.world       # Gazebo world with cylindrical obstacles
```

---

## Quick Start

### Prerequisites

- Docker + Docker Compose
- Nvidia Container Toolkit (`nvidia-container-toolkit`)
- X11 display forwarding

### First time setup

```bash
# Build the Docker image
make docker-build

# Allow Docker to use your display
xhost +local:docker
```

### Running

```bash
# Start container (host terminal)
make docker

# Inside container — full demo
make demo

# Inside container — smoother comparison only (no Gazebo, no robot)
make graph_compare

# Inside container — demo with obstacle avoidance
make demo_obstacles
```

---

### Outputs 

```bash
make demo
```
![OUTPUT of make demo](images/demo_launch.png)

```bash
make demo_obstacles
```
![OUTPUT of make demo_obstacles](images/demo_obstacles.png)


```bash
make graph_compare
```
![OUTPUT of make graph_compare](images/visualize_curves.png)

## Configuration

All tunable parameters are in `config/params.yaml`. No recompilation needed — just edit and relaunch.

```yaml
trajectory_controller:
  ros__parameters:

    waypoints: [0.0,0.0, 1.0,0.5, 2.0,2.0, 3.0,2.5, 4.0,1.5, 5.0,0.5, 6.0,1.0]

    smoother:
      samples_per_segment: 20
      bezier_tension: 0.25
      gradient_alpha: 0.8
      gradient_learning_rate: 0.1
      gradient_iterations: 500

    trajectory:
      max_velocity: 0.18      # m/s — Turtlebot3 Burger hardware limit
      acceleration: 0.05      # m/s²

    controller:
      lookahead_gain: 1.5
      min_lookahead: 0.15     # metres
      max_lookahead: 0.60     # metres
      goal_tolerance: 0.10    # metres

    obstacle_detector:
      cluster_threshold: 0.25
      min_cluster_points: 8

    obstacle_avoider:
      safe_margin: 0.45       # metres beyond obstacle radius
```

---

## RViz2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/waypoints` | MarkerArray | Green spheres + yellow connecting line |
| `/path_catmullrom` | MarkerArray | Blue smooth curve |
| `/path_bezier` | MarkerArray | Red smooth curve |
| `/path_gradient` | MarkerArray | Orange smooth curve |
| `/trajectory` | Path | Reference trajectory with heading arrows |
| `/actual_path` | Path | Robot's actual travelled path |
| `/detected_obstacles` | MarkerArray | Red cylinders at detected obstacle positions |
| `/path_avoiding` | MarkerArray | Purple obstacle-avoided path |
| `/odom` | Odometry | Robot pose |

---

## Makefile Reference

### Host (run from `~/trajectory_control/`)

| Command | Description |
|---------|-------------|
| `make docker` | Start container and attach shell |
| `make docker-build` | Rebuild Docker image |
| `make docker-down` | Stop and remove container |

### Container (run from `/ros2_ws/src/trajectory_controller/`)

| Command | Description |
|---------|-------------|
| `make build` | Build the ROS2 package & source the ws|
| `make demo` | Full demo -> Gazebo + all nodes + RViz2 |
| `make demo_obstacles` | Demo with obstacle avoidance |
| `make graph_compare` | Smoother comparison -> no Gazebo, no robot, just RViz |
| `make controller` | Run controller node only |
| `make topics` | List all active ROS2 topics |
| `make nodes` | List all active ROS2 nodes |
| `make tf` | Visualise TF tree |

---


## Troubleshooting

**Gazebo hangs on startup**
```bash
chmod 700 /tmp/runtime-root
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models
```

**Robot falls through the ground**
Make sure `GAZEBO_MODEL_PATH` includes `/usr/share/gazebo-11/models` so the ground plane model is found.

**RViz2 shows no data**
Check fixed frame is set to `odom`. Run `make topics` to verify nodes are publishing.

**Controller prints "Waiting for avoided path..."**
Start `obstacle_detector_node` and `obstacle_avoider_node` before the controller. The controller waits until a valid path is received before moving.


---

