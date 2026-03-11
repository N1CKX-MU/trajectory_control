# trajectory_control

ROS2 path smoothing and trajectory tracking for a Turtlebot3 Burger differential-drive robot, running in Gazebo Classic via Docker.

Implements three path smoothing algorithms (Catmull-Rom, Bezier, Gradient Descent), a trapezoidal velocity trajectory generator, a Pure Pursuit controller, and LiDAR-based obstacle detection — all visualised simultaneously in RViz2.

---

## Demo

```bash
git clone <your-repo-url>
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
        │   └── types.hpp                   # shared structs + waypoint loader
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

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     ROS2 Node Graph                     │
│                                                         │
│  waypoints_node  ──→  /waypoints                        │
│  catmullrom_node ──→  /path_catmullrom   (blue)         │
│  bezier_node     ──→  /path_bezier       (red)          │
│  gradient_node   ──→  /path_gradient     (cyan)         │
│                                                         │
│  trajectory_generator_node ──→ /trajectory              │
│                                                         │
│  obstacle_detector_node  ←── /scan (LiDAR)              │
│                         ──→ /detected_obstacles          │
│                                                         │
│  obstacle_avoider_node  ←── /path_catmullrom            │
│                         ←── /detected_obstacles          │
│                         ──→ /path_avoiding   (purple)   │
│                                                         │
│  controller_node  ←── /odom                             │
│                   ←── /path_avoiding                    │
│                   ──→ /cmd_vel                           │
│                   ──→ /actual_path                       │
│                   ──→ /tracking_error                    │
└─────────────────────────────────────────────────────────┘
```

---

## Path Smoothing Algorithms

### Catmull-Rom Spline (Blue)
Passes through every waypoint. Uses the previous and next waypoint as implicit control points to determine curvature at each point. Produces natural-looking curves with C1 continuity.

```
For segment i → i+1:
  p0 = waypoint[i-1]  (ghost point)
  p1 = waypoint[i]
  p2 = waypoint[i+1]
  p3 = waypoint[i+2]  (ghost point)

  x(t) = 0.5 * [(2p1) + (-p0+p2)t + (2p0-5p1+4p2-p3)t² + (-p0+3p1-3p2+p3)t³]
```

### Cubic Bezier (Red)
Does not pass through waypoints — control points pull the curve like magnets. Control points are computed automatically from neighbouring waypoints:

```
P1 = waypoint[i]   + 0.25 * (waypoint[i+1] - waypoint[i-1])
P2 = waypoint[i+1] - 0.25 * (waypoint[i+2] - waypoint[i])

x(t) = (1-t)³P0 + 3(1-t)²t·P1 + 3(1-t)t²·P2 + t³P3
```

### Gradient Descent (Cyan)
Starts with straight lines between waypoints and iteratively minimises a cost function balancing smoothness against deviation from the original path:

```
cost = α * smoothness + (1-α) * distance_from_original

smoothness gradient: p[i-1] - 2*p[i] + p[i+1]   (discrete Laplacian)
distance gradient:   original[i] - current[i]

update: p[i] += lr * (α * smooth_grad + (1-α) * dist_grad)
```

---

## Trajectory Generation

Takes the Catmull-Rom smooth path and computes a time-stamped trajectory with a trapezoidal velocity profile:

```
Phase 1 — Accelerate:  v = sqrt(2 * a * s)
Phase 2 — Cruise:      v = v_max
Phase 3 — Decelerate:  v = sqrt(2 * a * (total_length - s))

v_actual = min(v_max, v_ramp_up, v_ramp_down)

Timestamps computed from arc length and average velocity between consecutive points.
```

Publishes on `/trajectory` as `nav_msgs/Path` and velocity profile on `/velocity_profile`.

---

## Pure Pursuit Controller

Tracks the trajectory by looking ahead a fixed distance along the path and steering towards that point:

```
Adaptive lookahead:  L = clamp(k * v, L_min, L_max)

Find lookahead point: first trajectory point further than L from robot

alpha = angle between robot heading and direction to lookahead point
curvature = 2 * sin(alpha) / L
omega = v * curvature
```

Publishes tracking error on `/tracking_error`:
- `data[0]` — cross track error (metres)
- `data[1]` — heading error (radians)
- `data[2]` — linear velocity command
- `data[3]` — angular velocity command

---

## Obstacle Detection

The Turtlebot3 Burger's built-in 360° LiDAR (`/scan`, 3.5m range) is used to detect obstacles at runtime:

1. Convert each valid laser return from polar to Cartesian coordinates in the robot frame
2. Group adjacent scan points into clusters using sequential clustering (only groups points that are adjacent in angular order)
3. Discard clusters with fewer than 8 points (noise rejection)
4. Compute the centroid and radius of each cluster
5. Transform cluster positions from `base_scan` to `odom` frame using TF2
6. Publish as cylinder markers on `/detected_obstacles`

---

## Obstacle Avoidance

Uses a geometric visibility-based approach — deterministic, no iteration, no side-flickering:

1. For each path segment, check if it intersects any detected obstacle (circle-segment intersection)
2. If it does, compute a bypass point perpendicular to the path direction on the safe side
3. The safe side is determined by the cross product of the path direction and the obstacle direction — computed once from the original path so it never changes
4. The bypass point is placed at `(obstacle_radius + safe_margin) * 1.2` from the obstacle centre
5. The resulting path is smoothed with Catmull-Rom

---

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

## 2.2 Design Choices & Algorithms

<!-- TODO: Write in your own words -->
<!-- PLACEHOLDER: Explain why you chose Pure Pursuit over PID, why Catmull-Rom as primary smoother,
     why separate nodes instead of one monolithic node, why Docker for deployment -->

---

## 2.3 Extending to a Real Robot

<!-- TODO: Write in your own words -->
<!-- PLACEHOLDER: Explain sensor calibration, replacing /odom with real odometry + IMU fusion,
     replacing Gazebo LiDAR with real RPLidar A1, tuning lookahead gain for real wheel slip,
     safety stops, real-time constraints -->

---

## 2.4 AI Tools Used

<!-- TODO: Write in your own words -->
<!-- PLACEHOLDER: Mention Claude was used for code generation and debugging throughout development.
     Be specific about what was AI-assisted vs what you understood and directed -->

---

## 2.5 Obstacle Avoidance Extension

<!-- TODO: Write in your own words -->
<!-- PLACEHOLDER: Explain the LiDAR-based detection pipeline, the gradient descent + repulsion
     approach, the visibility graph bypass approach, why static replanning was chosen,
     and how dynamic replanning would work with a replanning loop -->