# Kinematics-Aware Autonomous Navigation


A **production-ready** point-to-point navigation system for differential drive robots, demonstrating mastery of robotics fundamentals through a complete, tested implementation.

---

## Production Features

| Feature | Description |
|---------|-------------|
| **Obstacle Avoidance** | LiDAR-based detection with emergency stop, velocity scaling, and avoidance maneuvers |
| **Velocity Smoothing** | Acceleration/deceleration limits preventing motor stress and wheel slip |
| **Recovery Behaviors** | Automatic handling of stuck situations, timeouts, and oscillation detection |
| **Error Handling** | Input validation, graceful degradation, diagnostic publishing |
| **Comprehensive Tests** | 40+ unit tests covering all components |

---

## Project Structure

```
kinematic_nav_pkg/
├── include/kinematic_nav_pkg/
│   ├── pid_controller.hpp           # PID with anti-windup & derivative filter
│   ├── differential_drive_kinematics.hpp  # Forward/inverse kinematics
│   ├── obstacle_detector.hpp        # LiDAR-based obstacle detection
│   ├── velocity_smoother.hpp        # Acceleration limiting
│   └── recovery_manager.hpp         # Stuck/timeout recovery
├── src/
│   └── point_to_point_navigator.cpp # Production ROS2 node
├── test/
│   ├── test_pid_controller.cpp
│   ├── test_obstacle_detector.cpp
│   ├── test_velocity_smoother.cpp
│   └── test_recovery_manager.cpp
├── launch/
│   ├── navigation.launch.py
│   └── simulation.launch.py
├── config/
│   ├── navigator_params.yaml
│   └── navigation.rviz
├── CMakeLists.txt
└── package.xml

docs/
├── kinematics_theory.tex            # Mathematical foundations (LaTeX)
└── simulation_setup_guide.md        # Gazebo/RViz2 guide
```

---

## Competencies Demonstrated

### 1. Physics/Kinematics
- Forward/inverse kinematics for differential drive
- Unicycle model with nonholonomic constraints
- SE(2) state-space representation
- Exact arc integration for pose propagation

### 2. Automatic Control
- PID controller with anti-windup and derivative filtering
- Transfer function analysis for controller design
- State machine for navigation phases
- Feedback stabilization with disturbance rejection

### 3. C++ & OOP (Production-Grade)
- Modern C++17 (`[[nodiscard]]`, structured bindings, `std::optional`)
- RAII with smart pointers (`unique_ptr`, `shared_ptr`)
- Exception-safe design with try-catch blocks
- Clean separation of concerns (5 independent classes)

### 4. Real-time Planning
- 50Hz control loop with sensor fusion
- LiDAR integration for obstacle detection
- Dynamic velocity scaling based on proximity
- Recovery behaviors for edge cases

---

## Key Equations

**Forward Kinematics:**
$$v = \frac{r}{2}(\omega_R + \omega_L), \quad \omega = \frac{r}{L}(\omega_R - \omega_L)$$

**Inverse Kinematics:**
$$\omega_R = \frac{1}{r}\left(v + \frac{L}{2}\omega\right), \quad \omega_L = \frac{1}{r}\left(v - \frac{L}{2}\omega\right)$$

**PID Control Law:**
$$u(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{de(t)}{dt}$$

---

## Quick Start

```bash
# Build
cd ~/ros2_ws/src && cp -r kinematic_nav_pkg .
cd ~/ros2_ws && colcon build --packages-select kinematic_nav_pkg

# Run tests
colcon test --packages-select kinematic_nav_pkg
colcon test-result --verbose

# Launch simulation
source install/setup.bash
ros2 launch kinematic_nav_pkg simulation.launch.py

# Send a goal (in RViz2: click "2D Goal Pose")
```

---

## Architecture

```
                    ┌─────────────────────────────────────────┐
                    │        PointToPointNavigator            │
                    │                                         │
  /goal_pose ──────▶│  ┌─────────┐  ┌──────────────────┐     │
                    │  │  State  │  │  PIDController   │     │
  /odom ───────────▶│  │ Machine │  │  (distance, heading)   │──▶ /cmd_vel
                    │  └────┬────┘  └──────────────────┘     │
  /scan ───────────▶│       │                                 │
                    │  ┌────▼────┐  ┌──────────────────┐     │
                    │  │Obstacle │  │ VelocitySmoother │     │──▶ /navigator/status
                    │  │Detector │  │ (accel limits)   │     │
                    │  └─────────┘  └──────────────────┘     │
                    │                                         │
                    │  ┌─────────────────────────────────┐   │
                    │  │      RecoveryManager            │   │
                    │  │  (timeout, stuck, oscillation)  │   │
                    │  └─────────────────────────────────┘   │
                    └─────────────────────────────────────────┘
```

---

## Test Coverage

| Component | Tests | Coverage |
|-----------|-------|----------|
| PIDController | 13 | Gains, saturation, anti-windup, filtering |
| ObstacleDetector | 10 | Zones, emergency stop, avoidance |
| VelocitySmoother | 11 | Accel limits, emergency stop, reset |
| RecoveryManager | 10 | Timeout, stuck, abort, reset |

---

## Configuration

All parameters are configurable via `config/navigator_params.yaml`:

```yaml
# PID Gains
distance_kp: 1.0
heading_kp: 2.0

# Obstacle Avoidance
emergency_stop_distance: 0.15  # meters
slow_down_distance: 0.5

# Recovery
goal_timeout_sec: 60.0
stuck_timeout_sec: 5.0
max_recovery_attempts: 3
```

---

## Author

MSc AI & Robotics Applicant
Sapienza University of Rome, 2024
