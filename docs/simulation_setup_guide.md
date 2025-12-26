# Simulation Setup Guide
## Gazebo & RViz2 for Kinematics-Aware Navigation

This guide explains how to simulate the differential drive navigation system
using Gazebo (physics simulation) and RViz2 (visualization).

---

## 1. Prerequisites

### ROS2 Installation (Ubuntu 22.04 - Humble)

```bash
# Install ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS2
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### TurtleBot3 Packages

```bash
# Install TurtleBot3 simulation packages
sudo apt install ros-humble-turtlebot3-gazebo \
                 ros-humble-turtlebot3-description \
                 ros-humble-turtlebot3-teleop

# Set the robot model
export TURTLEBOT3_MODEL=burger
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

### Gazebo Installation

```bash
# Gazebo Classic (included with ros-humble-desktop)
# OR Gazebo Ignition (recommended for new projects)
sudo apt install ros-humble-ros-gz
```

---

## 2. Build the Package

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy the kinematic_nav_pkg here
cp -r /path/to/kinematic_nav_pkg .

# Build
cd ~/ros2_ws
colcon build --packages-select kinematic_nav_pkg
source install/setup.bash
```

---

## 3. Launch Simulation

### Option A: Full Simulation (Gazebo + RViz2 + Navigator)

```bash
# Terminal 1: Launch everything
ros2 launch kinematic_nav_pkg simulation.launch.py
```

### Option B: Step-by-Step Launch

```bash
# Terminal 1: Launch Gazebo with TurtleBot3
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch RViz2
rviz2 -d ~/ros2_ws/src/kinematic_nav_pkg/config/navigation.rviz

# Terminal 3: Launch Navigator
ros2 launch kinematic_nav_pkg navigation.launch.py use_sim_time:=true
```

---

## 4. Sensor Data Flow

### Odometry (/odom)
- **Source**: Gazebo differential drive plugin
- **Message Type**: `nav_msgs/msg/Odometry`
- **Content**: Robot pose (x, y, theta) and velocity (v, omega)
- **Frame**: `odom` → `base_footprint`

```bash
# View odometry data
ros2 topic echo /odom
```

### LiDAR Scan (/scan)
- **Source**: Gazebo ray sensor plugin
- **Message Type**: `sensor_msgs/msg/LaserScan`
- **Content**: 360° range measurements
- **Frame**: `base_scan`

```bash
# View LiDAR data
ros2 topic echo /scan --once
```

### Velocity Commands (/cmd_vel)
- **Destination**: Gazebo differential drive plugin
- **Message Type**: `geometry_msgs/msg/Twist`
- **Content**: Linear.x (forward velocity), Angular.z (turning rate)

```bash
# Manually send velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

---

## 5. Sending Navigation Goals

### Using RViz2 (Recommended)
1. Click the "2D Goal Pose" button in the toolbar
2. Click and drag on the map to set position and orientation
3. The navigator will receive the goal on `/goal_pose`

### Using Command Line

```bash
# Send a goal to position (2.0, 1.0) with 0° orientation
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, \
    pose: {position: {x: 2.0, y: 1.0, z: 0.0}, \
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### Using Python Script

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def send_goal(self, x, y, theta=0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        # Convert theta to quaternion (simplified for z-rotation only)
        import math
        msg.pose.orientation.z = math.sin(theta / 2)
        msg.pose.orientation.w = math.cos(theta / 2)
        self.publisher.publish(msg)
        self.get_logger().info(f'Goal sent: ({x}, {y}, {theta})')

def main():
    rclpy.init()
    node = GoalPublisher()
    node.send_goal(2.0, 1.0, 1.57)  # Go to (2,1) facing +Y
    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 6. Real-Time Planning Integration

### Architecture for Sensor-Based Navigation

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   LiDAR     │────▶│  Costmap2D   │────▶│   Planner   │
│   /scan     │     │  (obstacle   │     │  (A*, RRT)  │
└─────────────┘     │   layer)     │     └──────┬──────┘
                    └──────────────┘            │
                                                ▼
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│  Odometry   │────▶│  Navigator   │◀────│    Path     │
│   /odom     │     │  (our PID)   │     │   /plan     │
└─────────────┘     └──────┬───────┘     └─────────────┘
                           │
                           ▼
                    ┌──────────────┐
                    │   /cmd_vel   │
                    │   (Twist)    │
                    └──────────────┘
```

### Extending for Obstacle Avoidance

To integrate with Nav2 for full navigation:

```bash
# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Launch Nav2 with our controller
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

---

## 7. Monitoring and Debugging

### View TF Tree
```bash
ros2 run tf2_tools view_frames
# Creates frames.pdf showing transform hierarchy
```

### Monitor Control Loop
```bash
# Watch velocity commands
ros2 topic hz /cmd_vel

# Echo with timestamps
ros2 topic echo /cmd_vel --field linear.x
```

### Plot Data with PlotJuggler
```bash
sudo apt install ros-humble-plotjuggler-ros
ros2 run plotjuggler plotjuggler
# Stream /odom, /cmd_vel for real-time plots
```

### Record and Replay
```bash
# Record simulation data
ros2 bag record /odom /cmd_vel /scan /goal_pose -o navigation_test

# Replay later
ros2 bag play navigation_test
```

---

## 8. Tuning the Controller

### Live Parameter Updates

```bash
# List parameters
ros2 param list /point_to_point_navigator

# Get current value
ros2 param get /point_to_point_navigator distance_kp

# Set new value (live update)
ros2 param set /point_to_point_navigator distance_kp 1.5
```

### Systematic Tuning Process

1. **Start with Distance Controller**
   - Set `heading_kp=0` temporarily
   - Drive robot straight toward goal
   - Tune `distance_kp` until response is fast but not oscillating

2. **Tune Heading Controller**
   - Reset heading gains
   - Command rotation-only goals
   - Increase `heading_kp` until fast response, add `heading_kd` to damp

3. **Combined Motion**
   - Test diagonal goals
   - Adjust coupling between controllers

---

## 9. Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| Robot doesn't move | Gazebo not publishing /odom | Check `ros2 topic list` |
| Robot spins in place | Heading error never settles | Reduce `heading_kp` |
| Oscillations | Gains too high | Reduce Kp, increase Kd |
| Slow response | Gains too low | Gradually increase Kp |
| Wrong direction | TF frames misaligned | Check `ros2 run tf2_ros tf2_echo odom base_footprint` |

---

## 10. Next Steps

1. **Add obstacle avoidance** using LiDAR data
2. **Implement path tracking** (pure pursuit, Stanley controller)
3. **Integrate with Nav2** for SLAM and global planning
4. **Add sensor fusion** (IMU + wheel odometry) with EKF
