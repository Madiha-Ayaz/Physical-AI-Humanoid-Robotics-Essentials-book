---
sidebar_position: 4
title: ROS 2 Hands-On Exercises
description: Practical exercises to master ROS 2 programming
---

# ROS 2 Hands-On Exercises

## Overview

These exercises build progressively from simple single-node systems to complex multi-node architectures. Complete them in order to solidify your ROS 2 programming skills.

**Time Required**: 6-8 hours total
**Prerequisites**: Completed Week 3-5 ROS 2 Fundamentals

---

## Exercise 1: Hello Robot Node

**Objective**: Create your first ROS 2 node that logs messages

**Difficulty**: ⭐ Beginner

### Requirements

Create a node that:
1. Prints "Hello, Robot World!" when it starts
2. Logs a counter message every 2 seconds
3. Uses proper ROS 2 logging levels (INFO, WARN, ERROR)

### Implementation Template

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class HelloRobotNode(Node):
    def __init__(self):
        super().__init__('hello_robot')

        # TODO: Add timer that calls hello_callback every 2 seconds

        # TODO: Initialize counter to 0

        self.get_logger().info('Hello, Robot World!')

    def hello_callback(self):
        # TODO: Increment counter
        # TODO: Log message with counter value
        # TODO: Log WARNING if counter > 5
        # TODO: Log ERROR if counter > 10
        pass


def main(args=None):
    rclpy.init(args=args)
    node = HelloRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Expected Output

```
[INFO] [1234567890.123]: Hello, Robot World!
[INFO] [1234567892.123]: Message count: 1
[INFO] [1234567894.123]: Message count: 2
...
[WARN] [1234567902.123]: Message count: 6 (Warning: High count)
...
[ERROR] [1234567912.123]: Message count: 11 (Error: Very high count!)
```

### Success Criteria

- ✅ Node starts without errors
- ✅ Messages logged every 2 seconds
- ✅ Counter increments correctly
- ✅ WARNING logged after 5 messages
- ✅ ERROR logged after 10 messages

---

## Exercise 2: Sensor Data Publisher

**Objective**: Publish simulated sensor data on a topic

**Difficulty**: ⭐⭐ Beginner-Intermediate

### Requirements

Create a publisher node that:
1. Publishes simulated LIDAR distance data (0-10 meters)
2. Publishes at 10 Hz (10 times per second)
3. Uses `sensor_msgs/msg/Range` message type
4. Adds random noise to simulate real sensor behavior
5. Includes proper frame_id and timestamp

### Implementation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import random
import math


class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')

        # Create publisher for Range messages
        self.publisher_ = self.create_publisher(
            Range,
            '/sensor/lidar',
            10
        )

        # Timer for 10 Hz publishing
        self.timer = self.create_timer(0.1, self.publish_range)

        # Simulation variables
        self.angle = 0.0  # Radians

        self.get_logger().info('LIDAR Publisher started')

    def publish_range(self):
        msg = Range()

        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'

        # Range sensor characteristics
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.1  # radians (~5.7 degrees)
        msg.min_range = 0.1  # meters
        msg.max_range = 10.0  # meters

        # Simulate distance with sine wave + noise
        base_distance = 5.0 + 3.0 * math.sin(self.angle)
        noise = random.gauss(0, 0.05)  # Gaussian noise
        msg.range = max(msg.min_range, min(msg.max_range, base_distance + noise))

        # Publish
        self.publisher_.publish(msg)

        # Update angle for next reading
        self.angle += 0.1

        if self.angle > 2 * math.pi:
            self.angle = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Test Commands

```bash
# Run publisher
ros2 run my_robot_package lidar_publisher

# In another terminal, echo messages
ros2 topic echo /sensor/lidar

# Check publishing rate
ros2 topic hz /sensor/lidar
# Should show ~10 Hz

# View message details
ros2 topic info /sensor/lidar --verbose
```

### Success Criteria

- ✅ Publishes Range messages at 10 Hz
- ✅ Range values between 0.1 and 10.0 meters
- ✅ Timestamps are current
- ✅ Frame ID is correctly set
- ✅ Data varies realistically with noise

---

## Exercise 3: Subscriber Processing Pipeline

**Objective**: Create a subscriber that processes and republishes data

**Difficulty**: ⭐⭐⭐ Intermediate

### Requirements

Create a processing node that:
1. Subscribes to `/sensor/lidar` (from Exercise 2)
2. Applies a moving average filter (window size = 5)
3. Detects obstacles (distance < 2 meters)
4. Publishes filtered data to `/sensor/lidar_filtered`
5. Publishes obstacle warnings to `/alerts/obstacles`

### Implementation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from collections import deque


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Subscriber
        self.subscription = self.create_subscription(
            Range,
            '/sensor/lidar',
            self.lidar_callback,
            10
        )

        # Publishers
        self.filtered_pub = self.create_publisher(
            Range,
            '/sensor/lidar_filtered',
            10
        )

        self.alert_pub = self.create_publisher(
            String,
            '/alerts/obstacles',
            10
        )

        # Moving average buffer (FIFO queue)
        self.buffer = deque(maxlen=5)

        # Statistics
        self.total_readings = 0
        self.obstacles_detected = 0

        self.get_logger().info('LIDAR Processor started')

    def lidar_callback(self, msg):
        """
        Process incoming LIDAR data with moving average filter.
        """
        self.total_readings += 1

        # Add to buffer
        self.buffer.append(msg.range)

        # Calculate moving average
        if len(self.buffer) > 0:
            filtered_range = sum(self.buffer) / len(self.buffer)
        else:
            filtered_range = msg.range

        # Publish filtered data
        filtered_msg = Range()
        filtered_msg.header = msg.header
        filtered_msg.radiation_type = msg.radiation_type
        filtered_msg.field_of_view = msg.field_of_view
        filtered_msg.min_range = msg.min_range
        filtered_msg.max_range = msg.max_range
        filtered_msg.range = filtered_range

        self.filtered_pub.publish(filtered_msg)

        # Obstacle detection
        obstacle_threshold = 2.0  # meters

        if filtered_range < obstacle_threshold:
            self.obstacles_detected += 1

            alert_msg = String()
            alert_msg.data = (
                f'OBSTACLE DETECTED! Distance: {filtered_range:.2f}m '
                f'(Raw: {msg.range:.2f}m)'
            )

            self.alert_pub.publish(alert_msg)
            self.get_logger().warn(alert_msg.data)

        # Log statistics every 100 readings
        if self.total_readings % 100 == 0:
            self.get_logger().info(
                f'Stats: Total readings={self.total_readings}, '
                f'Obstacles detected={self.obstacles_detected}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Test Scenario

```bash
# Terminal 1: Run LIDAR publisher (from Exercise 2)
ros2 run my_robot_package lidar_publisher

# Terminal 2: Run processor
ros2 run my_robot_package lidar_processor

# Terminal 3: Monitor filtered output
ros2 topic echo /sensor/lidar_filtered

# Terminal 4: Monitor alerts
ros2 topic echo /alerts/obstacles
```

### Success Criteria

- ✅ Subscribes to raw LIDAR data
- ✅ Applies moving average smoothing
- ✅ Publishes filtered data
- ✅ Detects obstacles when distance < 2m
- ✅ Publishes alerts for obstacles
- ✅ Logs statistics periodically

---

## Exercise 4: Service-Based Calculator

**Objective**: Implement a robot kinematics service

**Difficulty**: ⭐⭐⭐ Intermediate

### Requirements

Create a service system for forward kinematics:
1. **Service**: Calculates end-effector position from joint angles
2. **Server**: Implements 2-DOF planar arm forward kinematics
3. **Client**: Sends joint angles and receives Cartesian position
4. Uses custom service definition

### Custom Service Definition

**File**: `~/ros2_ws/src/my_robot_interfaces/srv/ForwardKinematics.srv`

```
# Request: Joint angles in radians
float64 joint1_angle
float64 joint2_angle

# Link lengths in meters
float64 link1_length
float64 link2_length
---
# Response: End-effector position
float64 x
float64 y
bool success
string message
```

### Server Implementation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ForwardKinematics
import math


class KinematicsServer(Node):
    def __init__(self):
        super().__init__('kinematics_server')

        self.srv = self.create_service(
            ForwardKinematics,
            'forward_kinematics',
            self.forward_kinematics_callback
        )

        self.get_logger().info('Forward Kinematics Service Ready')

    def forward_kinematics_callback(self, request, response):
        """
        Calculate forward kinematics for 2-DOF planar arm.

        Math:
        x = L1*cos(θ1) + L2*cos(θ1 + θ2)
        y = L1*sin(θ1) + L2*sin(θ1 + θ2)
        """
        try:
            theta1 = request.joint1_angle
            theta2 = request.joint2_angle
            L1 = request.link1_length
            L2 = request.link2_length

            # Validate inputs
            if L1 <= 0 or L2 <= 0:
                response.success = False
                response.message = "Link lengths must be positive"
                return response

            # Forward kinematics calculation
            x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
            y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)

            response.x = x
            response.y = y
            response.success = True
            response.message = f"Calculated position: ({x:.3f}, {y:.3f})"

            self.get_logger().info(
                f'FK: θ1={math.degrees(theta1):.1f}°, '
                f'θ2={math.degrees(theta2):.1f}° → '
                f'({x:.3f}, {y:.3f})'
            )

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    server = KinematicsServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass

    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Client Implementation

```python
#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ForwardKinematics
import math


class KinematicsClient(Node):
    def __init__(self):
        super().__init__('kinematics_client')

        self.client = self.create_client(
            ForwardKinematics,
            'forward_kinematics'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def calculate_position(self, joint1_deg, joint2_deg, l1=1.0, l2=0.8):
        """
        Call forward kinematics service.

        Args:
            joint1_deg: Joint 1 angle in degrees
            joint2_deg: Joint 2 angle in degrees
            l1: Link 1 length in meters
            l2: Link 2 length in meters
        """
        request = ForwardKinematics.Request()
        request.joint1_angle = math.radians(joint1_deg)
        request.joint2_angle = math.radians(joint2_deg)
        request.link1_length = l1
        request.link2_length = l2

        self.get_logger().info(
            f'Requesting FK for joints: {joint1_deg}°, {joint2_deg}°'
        )

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'End-effector position: ({response.x:.3f}, {response.y:.3f})'
                )
                return (response.x, response.y)
            else:
                self.get_logger().error(f'Service error: {response.message}')
        else:
            self.get_logger().error('Service call failed')

        return None


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print('Usage: ros2 run my_robot_package kinematics_client <joint1_deg> <joint2_deg>')
        return

    joint1 = float(sys.argv[1])
    joint2 = float(sys.argv[2])

    client = KinematicsClient()
    result = client.calculate_position(joint1, joint2)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Test Cases

```bash
# Start server
ros2 run my_robot_package kinematics_server

# Test cases (in separate terminals)
ros2 run my_robot_package kinematics_client 0 0
# Expected: x ≈ 1.8, y ≈ 0.0 (arm fully extended right)

ros2 run my_robot_package kinematics_client 90 0
# Expected: x ≈ 0.8, y ≈ 1.0 (arm pointing up)

ros2 run my_robot_package kinematics_client 45 -45
# Expected: x ≈ 1.27, y ≈ 0.71 (arm at 45° angle)

ros2 run my_robot_package kinematics_client 180 0
# Expected: x ≈ -1.8, y ≈ 0.0 (arm extended left)
```

### Success Criteria

- ✅ Service server starts without errors
- ✅ Client can connect to service
- ✅ Forward kinematics calculations are correct
- ✅ Handles invalid inputs gracefully
- ✅ All test cases produce expected results (within 0.01m tolerance)

---

## Exercise 5: Multi-Node Temperature Monitoring System

**Objective**: Build a complete multi-node system with launch file

**Difficulty**: ⭐⭐⭐⭐ Advanced

### System Architecture

```
┌─────────────────┐
│ Multi-Sensor    │ ──┬──> /sensor/temp_1
│ Publisher       │   ├──> /sensor/temp_2
│                 │   └──> /sensor/temp_3
└─────────────────┘
         │
         v
┌─────────────────┐        ┌──────────────┐
│ Temperature     │ ────>  │ Alert        │
│ Monitor         │ alerts │ Handler      │
│ (Subscriber)    │        │              │
└─────────────────┘        └──────────────┘
         │
         │ /statistics (service)
         v
┌─────────────────┐
│ Statistics      │
│ Server          │
└─────────────────┘
```

### Requirements

1. **Publisher**: Publishes data from 3 simulated temperature sensors
2. **Monitor**: Subscribes to all sensors, tracks statistics, publishes alerts
3. **Alert Handler**: Logs and stores alert history
4. **Statistics Service**: Returns min/max/avg for each sensor
5. **Launch File**: Starts all nodes with parameters
6. **Parameter File**: Configures sensor names and thresholds

### Implementation Files Needed

- `multi_sensor_publisher.py`
- `temperature_monitor.py`
- `alert_handler.py`
- `statistics_server.py`
- `launch/temp_system_launch.py`
- `config/temp_params.yaml`

### Sample Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_package'),
        'config',
        'temp_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='multi_sensor_publisher',
            name='sensor_array',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='my_robot_package',
            executable='temperature_monitor',
            name='temp_monitor',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='my_robot_package',
            executable='alert_handler',
            name='alert_handler',
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='statistics_server',
            name='stats_service',
            output='screen'
        ),
    ])
```

### Sample Parameter File

```yaml
multi_sensor_publisher:
  ros__parameters:
    sensor_names: ['engine', 'battery', 'motor']
    base_temperatures: [85.0, 45.0, 70.0]
    update_rate: 10.0

temperature_monitor:
  ros__parameters:
    alert_threshold: 90.0
    critical_threshold: 100.0
    window_size: 10
```

### Success Criteria

- ✅ All 4 nodes start via single launch command
- ✅ Temperature data published for 3 sensors at 10 Hz
- ✅ Monitor tracks statistics for each sensor
- ✅ Alerts published when thresholds exceeded
- ✅ Alert handler logs all alerts
- ✅ Statistics service returns accurate data
- ✅ System runs for 5 minutes without errors
- ✅ Parameters can be changed without code modifications

### Bonus Challenges

1. Add a visualization node using `matplotlib`
2. Implement data logging to CSV file
3. Add a shutdown service to stop all nodes gracefully
4. Create unit tests for the monitor node

---

## Troubleshooting Tips

### Common Issues

**Problem**: Node not found after building
```bash
# Solution: Ensure setup.py has the entry point
# Rebuild and re-source
colcon build --packages-select my_robot_package
source install/setup.bash
```

**Problem**: Import errors for custom messages
```bash
# Solution: Build interfaces package first
colcon build --packages-select my_robot_interfaces
source install/setup.bash
colcon build --packages-select my_robot_package
```

**Problem**: Nodes can't discover each other
```bash
# Solution: Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID
# Set to same value for all nodes (0-101)
export ROS_DOMAIN_ID=0
```

---

## Submission Checklist

For each exercise, ensure you have:

- [ ] Source code with proper comments
- [ ] Entry points in `setup.py`
- [ ] Successful build output
- [ ] Test command outputs demonstrating functionality
- [ ] Screenshots or logs showing expected behavior
- [ ] Brief explanation of your implementation choices

---

**Next**: [Module 2: Robot Simulation →](../module-02/intro.md)

Congratulations on completing Module 1! You now have solid ROS 2 fundamentals. Continue to Module 2 to learn simulation in Gazebo and Unity.
