---
sidebar_position: 3
title: Week 3-5 - ROS 2 Fundamentals
description: Mastering ROS 2 architecture, nodes, topics, services, and Python programming
---

# Week 3-5: ROS 2 Fundamentals & Python Programming

## Learning Objectives

By the end of this section, you will be able to:

- Understand ROS 2 architecture and the DDS middleware layer
- Create and manage ROS 2 nodes using Python (`rclpy`)
- Implement publisher-subscriber patterns for asynchronous communication
- Use services for synchronous request-response interactions
- Define custom message types for your robot applications
- Build and manage ROS 2 packages with `colcon`
- Configure nodes using parameters and launch files
- Debug multi-node systems using ROS 2 command-line tools

---

## 1. ROS 2 Architecture Overview

### What is ROS 2?

**ROS 2 (Robot Operating System 2)** is an open-source middleware framework for building robot applications. It provides:

- **Communication infrastructure** for distributed systems
- **Hardware abstraction** for sensors and actuators
- **Package management** for modular development
- **Tools** for visualization, debugging, and simulation

### Key Concepts

#### Computational Graph

ROS 2 applications consist of a **computational graph** of nodes:

```
┌─────────────┐      Topic: /sensor_data      ┌─────────────┐
│   Sensor    │ ──────────────────────────────>│  Processing │
│    Node     │                                 │    Node     │
└─────────────┘                                 └─────────────┘
                                                       │
                                                       │ Topic: /cmd_vel
                                                       v
                                                ┌─────────────┐
                                                │   Robot     │
                                                │  Controller │
                                                └─────────────┘
```

**Components**:
- **Nodes**: Independent processes that perform computation
- **Topics**: Named buses for asynchronous message passing
- **Services**: Synchronous request-response communication
- **Actions**: Asynchronous goal-based tasks with feedback
- **Parameters**: Configuration values for nodes

### DDS Middleware

ROS 2 uses **Data Distribution Service (DDS)** as its middleware:

- **Quality of Service (QoS)**: Configure reliability, durability, history
- **Discovery**: Automatic node and topic discovery on the network
- **Security**: DDS-Security for encrypted communication
- **Real-time**: Low-latency, deterministic communication

:::info DDS Implementations
ROS 2 supports multiple DDS vendors: Fast DDS (default), CycloneDDS, RTI Connext. They're interchangeable via environment variables.
:::

---

## 2. Setting Up Your ROS 2 Workspace

### Installation Verification

```bash
# Check ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 --version

# Should output: ros2 doctor 0.10.x or similar
```

### Creating a Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create your first package
ros2 pkg create --build-type ament_python my_robot_package \
  --dependencies rclpy std_msgs geometry_msgs

# Build the workspace
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash
```

**Directory Structure**:
```
ros2_ws/
├── src/                    # Source code
│   └── my_robot_package/
│       ├── my_robot_package/  # Python module
│       ├── package.xml        # Package metadata
│       ├── setup.py           # Python setup
│       └── setup.cfg          # Configuration
├── build/                  # Build artifacts
├── install/                # Installed files
└── log/                    # Build logs
```

---

## 3. Creating a ROS 2 Publisher Node

### Simple Publisher Example

Create a node that publishes sensor data at regular intervals.

**File**: `~/ros2_ws/src/my_robot_package/my_robot_package/sensor_publisher.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random


class SensorPublisher(Node):
    """
    A simple ROS 2 publisher node that simulates sensor data.
    Publishes temperature readings to /sensor/temperature topic.
    """

    def __init__(self):
        # Initialize the node with name 'sensor_publisher'
        super().__init__('sensor_publisher')

        # Create publisher for String messages on /sensor/temperature topic
        # Queue size of 10 means it buffers up to 10 messages
        self.publisher_ = self.create_publisher(
            String,
            '/sensor/temperature',
            10
        )

        # Create timer that calls timer_callback every 1.0 seconds
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Counter for number of messages published
        self.count = 0

        self.get_logger().info('Sensor Publisher Node Started')

    def timer_callback(self):
        """
        Called every timer_period seconds. Publishes simulated temperature data.
        """
        # Create message
        msg = String()

        # Simulate temperature reading (20-30 degrees Celsius)
        temperature = 20.0 + random.uniform(0, 10)
        msg.data = f'Temperature: {temperature:.2f}°C | Count: {self.count}'

        # Publish message
        self.publisher_.publish(msg)

        # Log to console
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.count += 1


def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create node instance
    sensor_publisher = SensorPublisher()

    # Spin (keep node running and processing callbacks)
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass

    # Cleanup
    sensor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Make executable**:
```bash
chmod +x ~/ros2_ws/src/my_robot_package/my_robot_package/sensor_publisher.py
```

**Update `setup.py`** to register the node:

```python
entry_points={
    'console_scripts': [
        'sensor_publisher = my_robot_package.sensor_publisher:main',
    ],
},
```

**Build and run**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash

# Run the publisher
ros2 run my_robot_package sensor_publisher
```

---

## 4. Creating a ROS 2 Subscriber Node

### Simple Subscriber Example

Create a node that subscribes to the temperature data and processes it.

**File**: `~/ros2_ws/src/my_robot_package/my_robot_package/sensor_subscriber.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SensorSubscriber(Node):
    """
    A simple ROS 2 subscriber node that receives sensor data.
    Subscribes to /sensor/temperature topic and processes readings.
    """

    def __init__(self):
        super().__init__('sensor_subscriber')

        # Create subscription to /sensor/temperature topic
        self.subscription = self.create_subscription(
            String,
            '/sensor/temperature',
            self.listener_callback,
            10  # QoS queue size
        )

        # Prevent unused variable warning
        self.subscription

        # Statistics tracking
        self.message_count = 0
        self.temperature_sum = 0.0

        self.get_logger().info('Sensor Subscriber Node Started')

    def listener_callback(self, msg):
        """
        Called whenever a message is received on /sensor/temperature topic.

        Args:
            msg (String): The received message
        """
        # Log received message
        self.get_logger().info(f'Received: "{msg.data}"')

        # Parse temperature value (simple string parsing)
        try:
            # Extract temperature from "Temperature: 25.34°C | Count: 5"
            temp_str = msg.data.split('|')[0].split(':')[1].strip()
            temperature = float(temp_str.replace('°C', ''))

            # Update statistics
            self.message_count += 1
            self.temperature_sum += temperature

            avg_temp = self.temperature_sum / self.message_count

            # Log analysis
            self.get_logger().info(
                f'Current: {temperature:.2f}°C | '
                f'Average: {avg_temp:.2f}°C | '
                f'Messages: {self.message_count}'
            )

            # Temperature alert logic
            if temperature > 28.0:
                self.get_logger().warn(f'High temperature alert: {temperature:.2f}°C')

        except Exception as e:
            self.get_logger().error(f'Error parsing message: {e}')


def main(args=None):
    rclpy.init(args=args)

    sensor_subscriber = SensorSubscriber()

    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        pass

    sensor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Update `setup.py`**:
```python
entry_points={
    'console_scripts': [
        'sensor_publisher = my_robot_package.sensor_publisher:main',
        'sensor_subscriber = my_robot_package.sensor_subscriber:main',
    ],
},
```

**Run both nodes** (in separate terminals):

```bash
# Terminal 1: Publisher
ros2 run my_robot_package sensor_publisher

# Terminal 2: Subscriber
ros2 run my_robot_package sensor_subscriber
```

---

## 5. Defining Custom Messages

Create custom message types for structured robot data.

### Step 1: Create Message Definition

**File**: `~/ros2_ws/src/my_robot_package/msg/SensorReading.msg`

```
# Custom message for sensor readings

# Header with timestamp and frame info
std_msgs/Header header

# Sensor identification
string sensor_id
string sensor_type

# Readings
float64 temperature
float64 humidity
float64 pressure

# Status
bool is_calibrated
uint8 error_code
```

### Step 2: Update `package.xml`

```xml
<!-- Add these dependencies -->
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Step 3: Update `CMakeLists.txt` (for ament_cmake) or `setup.py` (for ament_python)

For **ament_python** package, create a new **ament_cmake** package for messages:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_interfaces

# Move msg files there
mkdir -p my_robot_interfaces/msg
mv my_robot_package/msg/SensorReading.msg my_robot_interfaces/msg/
```

**Edit `my_robot_interfaces/CMakeLists.txt`**:

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorReading.msg"
  DEPENDENCIES std_msgs
)
```

**Build**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_interfaces
source install/setup.bash
```

### Step 4: Use Custom Message

**Publisher with custom message**:

```python
from my_robot_interfaces.msg import SensorReading
from std_msgs.msg import Header

class CustomSensorPublisher(Node):
    def __init__(self):
        super().__init__('custom_sensor_publisher')

        self.publisher_ = self.create_publisher(
            SensorReading,
            '/sensor/readings',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_reading)

    def publish_reading(self):
        msg = SensorReading()

        # Populate header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_frame'

        # Sensor info
        msg.sensor_id = 'DHT22_01'
        msg.sensor_type = 'temperature_humidity'

        # Readings
        msg.temperature = 25.0 + random.uniform(-2, 2)
        msg.humidity = 60.0 + random.uniform(-10, 10)
        msg.pressure = 1013.25 + random.uniform(-5, 5)

        # Status
        msg.is_calibrated = True
        msg.error_code = 0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: Temp={msg.temperature:.2f}°C')
```

---

## 6. Using Services

Services provide **synchronous request-response** communication.

### Built-in Service Example

**Server Node** - Adds two integers:

**File**: `~/ros2_ws/src/my_robot_package/my_robot_package/add_two_ints_server.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """
    Service server that adds two integers.
    """

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Add Two Ints Service Server Ready')

    def add_two_ints_callback(self, request, response):
        """
        Service callback that performs addition.

        Args:
            request: Contains 'a' and 'b' integers
            response: Contains 'sum' field to populate

        Returns:
            response with sum calculated
        """
        response.sum = request.a + request.b

        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    add_two_ints_server = AddTwoIntsServer()

    try:
        rclpy.spin(add_two_ints_server)
    except KeyboardInterrupt:
        pass

    add_two_ints_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Client Node**:

**File**: `~/ros2_ws/src/my_robot_package/my_robot_package/add_two_ints_client.py`

```python
#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """
    Service client that requests addition of two integers.
    """

    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Connected to Add Two Ints Service')

    def send_request(self, a, b):
        """
        Send request to add two integers.

        Args:
            a (int): First integer
            b (int): Second integer

        Returns:
            int: Sum of a and b
        """
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service asynchronously
        future = self.client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Result: {a} + {b} = {response.sum}')
            return response.sum
        else:
            self.get_logger().error('Service call failed')
            return None


def main(args=None):
    rclpy.init(args=args)

    # Get numbers from command line
    if len(sys.argv) < 3:
        print('Usage: ros2 run my_robot_package add_two_ints_client <a> <b>')
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    client = AddTwoIntsClient()
    result = client.send_request(a, b)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Run**:
```bash
# Terminal 1: Server
ros2 run my_robot_package add_two_ints_server

# Terminal 2: Client
ros2 run my_robot_package add_two_ints_client 10 25
# Output: Result: 10 + 25 = 35
```

---

## 7. Launch Files

Launch files start multiple nodes with configuration.

### Python Launch File

**File**: `~/ros2_ws/src/my_robot_package/launch/sensor_system_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file to start sensor publisher and subscriber nodes.
    """

    return LaunchDescription([
        # Sensor Publisher Node
        Node(
            package='my_robot_package',
            executable='sensor_publisher',
            name='sensor_publisher',
            output='screen',
            parameters=[{
                'timer_period': 0.5  # Publish every 0.5 seconds
            }]
        ),

        # Sensor Subscriber Node
        Node(
            package='my_robot_package',
            executable='sensor_subscriber',
            name='sensor_subscriber',
            output='screen',
            remappings=[
                ('/sensor/temperature', '/sensor/temp')  # Topic remapping
            ]
        ),

        # Service Server Node
        Node(
            package='my_robot_package',
            executable='add_two_ints_server',
            name='calculator_service',
            output='screen'
        ),
    ])
```

**Directory structure for launch files**:
```bash
mkdir -p ~/ros2_ws/src/my_robot_package/launch
```

**Update `setup.py`**:
```python
import os
from glob import glob

setup(
    # ... existing setup ...
    data_files=[
        # ... existing data_files ...
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
)
```

**Run launch file**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash

ros2 launch my_robot_package sensor_system_launch.py
```

---

## 8. Parameters

Configure nodes at runtime without code changes.

### Using Parameters in Nodes

```python
class ConfigurablePublisher(Node):
    def __init__(self):
        super().__init__('configurable_publisher')

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('topic_name', '/data')
        self.declare_parameter('message_prefix', 'Data')

        # Get parameter values
        rate = self.get_parameter('publish_rate').value
        topic = self.get_parameter('topic_name').value
        self.prefix = self.get_parameter('message_prefix').value

        # Create publisher with configured topic
        self.publisher_ = self.create_publisher(String, topic, 10)

        # Create timer with configured rate
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(
            f'Publishing to {topic} at {rate} Hz with prefix "{self.prefix}"'
        )
```

### YAML Parameter File

**File**: `~/ros2_ws/src/my_robot_package/config/params.yaml`

```yaml
configurable_publisher:
  ros__parameters:
    publish_rate: 5.0
    topic_name: '/sensor/data'
    message_prefix: 'Sensor'
```

**Launch with parameters**:
```python
Node(
    package='my_robot_package',
    executable='configurable_publisher',
    name='configurable_publisher',
    parameters=[os.path.join(
        get_package_share_directory('my_robot_package'),
        'config', 'params.yaml'
    )]
)
```

---

## 9. Command-Line Tools

### Essential ROS 2 CLI Commands

```bash
# List all running nodes
ros2 node list

# Get info about a node
ros2 node info /sensor_publisher

# List all topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /sensor/temperature

# Get topic info
ros2 topic info /sensor/temperature

# Publish to a topic from command line
ros2 topic pub /sensor/temperature std_msgs/msg/String "{data: 'Test: 25.5°C'}"

# List all services
ros2 service list

# Call a service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"

# Get/set parameters
ros2 param list /sensor_publisher
ros2 param get /sensor_publisher publish_rate
ros2 param set /sensor_publisher publish_rate 20.0

# View computational graph
rqt_graph
```

---

## Key Takeaways

1. **ROS 2 nodes** are independent processes communicating via topics, services, and actions
2. **Topics** enable asynchronous, many-to-many communication
3. **Services** provide synchronous request-response patterns
4. **Custom messages** define structured data types for your robot
5. **Launch files** orchestrate multiple nodes with configuration
6. **Parameters** make nodes configurable without code changes
7. **CLI tools** are essential for debugging and introspection

---

## Hands-On Exercise

**Objective**: Build a temperature monitoring system with alerts

**Requirements**:
1. Publisher node simulating multiple temperature sensors (3 sensors)
2. Subscriber node that:
   - Tracks min/max/average for each sensor
   - Publishes alerts when temperature > 30°C
3. Service for resetting statistics
4. Launch file to start the system
5. Parameter file for sensor names and alert thresholds

**Expected Files**:
- `multi_sensor_publisher.py`
- `temperature_monitor.py`
- `reset_stats_server.py`
- `temp_monitor_launch.py`
- `config/temp_monitor_params.yaml`

**Success Criteria**:
- All nodes start via launch file
- Temperature data published at 10 Hz per sensor
- Alerts logged when threshold exceeded
- Statistics can be reset via service call
- Sensor names configurable via parameters

---

## Assessment Questions

1. What is the difference between a topic and a service in ROS 2?
2. Why does ROS 2 use DDS middleware instead of the ROS 1 approach?
3. Write the command to list all messages published on the `/cmd_vel` topic.
4. What is the purpose of Quality of Service (QoS) in ROS 2?
5. How do you make a Python node executable after creation?
6. Explain the benefit of using launch files instead of running nodes individually.
7. What is the difference between `colcon build` and `colcon build --packages-select`?
8. How do parameters improve node reusability?

---

## Further Reading

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [rclpy API Reference](https://docs.ros2.org/latest/api/rclpy/)
- [ROS 2 Design Decisions](https://design.ros2.org/)
- [DDS Specification](https://www.omg.org/spec/DDS/)

---

**Next**: [ROS 2 Hands-On Exercises →](./ros2-hands-on.md)

Practice your skills with guided exercises building complete ROS 2 systems.
