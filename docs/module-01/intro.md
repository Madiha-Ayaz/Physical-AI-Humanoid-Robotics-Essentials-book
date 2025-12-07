---
sidebar_position: 1
title: Module 1 Overview
description: Introduction to Physical AI and ROS 2 Fundamentals (Weeks 1-5)
---

# Module 1: Physical AI & ROS 2 Fundamentals

**Weeks 1-5: Building the Foundation**

Welcome to Module 1, where you'll establish the fundamental knowledge required for all subsequent modules. Over five weeks, you'll explore the concept of Physical AI - artificial intelligence that exists not just in the cloud but in physical bodies capable of sensing and acting in the real world. You'll master ROS 2 (Robot Operating System 2), the industry-standard middleware for building modular, scalable robotic applications.

## Module Overview

This module bridges the gap between traditional software AI and embodied robotic systems. You'll learn how robots perceive their environment through sensors, process information using ROS 2's distributed architecture, and coordinate complex behaviors through inter-process communication.

### What You'll Learn

By the end of this module, you will:

- ✅ Understand the principles of **embodied intelligence** and how Physical AI differs from cloud-based AI
- ✅ Identify and work with common **robotic sensors** (cameras, LIDAR, IMUs, encoders)
- ✅ Navigate the **ROS 2 architecture** including DDS middleware and the workspace structure
- ✅ Create **ROS 2 nodes** that communicate via topics, services, and actions
- ✅ Write **Python-based ROS 2 packages** with proper dependency management
- ✅ Build a complete **publisher-subscriber system** for sensor data processing
- ✅ Implement **service-client patterns** for request-response workflows
- ✅ Debug ROS 2 applications using command-line tools

## Learning Outcomes

### Knowledge

- Explain the concept of embodied intelligence and its advantages over disembodied AI
- Describe the role of sensors in enabling physical perception
- Understand ROS 2's computational graph model (nodes, topics, services, actions)
- Comprehend the Data Distribution Service (DDS) middleware layer

### Skills

- Install and configure ROS 2 Humble on Ubuntu 22.04
- Create and build ROS 2 packages using `colcon`
- Write Python nodes that publish and subscribe to topics
- Implement service servers and clients for synchronous communication
- Use ROS 2 command-line tools (`ros2 topic`, `ros2 service`, `ros2 node`)
- Debug multi-node systems using `rqt_graph` and logging

### Application

- Design a multi-node robotic system for a specific task
- Process simulated sensor data in real-time
- Coordinate distributed behaviors across multiple processes
- Build a foundation for simulation (Module 2) and AI perception (Module 3)

## Weekly Breakdown

### Week 1-2: Physical AI Foundations

**Topics**: Embodied intelligence, sensor systems, robot perception

**Content**:
- [Introduction to Physical AI](./week-01-02-physical-ai-intro.md)
  - What is Physical AI? Embodied vs. Disembodied intelligence
  - Sensor types: Cameras (RGB, depth), LIDAR, IMUs, encoders
  - Sensor fusion basics
  - Real-world robotics applications

**Time Investment**: 6-8 hours (reading + exercises)

### Week 3-5: ROS 2 Fundamentals

**Topics**: ROS 2 architecture, nodes, topics, services, package development

**Content**:
- [ROS 2 Architecture & Python Programming](./week-03-05-ros2-fundamentals.md)
  - ROS 2 concepts and terminology
  - DDS middleware and Quality of Service (QoS)
  - Creating nodes with Python (`rclpy`)
  - Publishers and subscribers for topic-based communication
  - Services and clients for request-response patterns
  - Actions for long-running tasks with feedback
  - Building packages with `colcon`

**Time Investment**: 10-12 hours (reading + coding + exercises)

### Hands-On Practice

**Content**:
- [ROS 2 Hands-On Exercises](./ros2-hands-on.md)
  - Exercise 1: Hello Robot Node
  - Exercise 2: Sensor Data Publisher
  - Exercise 3: Subscriber Processing Pipeline
  - Exercise 4: Service-Based Calculator
  - Exercise 5: Multi-Node System

**Time Investment**: 6-8 hours (implementation + debugging)

## Prerequisites

Before starting this module, ensure you have:

- **Python 3.10+**: Understanding of functions, classes, and object-oriented programming
- **Linux Familiarity**: Ability to navigate directories, edit files, and run commands in terminal
- **Development Environment**: Ubuntu 22.04 (native, WSL2, or virtual machine) with 4GB+ RAM

:::warning ROS 2 on Windows/Mac
While ROS 2 supports Windows and macOS, this course focuses on Ubuntu 22.04 for consistency. We recommend using WSL2 (Windows Subsystem for Linux) on Windows or a virtual machine on macOS.
:::

## Required Software

Install the following before Week 3:

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble (Recommended)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete python3-colcon-common-extensions -y

# Source ROS 2 setup (add to ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash
```

**Verification**:
```bash
ros2 --version
# Should output: ros2 doctor 0.10.x (or similar)
```

## Key Concepts Introduced

- **Physical AI**: AI systems embodied in physical robots that perceive and act in the real world
- **Embodied Intelligence**: Cognitive capabilities arising from physical interaction with the environment
- **Sensor Fusion**: Combining data from multiple sensors for robust perception
- **ROS 2 Node**: Independent process that performs computation
- **Topic**: Named bus for asynchronous message passing
- **Service**: Synchronous request-response communication pattern
- **Action**: Asynchronous goal-based interface with feedback
- **DDS**: Data Distribution Service middleware for real-time communication
- **QoS**: Quality of Service policies for reliable data transmission

## Module Assessment

At the end of Module 1, you'll complete a quiz covering:

- Physical AI concepts and embodied intelligence principles
- Sensor types and their applications
- ROS 2 architecture and communication patterns
- Python implementation of ROS 2 nodes
- Debugging and troubleshooting techniques

**Passing Criteria**: 80% correct answers (12 out of 15 questions)

## Hands-On Deliverables

You'll build:

1. **Simple Publisher Node**: Publishes simulated sensor data at 10 Hz
2. **Subscriber Node**: Processes incoming data and logs results
3. **Service Server**: Responds to calculation requests
4. **Multi-Node System**: Coordinates multiple processes via topics and services

## Time Management Tips

- **Week 1-2**: Focus on understanding concepts, no heavy coding yet
- **Week 3**: Install ROS 2 and complete first 2 exercises
- **Week 4**: Build publisher-subscriber systems (Exercises 3-4)
- **Week 5**: Complete multi-node system and review for assessment

:::tip Stay on Track
This module sets the foundation for everything that follows. Don't skip the exercises - hands-on practice is essential for mastering ROS 2.
:::

## Common Challenges & Solutions

| Challenge | Solution |
|-----------|----------|
| ROS 2 installation fails | Use Docker container: `docker run -it ros:humble` |
| Python package not found | Ensure `rclpy` is installed: `pip3 install rclpy` |
| Nodes can't discover each other | Check DDS_DOMAIN_ID and firewall settings |
| Build errors with colcon | Source ROS 2 setup and check package.xml dependencies |

## Additional Resources

- [ROS 2 Official Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [rclpy API Reference](https://docs.ros2.org/latest/api/rclpy/)
- [Awesome ROS 2 List](https://github.com/fkromer/awesome-ros2)

## Ready to Start?

Begin with [Week 1-2: Introduction to Physical AI →](./week-01-02-physical-ai-intro.md)

---

**Module Duration**: 5 weeks
**Difficulty**: Beginner to Intermediate
**Estimated Hours**: 22-28 hours total
**Key Deliverables**: 5 hands-on exercises + assessment quiz
