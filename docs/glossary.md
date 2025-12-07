---
sidebar_position: 102
title: Glossary
description: Technical terms and definitions for Physical AI & Robotics
---

# Glossary of Terms

## A

**Action (ROS 2)**
Asynchronous communication pattern for long-running tasks that provides feedback during execution. Combines aspects of topics (one-way) and services (goal-oriented).

**Actuator**
Device that converts electrical signals into physical motion (motors, servos, hydraulic systems).

**AGV (Automated Guided Vehicle)**
Mobile robot that follows markers or wires for navigation in warehouses and factories.

## B

**Bipedal Locomotion**
Walking on two legs. Critical challenge in humanoid robotics involving balance, gait planning, and ZMP control.

**BMS (Battery Management System)**
Circuit that monitors and controls charging/discharging of rechargeable batteries.

## C

**Colcon**
Build tool for ROS 2 workspaces. Compiles packages and manages dependencies.

**CUDA (Compute Unified Device Architecture)**
NVIDIA's parallel computing platform for GPU programming. Required for Isaac Sim.

## D

**DDS (Data Distribution Service)**
Middleware standard used by ROS 2 for real-time,  decentralized communication between nodes.

**Depth Camera**
Camera that captures distance to objects, not just color. Types: stereo, structured light, time-of-flight.

**DH Parameters (Denavit-Hartenberg)**
Standard convention for defining robot kinematic chains using 4 parameters per joint: α (twist), a (length), d (offset), θ (angle).

**DOF (Degrees of Freedom)**
Number of independent motions a robot can perform. Humanoid robots typically have 20-40 DOF.

## E

**Embodied Intelligence**
AI that exists in a physical form (robot body) and learns through interaction with the environment. Contrasts with disembodied AI (cloud-based).

**Encoder**
Sensor that measures rotation angle of motors/wheels. Used for odometry and joint position feedback.

**End-Effector**
The "hand" of a robot arm - gripper, tool, or other attachment at the terminal link.

## F

**Forward Kinematics (FK)**
Calculating end-effector position/orientation from known joint angles. Opposite of inverse kinematics.

**Frame (TF)**
Coordinate system in 3D space. ROS uses transformation frames to represent robot parts, sensors, and world locations.

## G

**Gazebo**
Open-source 3D robot simulator with physics engine. Used for testing before deploying to hardware.

**GPT (Generative Pre-trained Transformer)**
Large language model architecture. Used in conversational robotics for natural language understanding.

## H

**Humanoid Robot**
Robot with human-like form: head, torso, two arms, two legs. Examples: Atlas, Optimus, Unitree H1.

## I

**IMU (Inertial Measurement Unit)**
Sensor combining accelerometer (linear acceleration) and gyroscope (angular velocity). Often includes magnetometer for compass heading.

**Inverse Kinematics (IK)**
Calculating joint angles needed to reach a desired end-effector pose. Computationally harder than FK.

**Isaac Sim**
NVIDIA's physically accurate robot simulator built on Omniverse. Includes photo-realistic rendering and AI integration.

## J

**Jetson**
NVIDIA's embedded AI computing platform for robots. Models: Nano, Xavier, Orin (8GB, 16GB, 32GB, 64GB).

**Joint Space**
Representation of robot configuration using joint angles/positions. Contrasts with Cartesian/task space.

## K

**Kinematics**
Study of robot motion without considering forces. Forward/inverse kinematics solve position/orientation problems.

## L

**Launch File**
Script (Python or XML) that starts multiple ROS nodes with configuration. Simplifies system startup.

**LIDAR (Light Detection and Ranging)**
Sensor that uses laser pulses to measure distances. Provides precise range data for mapping and obstacle detection.

**Localization**
Process of determining robot's position and orientation in a known map.

## M

**Manipulation**
Task of grasping, moving, and placing objects with robot arms/grippers.

**Mapping**
Process of building a representation of the environment from sensor data.

**MDX (Markdown + JSX)**
Extended markdown format used by Docusaurus. Allows embedding React components in documentation.

## N

**Navigation Stack**
Set of ROS packages for autonomous robot navigation: mapping, localization, path planning, obstacle avoidance.

**Node (ROS 2)**
Independent process that performs specific computation. Communicates with other nodes via topics, services, or actions.

## O

**Odometry**
Estimating robot position by measuring wheel rotations or visual features. Accumulates error over distance.

**Omniverse**
NVIDIA's platform for 3D simulation and collaboration. Isaac Sim is built on Omniverse.

## P

**Parameter**
Configuration value for ROS nodes. Can be set at launch time or changed dynamically.

**Perception**
Robot's ability to understand its environment through sensors (vision, touch, range, etc.).

**Physical AI**
AI systems embodied in physical robots, capable of sensing and acting in the real world.

**PID Controller**
Proportional-Integral-Derivative controller. Common feedback control algorithm for motors.

**Publisher (ROS 2)**
Node that sends messages on a topic. Many-to-many communication pattern.

## Q

**QoS (Quality of Service)**
DDS policies for reliability, durability, history, and latency. Configurable per-topic in ROS 2.

**Quadruped**
Four-legged robot. Examples: Boston Dynamics Spot, Unitree Go2, ANYmal.

## R

**Ray Tracing**
Rendering technique that simulates light physics for photo-realistic images. Required by Isaac Sim (needs RTX GPU).

**RealSense**
Intel's depth camera series (D400 stereo, L500 LiDAR). Popular for indoor robotics.

**rclpy**
ROS 2 Python client library. Used to create nodes, publishers, subscribers, services in Python.

**RGB-D**
Color (RGB) + Depth (D) image. Provides both appearance and 3D structure.

**RL (Reinforcement Learning)**
Machine learning paradigm where agents learn by trial-and-error with reward signals.

**ROS (Robot Operating System)**
Middleware framework for robot software development. ROS 2 is the current generation.

**RTX**
NVIDIA GPU architecture with real-time ray tracing hardware. Required for Isaac Sim.

## S

**SDF (Simulation Description Format)**
XML format for describing robots, environments, and sensors in Gazebo.

**Sensor Fusion**
Combining data from multiple sensors (cameras, LIDAR, IMU) for robust perception.

**Service (ROS 2)**
Synchronous request-response communication. Client sends request, server responds.

**Sim-to-Real**
Transferring policies trained in simulation to physical robots. Challenging due to reality gap.

**SLAM (Simultaneous Localization and Mapping)**
Building a map while simultaneously tracking robot's position in that map.

**Subscriber (ROS 2)**
Node that receives messages from a topic.

## T

**TF (Transform)**
ROS system for tracking coordinate frames. Essential for robots with multiple moving parts.

**Topic (ROS 2)**
Named bus for asynchronous message passing. Many publishers and subscribers can connect.

**TOPS (Tera Operations Per Second)**
Measure of AI performance for neural network inference. Jetson Orin Nano: 40 TOPS.

**TurtleBot**
Popular low-cost mobile robot platform for ROS education and research.

## U

**Unity**
Game engine used for robot visualization and simulation via Unity Robotics Hub.

**URDF (Unified Robot Description Format)**
XML format for describing robot kinematic and dynamic properties in ROS.

## V

**VLA (Vision-Language-Action)**
Multimodal AI system that combines vision (cameras), language (GPT), and action (robot control).

**VRAM (Video RAM)**
GPU memory. Isaac Sim requires 12GB+ for complex scenes with humanoid robots.

**VSLAM (Visual SLAM)**
SLAM using only camera images, without LIDAR. Examples: ORB-SLAM, RTAB-Map.

## W

**Workspace (ROS 2)**
Directory structure containing source code, build artifacts, and installation files for ROS packages.

**Whisper**
OpenAI's speech recognition model. Used in conversational robotics for voice input.

## X

**XML (eXtensible Markup Language)**
Format used for URDF, SDF, launch files, and configuration in ROS.

## Z

**ZMP (Zero-Moment Point)**
Point on the ground where total moment from contact forces equals zero. Critical for bipedal balance.

---

## Mathematical Notation

### Linear Algebra

- **x, y, z**: Cartesian coordinates (position)
- **θ (theta)**: Rotation angle
- **T**: Homogeneous transformation matrix (4x4)
- **R**: Rotation matrix (3x3)
- **p**: Position vector
- **v**: Velocity vector
- **q**: Joint angle vector

### Kinematics

- **DH(α, a, d, θ)**: Denavit-Hartenberg transformation
- **J**: Jacobian matrix (maps joint velocities to end-effector velocities)
- **q̇ (q-dot)**: Joint velocity
- **ẋ (x-dot)**: End-effector velocity

### Dynamics

- **τ (tau)**: Torque
- **m**: Mass
- **I**: Moment of inertia
- **F**: Force
- **g**: Gravitational acceleration (9.81 m/s²)

---

## Acronyms Quick Reference

| Acronym | Full Term |
|---------|-----------|
| API | Application Programming Interface |
| CNN | Convolutional Neural Network |
| CPU | Central Processing Unit |
| DDS | Data Distribution Service |
| DOF | Degrees of Freedom |
| FK | Forward Kinematics |
| FPS | Frames Per Second |
| GPU | Graphics Processing Unit |
| GUI | Graphical User Interface |
| HRI | Human-Robot Interaction |
| Hz | Hertz (cycles per second) |
| IK | Inverse Kinematics |
| IMU | Inertial Measurement Unit |
| LiDAR | Light Detection and Ranging |
| LLM | Large Language Model |
| ML | Machine Learning |
| MPC | Model Predictive Control |
| NLU | Natural Language Understanding |
| PID | Proportional-Integral-Derivative |
| QoS | Quality of Service |
| RGB-D | Red-Green-Blue + Depth |
| RL | Reinforcement Learning |
| ROS | Robot Operating System |
| RTX | Ray Tracing Texel eXtreme |
| SDK | Software Development Kit |
| SDF | Simulation Description Format |
| SLAM | Simultaneous Localization and Mapping |
| TF | Transform (ROS coordinate frames) |
| TOPS | Tera Operations Per Second |
| TTS | Text-to-Speech |
| URDF | Unified Robot Description Format |
| VLA | Vision-Language-Action |
| VRAM | Video Random Access Memory |
| VSLAM | Visual SLAM |
| ZMP | Zero-Moment Point |

---

**Last Updated**: 2025-12-07

**Note**: This glossary focuses on terms used in this course. For comprehensive robotics terminology, see the [Springer Handbook of Robotics](https://www.springer.com/gp/book/9783319325507).
