---
sidebar_position: 100
title: Hardware Requirements
description: Complete hardware specifications and comparison for Physical AI development
---

# Hardware Requirements & Setup Guide

## Overview

This course supports three development tiers, from pure simulation to physical robot development. Choose the tier that matches your goals and budget.

---

## Development Tier Comparison

| Tier | Purpose | Approximate Cost | Modules Supported |
|------|---------|------------------|-------------------|
| **Tier 1: Digital Twin Workstation** | Software development, simulation only | $1,000 - $3,000 | All modules (simulation) |
| **Tier 2: Physical AI Edge Kit** | Edge computing + basic robot hardware | $2,500 - $8,000 | Modules 1-3 + simple Module 4 |
| **Tier 3: Robot Lab Setup** | Full humanoid development platform | $15,000 - $100,000+ | All modules (hardware + simulation) |

---

## Tier 1: Digital Twin Workstation

**Best for**: Students, researchers, algorithm development without physical hardware

### Recommended Specifications

| Component | Minimum | Recommended | Optimal |
|-----------|---------|-------------|---------|
| **CPU** | Intel i5-10400 / AMD Ryzen 5 3600 (6 cores) | Intel i7-12700 / AMD Ryzen 7 5800X (8-12 cores) | Intel i9-13900K / AMD Ryzen 9 7950X (16+ cores) |
| **RAM** | 16 GB DDR4 | 32 GB DDR4 | 64 GB DDR4/DDR5 |
| **GPU** | NVIDIA GTX 1660 (6GB VRAM) | NVIDIA RTX 3060 (12GB VRAM) | NVIDIA RTX 4080 (16GB VRAM) |
| **Storage** | 512 GB NVMe SSD | 1 TB NVMe SSD + 2 TB HDD | 2 TB NVMe SSD + 4 TB HDD |
| **OS** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS (dual boot with Windows) | Ubuntu 22.04 LTS (native) |
| **Network** | Gigabit Ethernet | Gigabit Ethernet + WiFi 6 | 2.5 Gbps Ethernet + WiFi 6E |

### Why These Specs?

- **CPU**: ROS 2 compilation, multiple nodes, Gazebo physics simulation
- **RAM**: Gazebo (4-8GB), Unity (4-8GB), Isaac Sim (16GB+), ROS 2 nodes (4-8GB)
- **GPU**: Isaac Sim **requires** NVIDIA GPU with RTX (ray tracing), 12GB+ VRAM for complex scenes
- **Storage**: Isaac Sim alone is 50GB+, Unity projects 20GB+, ROS workspaces 10GB+

### GPU Requirements for NVIDIA Isaac Sim

| GPU Model | VRAM | Isaac Sim Support | Recommended Use |
|-----------|------|-------------------|-----------------|
| GTX 1660 / 1660 Ti | 6 GB | ❌ Not supported (no RTX) | Gazebo, Unity only |
| RTX 3060 | 12 GB | ✅ Basic scenes | Module 3 - simple robots |
| RTX 3070 / 3080 | 8-10 GB | ✅ Moderate scenes | Module 3 - moderate complexity |
| RTX 4070 | 12 GB | ✅ Complex scenes | Module 3 & 4 - humanoids |
| RTX 4080 / 4090 | 16-24 GB | ✅ Production-ready | Module 4 - full VLA systems |
| RTX A4000 / A5000 | 16-24 GB | ✅ Professional | Research & development |

:::warning Isaac Sim Requires RTX
NVIDIA Isaac Sim requires **RTX-capable GPUs** for ray tracing. GTX series (even high-end) will not work. Budget at least an RTX 3060 (12GB) for this course.
:::

### Software Stack

```bash
# Operating System
Ubuntu 22.04 LTS (Jammy Jellyfish)

# Core Tools
- ROS 2 Humble Hawksbill
- Python 3.10+
- Gazebo Fortress or Garden
- Unity 2022 LTS
- NVIDIA Isaac Sim 2023.1.1+

# Development Tools
- Visual Studio Code with ROS extensions
- Docker (for containerized development)
- Git and GitHub CLI
```

### Sample Workstation Builds

#### Budget Build ($1,200)
- **CPU**: AMD Ryzen 5 5600 ($130)
- **GPU**: NVIDIA RTX 3060 12GB ($350)
- **RAM**: 32GB DDR4 ($80)
- **Storage**: 1TB NVMe SSD ($60)
- **MB + PSU + Case**: $300
- **Monitor**: 1080p 24" ($180)
- **Peripherals**: $100

**Limitations**: Isaac Sim limited to simple scenes, 1-2 robots max

#### Recommended Build ($2,500)
- **CPU**: AMD Ryzen 7 7700X ($300)
- **GPU**: NVIDIA RTX 4070 12GB ($600)
- **RAM**: 64GB DDR5 ($200)
- **Storage**: 2TB NVMe SSD ($120)
- **MB + PSU + Case**: $500
- **Monitor**: 1440p 27" ($350)
- **Peripherals**: $200
- **UPS**: $230

**Capabilities**: Full Isaac Sim support, multiple humanoid robots, VLA development

#### High-End Build ($5,000+)
- **CPU**: AMD Ryzen 9 7950X ($550)
- **GPU**: NVIDIA RTX 4090 24GB ($1,600)
- **RAM**: 128GB DDR5 ($400)
- **Storage**: 4TB NVMe SSD + 8TB HDD ($500)
- **MB + PSU + Case**: $800
- **Monitor**: 4K 32" + 1440p 27" ($1,000)
- **Peripherals**: $300
- **UPS**: $400

**Capabilities**: Production-ready, multiple simultaneous simulations, ML training

---

## Tier 2: Physical AI Edge Kit

**Best for**: Deploying algorithms to real robots, mobile robot development

### Core Components

| Component | Purpose | Options | Price Range |
|-----------|---------|---------|-------------|
| **Edge Computer** | On-robot processing | Jetson Orin Nano, Orin NX, Orin AGX | $500 - $2,000 |
| **Robot Platform** | Mobile base or arm | TurtleBot4, ROSbot 2.0, LoCoBot | $1,500 - $5,000 |
| **Sensors** | Perception | RealSense D435i, RPLidar A1, ZED 2i | $300 - $1,000 |
| **Power System** | Battery + management | LiPo batteries, BMS, charger | $200 - $500 |
| **Accessories** | Mounting, cables, tools | 3D printed parts, cables, tools | $200 - $400 |

### NVIDIA Jetson Comparison

| Model | Jetson Orin Nano (8GB) | Jetson Orin NX (16GB) | Jetson Orin AGX (64GB) |
|-------|------------------------|------------------------|------------------------|
| **GPU** | 1024 CUDA cores | 1024 CUDA cores | 2048 CUDA cores |
| **AI Performance** | 40 TOPS | 100 TOPS | 275 TOPS |
| **RAM** | 8 GB | 16 GB | 32-64 GB |
| **Power** | 7-15W | 10-25W | 15-60W |
| **Price** | $499 | $899 | $1,999 - $2,599 |
| **Best For** | Simple mobile robots | Moderate AI workloads | Humanoid robots, VLA |
| **Module Support** | 1-2 (limited Isaac) | 1-3 (good for perception) | 1-4 (full course) |

:::tip Jetson vs Desktop GPU
Jetson is for **deployment** on robots. You still need a **desktop workstation** for development, simulation, and training. Jetson runs inference, not simulation.
:::

### Recommended Robot Platforms

#### Entry-Level Mobile Robots

| Robot | Type | Sensors Included | ROS 2 Support | Price | Best For |
|-------|------|------------------|---------------|-------|----------|
| **TurtleBot4** | Differential drive | RPLidar, OAK-D, IMU | ✅ Native | $1,500 - $2,000 | SLAM, navigation (Module 1-2) |
| **ROSbot 2.0 PRO** | Differential drive | RPLidar, RealSense, IMU | ✅ Native | $2,800 | Outdoor navigation, research |
| **LoCoBot WX250** | Differential drive + arm | RealSense, RPLidar, 5-DOF arm | ✅ Native | $3,500 | Manipulation + nav (Module 1-3) |
| **Unitree Go2** | Quadruped | Depth cameras, IMU, lidar (optional) | ✅ SDK available | $1,600 - $3,500 | Locomotion, terrain traversal |

#### Mid-Range Platforms with Manipulation

| Robot | Type | Degrees of Freedom | Payload | Price | Best For |
|-------|------|-------------------|---------|-------|----------|
| **WidowX 250** | Robotic arm | 6-DOF | 500g | $1,800 | Manipulation only |
| **Franka Emika Panda** | Collaborative arm | 7-DOF | 3kg | $15,000 | Research, precise manipulation |
| **Universal Robots UR5e** | Industrial arm | 6-DOF | 5kg | $25,000 | Industrial applications |
| **Kinova Gen3** | Robotic arm | 7-DOF | 1kg - 2.5kg | $15,000 - $30,000 | Research, assistive robotics |

:::info Robot Platforms vs Simulation
For **Modules 1-3**, simulation is sufficient. Physical robots are optional but enhance learning. For **Module 4 humanoid development**, simulation is recommended unless you have access to a research lab.
:::

### Sensor Options

#### Cameras

| Sensor | Type | Resolution | FPS | Range | Price | Use Case |
|--------|------|------------|-----|-------|-------|----------|
| **Intel RealSense D435i** | RGB-D (Stereo) | 1280x720 | 90 | 0.3-10m | $350 | Indoor SLAM, object detection |
| **Intel RealSense L515** | RGB-D (LiDAR) | 1920x1080 | 30 | 0.25-9m | $950 | High-accuracy 3D scanning |
| **Luxonis OAK-D** | Stereo + AI | 1920x1080 | 60 | 0.2-20m | $299 | Edge AI, depth perception |
| **ZED 2i** | Stereo + IMU | 4416x1242 | 15-120 | 0.5-20m | $700 | Outdoor SLAM, VIO |

#### LIDAR

| Sensor | Type | Range | Angular Resolution | Scan Rate | Price | Use Case |
|--------|------|-------|-------------------|-----------|-------|----------|
| **RPLidar A1M8** | 2D | 12m | 1° | 5.5 Hz | $100 | Indoor navigation |
| **Hokuyo UST-10LX** | 2D | 10m | 0.25° | 40 Hz | $1,500 | Precision mapping |
| **Livox Mid-360** | 3D (360°) | 40m | Dense | 10 Hz | $600 | Outdoor SLAM, point clouds |
| **Velodyne VLP-16** | 3D (360°) | 100m | 16 channels | 10-20 Hz | $4,000 | Autonomous vehicles |

---

## Tier 3: Robot Lab Setup

**Best for**: Research labs, advanced students, commercial development

### Humanoid Robot Options

| Robot | Height | DOF | Compute | Sensors | Price | Availability |
|-------|--------|-----|---------|---------|-------|--------------|
| **Unitree H1** | 180cm | 25+ | Jetson Orin | Depth cameras, IMU, force sensors | $90,000 | 2024 delivery |
| **Unitree G1** | 130cm | 23 | Jetson Orin NX | Depth cameras, joint encoders | $16,000 | Available |
| **Boston Dynamics Atlas** | 180cm | 28 | Custom | Stereo cameras, IMU, force sensors | $150,000+ | Research only |
| **Agility Digit** | 160cm | 20 | Intel NUC | Stereo cameras, IMU | $250,000+ | Commercial lease |
| **Tesla Optimus** | 173cm | 40+ | Custom FSD | Cameras, force sensors | TBD | Development |
| **Figure 01** | 165cm | 40+ | Custom | Cameras, IMU | TBD | Development |

:::warning Humanoid Cost Reality
Full humanoid robots are **extremely expensive** ($16K - $250K+). For **Module 4**, we focus on simulation using Isaac Sim and Gazebo. Physical humanoid hardware is optional and only available in well-funded research labs.
:::

### Budget Alternative: DIY Humanoid

| Component | Example Product | Price | Purpose |
|-----------|----------------|-------|---------|
| **Microcontroller** | Raspberry Pi 4 8GB | $75 | Main controller |
| **Motor Controllers** | 2x PCA9685 16-Channel | $30 | Servo control |
| **Servo Motors** | 16x MG996R High Torque | $160 | Joints (simplified humanoid) |
| **Power Supply** | 6V 10A DC + voltage regulators | $50 | Power distribution |
| **Sensors** | MPU6050 IMU, ultrasonic sensors | $30 | Balance & obstacle detection |
| **Frame** | 3D printed parts or aluminum | $100 | Structure |
| **Camera** | Raspberry Pi Camera v2 | $30 | Vision |
| **Cables & Misc** | Wires, screws, connectors | $50 | Assembly |
| **Total** | | **$525** | 16-DOF humanoid platform |

**Capabilities**: Basic bipedal walking (with tuning), arm movements, vision processing
**Limitations**: No force control, limited payload, slower than commercial robots

---

## Cloud vs On-Premise Cost Analysis

For teams or individuals considering cloud-based development:

### Cloud GPU Instances (Monthly Costs)

| Provider | Instance Type | GPU | VRAM | vCPU | RAM | Cost/Hour | Monthly (8h/day, 20 days) |
|----------|--------------|-----|------|------|-----|-----------|---------------------------|
| **AWS EC2** | g5.xlarge | A10G | 24GB | 4 | 16GB | $1.01 | $162 |
| **AWS EC2** | g5.4xlarge | A10G | 24GB | 16 | 64GB | $2.03 | $325 |
| **Google Cloud** | n1-standard-8 + T4 | T4 | 16GB | 8 | 30GB | $0.74 | $118 |
| **Google Cloud** | n1-standard-8 + V100 | V100 | 16GB | 8 | 30GB | $2.48 | $397 |
| **Azure** | NC6s_v3 | V100 | 16GB | 6 | 112GB | $3.06 | $490 |
| **Paperspace** | P4000 | Quadro P4000 | 8GB | 8 | 30GB | $0.51 | $82 |
| **Lambda Labs** | 1x RTX 6000 Ada | RTX 6000 | 48GB | 14 | 200GB | $1.25 | $200 |

### Cost Comparison: 1-Year Development

| Setup | Initial Cost | Monthly Cost | 1-Year Total |
|-------|--------------|--------------|--------------|
| **Budget Workstation** (RTX 3060) | $1,200 | $0 (electricity ~$10) | $1,320 |
| **Recommended Workstation** (RTX 4070) | $2,500 | $0 (electricity ~$15) | $2,680 |
| **Cloud** (AWS g5.xlarge, 8h/day) | $0 | $162 | $1,944 |
| **Cloud** (AWS g5.4xlarge, 8h/day) | $0 | $325 | $3,900 |

:::tip When to Use Cloud
- **Short-term projects** (< 6 months): Cloud may be cheaper
- **Variable workload**: Only pay when you need it
- **Team collaboration**: Easy sharing of instances
- **Testing different GPUs**: Try before buying hardware

Use **on-premise** for long-term development (> 1 year) or if you work > 8 hours/day.
:::

---

## Version Compatibility Matrix

### Verified Platform Versions

| Platform | Version | Ubuntu | ROS 2 | Python | CUDA | Notes |
|----------|---------|--------|-------|--------|------|-------|
| **ROS 2** | Humble | 22.04 | ✅ | 3.10 | N/A | LTS until 2027 |
| **ROS 2** | Jazzy | 24.04 | ✅ | 3.12 | N/A | Latest (2024) |
| **Gazebo** | Fortress | 22.04 | Humble | 3.10 | N/A | LTS |
| **Gazebo** | Garden | 22.04 | Humble | 3.10 | N/A | Latest stable |
| **Unity** | 2022.3 LTS | Any | Any | N/A | N/A | Unity Robotics Hub compatible |
| **Unity** | 2023.2+ | Any | Any | N/A | N/A | Latest features |
| **Isaac Sim** | 2023.1.1 | 22.04 | Humble | 3.10 | 12.2+ | Minimum |
| **Isaac Sim** | 2024.1.0 | 22.04 | Humble/Jazzy | 3.10 | 12.2+ | Latest features |

### CUDA & NVIDIA Driver Versions

| CUDA Version | Min Driver Version (Linux) | Isaac Sim Support | PyTorch Support |
|--------------|---------------------------|-------------------|-----------------|
| 11.8 | 450.80 | ✅ (older versions) | ✅ 2.0+ |
| 12.1 | 525.60 | ✅ | ✅ 2.1+ |
| 12.2 | 535.54 | ✅ Recommended | ✅ 2.2+ |
| 12.3+ | 545.23+ | ✅ Latest | ✅ 2.3+ |

**Installation**:
```bash
# Check current CUDA version
nvidia-smi

# Install CUDA 12.2 (recommended)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda-repo-ubuntu2204-12-2-local_12.2.0-535.54.03-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-12-2-local_12.2.0-535.54.03-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-12-2-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda
```

---

## Network Requirements

### Bandwidth

| Use Case | Min | Recommended | Notes |
|----------|-----|-------------|-------|
| **ROS 2 local nodes** | N/A | Gigabit Ethernet | &lt;1ms latency critical |
| **Remote visualization** | 10 Mbps | 100 Mbps | VNC, remote desktop |
| **Cloud GPU training** | 50 Mbps | 500 Mbps | Dataset uploads |
| **OTA updates** | 10 Mbps | 100 Mbps | Isaac Sim is 50GB+ |

### Recommended Network Setup

**For Lab/Home Development**:
- Gigabit Ethernet switch (UniFi, TP-Link managed)
- WiFi 6 router for mobile robots
- Separate VLAN for robots (ROS_DOMAIN_ID isolation)

**For Remote Development**:
- ZeroTier or Tailscale for secure remote access
- VNC or NoMachine for GUI forwarding
- SSH with tmux for terminal-based work

---

## Storage Recommendations

### Capacity Planning

| Component | Storage Required | Notes |
|-----------|------------------|-------|
| Ubuntu 22.04 | 25 GB | Base OS |
| ROS 2 Humble | 5 GB | Full desktop install |
| Gazebo | 2 GB | With models |
| Unity | 30 GB | Editor + templates |
| Isaac Sim | 50 GB | Full installation |
| Datasets (perception) | 50-500 GB | ImageNet, COCO, custom |
| ML models | 10-50 GB | Checkpoints, trained weights |
| ROS workspaces | 10-20 GB | Source code, build artifacts |
| **Total Minimum** | **200 GB** | |
| **Recommended** | **1 TB SSD + 2 TB HDD** | SSD for OS/apps, HDD for datasets |

---

## Key Takeaways

1. **For this course**: Tier 1 (Digital Twin Workstation) with RTX 3060+ is sufficient
2. **GPU is critical**: NVIDIA RTX required for Isaac Sim (12GB+ VRAM recommended)
3. **Physical robots are optional**: Simulation provides full learning experience
4. **Cloud can work short-term**: But on-premise is cheaper for long-term (> 6 months)
5. **Version compatibility matters**: Use Ubuntu 22.04 + ROS 2 Humble + Isaac Sim 2023.1+

---

## Next Steps

1. **Choose your tier** based on goals and budget
2. **Procure hardware** (allow 2-4 weeks for delivery/setup)
3. **Install software stack** following Module 1 instructions
4. **Verify installation** with test commands
5. **Start Module 1** once environment is ready

---

**Questions?** Check the [Resources page](./resources.md) for hardware vendors, community forums, and troubleshooting guides.
