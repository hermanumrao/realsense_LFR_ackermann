# realsense_LFR_ackermann

A ROS 2 package for an **autonomous line-following robot** designed for old warehouses and factories. It uses an **Intel RealSense D435i** depth camera to detect black tape markings on the floor, follow them, and stop for 5 seconds whenever a red marker is detected — enabling low-cost automation without replacing legacy hardware.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [ROS 2 Topics](#ros-2-topics)
- [Known Issues / Status](#known-issues--status)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This project targets **existing warehouses and factories** that rely on manual operations but cannot afford full infrastructure upgrades. By placing black tape lines on the floor and red stop markers at key locations, operators can guide the robot along a defined path — no laser scanners, no GPS, no expensive infrastructure needed.

The robot uses:
- A **RealSense D435i** for RGB + depth perception
- **Ackermann steering** for car-like, smooth navigation
- **ROS 2** as the middleware and autonomy stack

---

## Features

- **Line following** — detects and tracks configurable-width black tape on the floor
- **Red-area stop** — automatically halts for 5 seconds when a red marker is detected, then resumes
- **Configurable tape dimensions** — set the expected line width and length to match your floor layout
- **Ackermann-compatible** — steering commands are published as `AckermannDriveStamped` messages
- **ROS 2 native** — built on the ros2 autonomy stack

---

## Configs:

Please refer: https://github.com/hermanumrao/realsense_LFR_ackermann/tree/main/tayota_ws/src/realsense_line_follower

---

## System Architecture

```
RealSense D435i (RGB + Depth)
        │
        ▼
  realsense2_camera node
  (publishes /camera/color/image_raw)
        │
        ▼
  line_follower node  ◄──── configurable params (line width, length, color thresholds)
  (OpenCV-based detection)
        │
        ▼
  AckermannDriveStamped
        │
        ▼
  ros2_ackermann_motor_ctrl
  (Arduino Mega + Motor Driver + Servo)
        │
        ▼
       Robot
```

---

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| Intel RealSense D435i | RGB-D camera for line detection |
| Arduino Mega | Microcontroller for motor/servo control |
| Motor Driver | Compatible DC motor driver (e.g., L298N) |
| Servo Motor | For Ackermann steering |
| FlySky FS-i6 (optional) | RC transmitter/receiver for manual override |
| Robot chassis | Ackermann-steered platform |
| Floor markings | Black tape lines + red tape stop markers |

---

## Software Requirements

| Software | Version |
|----------|---------|
| Ubuntu | 22.04 (Jammy) |
| ROS 2 | Humble |
| Python | 3.10+ |
| OpenCV | 4.x |

---

## Dependencies

This package requires two external repositories:

### 1. ros2_ackermann_motor_ctrl
Motor control bridge between ROS 2 and the Arduino Mega.

```bash
git clone https://github.com/hermanumrao/ros2_ackermann_motor_ctrl.git
```

Provides the `AckermannDriveStamped` subscriber that relays commands to the motor driver and servo via serial communication.

### 2. RealSense ROS 2 Wrapper
Official Intel RealSense ROS 2 driver.

```bash
git clone -b ros2-master https://github.com/realsenseai/realsense-ros.git
```

Provides the `/camera/color/image_raw` and depth topics consumed by the line follower node.

---

## Installation

### 1. Set up workspace

```bash
mkdir -p ~/tayota_ws/src
cd ~/tayota_ws/src
```

### 2. Clone all required repositories

```bash
# Main package
git clone https://github.com/hermanumrao/realsense_LFR_ackermann.git

# Motor control dependency
git clone https://github.com/hermanumrao/ros2_ackermann_motor_ctrl.git

# RealSense ROS 2 driver
git clone -b ros2-master https://github.com/realsenseai/realsense-ros.git
```

### 3. Install ROS 2 package dependencies

```bash
cd ~/tayota_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### 5. Flash Arduino firmware

Upload the appropriate sketch from `ros2_ackermann_motor_ctrl/arduino_codes/` to your Arduino Mega using the Arduino IDE.

---

## Usage

### Launch the camera

```bash
ros2 launch realsense2_camera rs_launch.py
```

### Launch the line follower

```bash
ros2 launch realsense_LFR_ackermann line_follower.launch.py
```

### Launch motor controller

```bash
ros2 run arduino_motor ackermann_ctrl
```

Or launch everything together (if a combined launch file is available):

```bash
ros2 launch realsense_LFR_ackermann full_system.launch.py
```

---

## Configuration

Tape and detection parameters can be tuned to suit your floor and marking dimensions. Look for the parameter file at:

```
tayota_ws/src/realsense_LFR_ackermann/config/params.yaml
```

Key parameters:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `line_width_min` | Minimum detected line width (pixels) | `30` |
| `line_width_max` | Maximum detected line width (pixels) | `120` |
| `red_stop_duration` | Seconds to stop on red detection | `5.0` |
| `drive_speed` | Nominal forward speed | `0.3` |
| `camera_topic` | Input image topic | `/camera/color/image_raw` |

---

## ROS 2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Subscribed | RGB image from RealSense |
| `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | Published | Steering + throttle commands |
| `/line_follower/debug_image` | `sensor_msgs/Image` | Published | Annotated debug visualization |

---

## Known Issues / Status

> **This package is currently under active validation and testing.** Expect rough edges.

- Detection robustness varies with floor texture and ambient lighting conditions
- Red stop detection may trigger on surfaces with similar hues under certain lighting
- No formal test suite yet

Bug reports, feature requests, and pull requests are welcome and appreciated.

---

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Commit your changes: `git commit -m 'Add some feature'`
4. Push to the branch: `git push origin feature/my-feature`
5. Open a Pull Request

---

## License

You are free to modify and distribute just mention my name.

---

## Author

**hermanumrao** — [GitHub](https://github.com/hermanumrao)

Built as part of a broader ROS 2 autonomy stack for low-cost industrial automation.
