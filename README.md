# Sensor Fusion System

A comprehensive ROS2 Jazzy package for multi-sensor data fusion, monitoring, and analysis.

## 📋 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Package Structure](#package-structure)
- [Usage](#usage)
- [Nodes Description](#nodes-description)
- [Topics](#topics)
- [Parameters](#parameters)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

## 🎯 Overview

This package implements a complete sensor fusion system that simulates, processes, and analyzes data from multiple sensors including IMU, GPS, Temperature, and Barometer sensors. The system includes real-time monitoring, anomaly detection, data logging, and statistical analysis.

## ✨ Features

- **Multi-Sensor Simulation**: Realistic sensor data with configurable noise
- **Real-time Data Fusion**: Combines multiple sensor inputs for state estimation
- **Anomaly Detection**: Automated alert system for out-of-range values
- **Data Logging**: Comprehensive logging with timestamps and statistics
- **Data Visualization**: Real-time plotting capabilities
- **Configurable QoS**: Appropriate Quality of Service profiles for different sensors
- **Parameter Server**: Dynamic configuration through ROS2 parameters
- **Launch Files**: Easy deployment of all nodes
- **Unit Tests**: Comprehensive test coverage

## 🏗️ System Architecture

```
┌─────────────────┐
│ Sensor Publisher│
│   (10/2/1 Hz)   │
└────────┬────────┘
         │
         ├─── /imu/data (10Hz)
         ├─── /gps/fix (1Hz)
         ├─── /temperature/data (2Hz)
         └─── /barometer/data (2Hz)
                │
    ┌───────────┼───────────┐
    │           │           │
┌───▼────┐ ┌───▼────┐ ┌───▼──────┐
│  Data  │ │ Alert  │ │  Fusion  │
│ Logger │ │ System │ │   Node   │
└────────┘ └───┬────┘ └────┬─────┘
               │            │
          /alerts      /fused_data
```

## 📦 Prerequisites

### System Requirements
- Ubuntu 22.04 (Jammy Jellyfish) or later
- ROS2 Jazzy Jalisco
- Python 3.10+
- WSL2 (if on Windows)

### WSL Setup (for Windows Users)

If you're using WSL, ensure you have:

```bash
# Update WSL to WSL2
wsl --set-default-version 2

# Update Ubuntu packages
sudo apt update && sudo apt upgrade -y

# Install required dependencies
sudo apt install -y python3-pip python3-dev python3-matplotlib
```

## 🚀 Installation

### 1. Install ROS2 Jazzy (if not already installed)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Install development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

### 2. Create Workspace and Clone Package

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create the package
ros2 pkg create sensor_fusion_system --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs std_msgs

# Navigate to package
cd sensor_fusion_system
```

### 3. Install Python Dependencies

```bash
# Install required Python packages
pip3 install numpy matplotlib scipy
```

### 4. Initialize rosdep (first time only)

```bash
sudo rosdep init
rosdep update
```

### 5. Install Package Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 6. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select sensor_fusion_system
source install/setup.bash
```

## 📁 Package Structure

```
sensor_fusion_system/
├── sensor_fusion_system/
│   ├── __init__.py
│   ├── sensor_publisher.py      # Simulates all sensors
│   ├── data_logger.py           # Logs and analyzes data
│   ├── alert_system.py          # Monitors and alerts
│   ├── sensor_fusion.py         # Fuses sensor data
│   └── utils/
│       ├── __init__.py
│       ├── filters.py           # Signal filtering utilities
│       └── visualizer.py        # Data visualization
├── launch/
│   ├── sensor_fusion.launch.py # Launch all nodes
│   └── visualization.launch.py  # Launch with visualization
├── config/
│   └── sensor_params.yaml       # Configuration parameters
├── test/
│   ├── test_sensor_publisher.py
│   ├── test_data_logger.py
│   ├── test_alert_system.py
│   └── test_sensor_fusion.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 🎮 Usage

### Running Individual Nodes

```bash
# Terminal 1: Sensor Publisher
ros2 run sensor_fusion_system sensor_publisher

# Terminal 2: Data Logger
ros2 run sensor_fusion_system data_logger

# Terminal 3: Alert System
ros2 run sensor_fusion_system alert_system

# Terminal 4: Fusion Node
ros2 run sensor_fusion_system sensor_fusion
```

### Using Launch File (Recommended)

```bash
# Launch all nodes
ros2 launch sensor_fusion_system sensor_fusion.launch.py

# Launch with visualization
ros2 launch sensor_fusion_system visualization.launch.py
```

### Viewing Topics

```bash
# List all topics
ros2 topic list

# Echo specific topic
ros2 topic echo /imu/data
ros2 topic echo /gps/fix
ros2 topic echo /alerts
ros2 topic echo /fused_data

# Check topic frequency
ros2 topic hz /imu/data
```

### Parameter Configuration

```bash
# List parameters
ros2 param list /sensor_publisher

# Get parameter value
ros2 param get /sensor_publisher imu_noise_level

# Set parameter value
ros2 param set /sensor_publisher imu_noise_level 0.05
```

## 🔧 Nodes Description

### 1. Sensor Publisher Node

**File**: `sensor_publisher.py`

Simulates multiple sensors with realistic noise models.

**Published Topics**:
- `/imu/data` (sensor_msgs/Imu) - 10 Hz
- `/gps/fix` (sensor_msgs/NavSatFix) - 1 Hz
- `/temperature/data` (sensor_msgs/Temperature) - 2 Hz
- `/barometer/data` (sensor_msgs/FluidPressure) - 2 Hz

**Parameters**:
- `imu_noise_level` (double, default: 0.01)
- `gps_noise_level` (double, default: 0.0001)
- `temp_noise_level` (double, default: 0.5)
- `pressure_noise_level` (double, default: 100.0)

### 2. Data Logger Node

**File**: `data_logger.py`

Subscribes to all sensor topics and logs data with statistical analysis.

**Subscribed Topics**:
- `/imu/data`
- `/gps/fix`
- `/temperature/data`
- `/barometer/data`

**Features**:
- Timestamps all data
- Calculates min, max, average
- Prints summary every 5 seconds
- Exports logs to CSV files

### 3. Alert System Node

**File**: `alert_system.py`

Monitors sensors for anomalies and publishes alerts.

**Subscribed Topics**:
- All sensor topics

**Published Topics**:
- `/alerts` (std_msgs/String)

**Alert Conditions**:
- IMU acceleration > 20 m/s²
- GPS coordinates out of valid range
- Temperature < -40°C or > 85°C
- Pressure < 30000 or > 120000 Pa

### 4. Sensor Fusion Node

**File**: `sensor_fusion.py`

Combines multiple sensor inputs for improved state estimation.

**Subscribed Topics**:
- `/imu/data`
- `/gps/fix`
- `/barometer/data`
- `/temperature/data`

**Published Topics**:
- `/fused_data` (geometry_msgs/PoseStamped)

**Features**:
- Kalman filter implementation
- Altitude calculation from barometer
- Temperature-compensated pressure readings
- Moving average filter

## 📊 Topics

| Topic | Message Type | Rate | QoS | Description |
|-------|-------------|------|-----|-------------|
| `/imu/data` | sensor_msgs/Imu | 10 Hz | BEST_EFFORT | Inertial measurement data |
| `/gps/fix` | sensor_msgs/NavSatFix | 1 Hz | RELIABLE | GPS position fix |
| `/temperature/data` | sensor_msgs/Temperature | 2 Hz | RELIABLE | Temperature readings |
| `/barometer/data` | sensor_msgs/FluidPressure | 2 Hz | RELIABLE | Pressure/altitude data |
| `/alerts` | std_msgs/String | event | RELIABLE | System alerts |
| `/fused_data` | geometry_msgs/PoseStamped | 10 Hz | RELIABLE | Fused sensor output |

## ⚙️ Parameters

### Sensor Publisher Parameters

```yaml
sensor_publisher:
  ros__parameters:
    # Noise levels
    imu_noise_level: 0.01
    gps_noise_level: 0.0001
    temp_noise_level: 0.5
    pressure_noise_level: 100.0
    
    # Initial positions
    initial_latitude: 31.9522
    initial_longitude: 35.9450
    initial_altitude: 800.0
```

### Data Logger Parameters

```yaml
data_logger:
  ros__parameters:
    log_directory: "~/sensor_logs"
    summary_interval: 5.0
    export_format: "csv"
```

## 🧪 Running Tests

```bash
# Run all tests
cd ~/ros2_ws
colcon test --packages-select sensor_fusion_system

# View test results
colcon test-result --verbose
```

## 🐛 Troubleshooting

### Issue: "Package not found"

```bash
# Re-source the workspace
source ~/ros2_ws/install/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Issue: "Permission denied" on WSL

```bash
# Fix permissions
sudo chmod +x ~/ros2_ws/src/sensor_fusion_system/sensor_fusion_system/*.py
```

### Issue: Matplotlib not displaying on WSL

```bash
# Install X server for WSL
# Download and install VcXsrv or X410 on Windows
# Then in WSL:
export DISPLAY=:0
```

### Issue: Topics not showing

```bash
# Check if nodes are running
ros2 node list

# Check node info
ros2 node info /sensor_publisher

# Verify topic list
ros2 topic list -v
```

## 📈 Performance Tips

- Use appropriate QoS profiles (BEST_EFFORT for high-frequency data)
- Monitor CPU usage: `top` or `htop`
- Check network latency: `ros2 topic delay /imu/data`
- Profile nodes: `ros2 run sensor_fusion_system sensor_publisher --ros-args --log-level debug`

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Open a Pull Request

## 📝 License

This project is licensed under the Apache 2.0 License.

## 👤 Author

Developed for ROS2 Jazzy Jalisco

## 📚 Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [ROS2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [sensor_msgs Documentation](https://github.com/ros2/common_interfaces/tree/jazzy/sensor_msgs)

---

**Note**: This package is designed for educational and testing purposes. For production use, additional safety measures and validation should be implemented.
