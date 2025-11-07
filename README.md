# Ros2-week-3
# Sensor Fusion System

## Overview
A comprehensive ROS2 Jazzy package for multi-sensor data fusion, monitoring, and analysis. This system simulates and processes data from IMU, GPS, Temperature, and Barometer sensors with real-time anomaly detection and data logging capabilities.

## Features
- **Multi-Sensor Publishing**: Realistic sensor data simulation with configurable noise
- **Data Fusion**: Kalman filtering and sensor fusion algorithms
- **Anomaly Detection**: Real-time monitoring and alert system
- **Data Logging**: Comprehensive logging with statistical analysis
- **Visualization**: Real-time plotting of sensor data
- **Configurable Parameters**: ROS2 parameter-based configuration
- **Quality of Service**: Appropriate QoS profiles for different data types

## System Architecture

```
┌─────────────────────┐
│ Sensor Publisher    │
│ - IMU (10 Hz)       │
│ - GPS (1 Hz)        │
│ - Temperature (2 Hz)│
│ - Barometer (2 Hz)  │
└──────────┬──────────┘
           │
           ├──────────────────┬───────────────┬──────────────┐
           │                  │               │              │
           ▼                  ▼               ▼              ▼
    ┌─────────────┐   ┌─────────────┐  ┌──────────┐  ┌──────────┐
    │ Data Logger │   │ Alert System│  │  Fusion  │  │Visualizer│
    └─────────────┘   └─────────────┘  └──────────┘  └──────────┘
```

## Package Structure

```
sensor_fusion_system/
├── sensor_fusion_system/
│   ├── __init__.py
│   ├── sensor_publisher.py
│   ├── data_logger.py
│   ├── alert_system.py
│   ├── sensor_fusion.py
│   └── visualizer.py
├── launch/
│   ├── sensor_fusion_launch.py
│   └── all_nodes_launch.py
├── config/
│   └── sensor_params.yaml
├── test/
│   ├── test_sensor_publisher.py
│   ├── test_data_logger.py
│   └── test_alert_system.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## Prerequisites

### System Requirements
- Ubuntu 22.04 (or WSL with Ubuntu 22.04)
- ROS2 Jazzy
- Python 3.10+

### Dependencies
```bash
sudo apt install ros-jazzy-desktop
sudo apt install python3-colcon-common-extensions
pip3 install numpy matplotlib
```

## Installation

### 1. Create Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone/Create Package
```bash
# If from repository
git clone <repository_url> sensor_fusion_system

# Or create manually
ros2 pkg create sensor_fusion_system --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs std_msgs
```

### 3. Build Package
```bash
cd ~/ros2_ws
colcon build --packages-select sensor_fusion_system
source install/setup.bash
```

## Usage

### Launch All Nodes
```bash
ros2 launch sensor_fusion_system all_nodes_launch.py
```

### Launch Individual Nodes

#### Sensor Publisher
```bash
ros2 run sensor_fusion_system sensor_publisher
```

#### Data Logger
```bash
ros2 run sensor_fusion_system data_logger
```

#### Alert System
```bash
ros2 run sensor_fusion_system alert_system
```

#### Sensor Fusion
```bash
ros2 run sensor_fusion_system sensor_fusion
```

#### Visualizer
```bash
ros2 run sensor_fusion_system visualizer
```

## Topics

| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/sensors/imu` | sensor_msgs/Imu | 10 Hz | IMU data (acceleration, gyroscope) |
| `/sensors/gps` | sensor_msgs/NavSatFix | 1 Hz | GPS coordinates |
| `/sensors/temperature` | sensor_msgs/Temperature | 2 Hz | Temperature readings |
| `/sensors/barometer` | sensor_msgs/FluidPressure | 2 Hz | Barometric pressure |
| `/sensors/fused` | geometry_msgs/PoseStamped | 10 Hz | Fused sensor output |
| `/alerts` | std_msgs/String | Event-based | Anomaly alerts |

## Parameters

### Sensor Publisher Parameters
```yaml
sensor_publisher:
  imu_noise: 0.01          # IMU noise standard deviation
  gps_noise: 0.0001        # GPS noise standard deviation
  temp_noise: 0.5          # Temperature noise (°C)
  baro_noise: 10.0         # Barometer noise (Pa)
```

### Alert System Parameters
```yaml
alert_system:
  temp_min: -40.0          # Minimum temperature (°C)
  temp_max: 85.0           # Maximum temperature (°C)
  altitude_max: 10000.0    # Maximum altitude (m)
  imu_accel_max: 50.0      # Maximum acceleration (m/s²)
```

## Configuration

Edit `config/sensor_params.yaml` to customize sensor parameters:

```bash
ros2 run sensor_fusion_system sensor_publisher --ros-args --params-file config/sensor_params.yaml
```

## Data Logging

Logs are saved to `~/.ros/sensor_logs/` with timestamps. Each log includes:
- Raw sensor readings
- Statistical analysis (min, max, average)
- Timestamps
- Alert history

## Quality of Service (QoS)

The package implements appropriate QoS profiles:

- **Sensor Data**: `BEST_EFFORT` reliability for high-frequency IMU data
- **GPS Data**: `RELIABLE` for critical position information
- **Alerts**: `RELIABLE` + `TRANSIENT_LOCAL` for critical notifications
- **Logs**: `RELIABLE` for data integrity

## Visualization

The visualizer node provides real-time plotting:
- IMU acceleration (X, Y, Z axes)
- Temperature trends
- Altitude from barometer
- GPS position (2D plot)

Press `Ctrl+C` to close visualization windows.

## Testing

### Run Unit Tests
```bash
cd ~/ros2_ws
colcon test --packages-select sensor_fusion_system
colcon test-result --verbose
```

### Manual Testing
```bash
# Terminal 1: Launch all nodes
ros2 launch sensor_fusion_system all_nodes_launch.py

# Terminal 2: Monitor topics
ros2 topic list
ros2 topic echo /sensors/imu
ros2 topic hz /sensors/imu

# Terminal 3: Check for alerts
ros2 topic echo /alerts
```

## Troubleshooting

### Issue: Nodes not starting
```bash
# Check if ROS2 is sourced
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Verify package is built
ros2 pkg list | grep sensor_fusion
```

### Issue: No data on topics
```bash
# Check if publisher is running
ros2 node list

# Verify topic connections
ros2 topic info /sensors/imu
```

### Issue: Import errors
```bash
# Install missing dependencies
pip3 install numpy matplotlib
sudo apt install ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs
```

## Performance Optimization

- **CPU Usage**: ~5-10% on modern systems
- **Memory**: ~50-100 MB per node
- **Network**: Minimal bandwidth (~1 Mbps for all topics)

## Future Enhancements

- [ ] Add Lidar sensor simulation
- [ ] Implement Extended Kalman Filter (EKF)
- [ ] Add machine learning-based anomaly detection
- [ ] Create web-based visualization dashboard
- [ ] Implement sensor calibration routines
- [ ] Add bag file recording and playback

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details.

## References

- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [sensor_msgs Package](https://github.com/ros2/common_interfaces/tree/jazzy/sensor_msgs)
- [ROS2 QoS Documentation](https://docs.ros.org/en/jazzy/Concepts/About-Quality-of-Service-Settings.html)

## Contact

For issues and questions, please open an issue on the GitHub repository.

## Acknowledgments

- ROS2 community for excellent documentation
- Contributors and testers

---

**Built with ❤️ using ROS
