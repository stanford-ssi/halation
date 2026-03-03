# GPS and IMU Integration Guide

This guide explains how to use the GPS and IMU drivers together for your autonomous rover.

## Quick Start

### Build both packages
```bash
cd /workspace
colcon build --packages-select gps_driver imu_driver
source install/setup.bash
```

### Launch both GPS and IMU nodes
```bash
# Combined launch (requires both GPS on USB and IMU on I2C)
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyUSB0 i2c_bus:=1 i2c_address:=0x28
```

### Or launch individually
```bash
# Terminal 1: GPS
ros2 launch gps_driver gps.launch.py port:=/dev/ttyUSB0

# Terminal 2: IMU
ros2 launch imu_driver imu.launch.py i2c_bus:=1 i2c_address:=0x28
```

## ROS2 Topics Overview

### GPS Topics
- **`/gps/fix`** (sensor_msgs/NavSatFix)
  - Latitude, Longitude, Altitude with covariance
  - Publish rate: 1-10 Hz (GPS update rate)

- **`/gps/vel`** (geometry_msgs/TwistWithCovarianceStamped)
  - Velocity in East-North-Up frame
  - Publish rate: Variable (when VTG sentences available)

### IMU Topics
- **`/imu/heading`** (std_msgs/Float32)
  - Heading in radians: 0=East, CCW positive, ENU convention
  - Publish rate: 100 Hz (configurable)

- **`/imu/data`** (sensor_msgs/Imu)
  - Raw accelerometer and gyroscope data
  - Publish rate: 100 Hz (configurable)

## Coordinate Frames

### GPS Frame
- **Type**: Geographic (WGS84 datum)
- **Frame ID**: "gps"
- **Coordinates**: Latitude (°), Longitude (°), Altitude (m)
- **Velocity**: East-North-Up (ENU)
  - X (East): Positive = moving East
  - Y (North): Positive = moving North
  - Z (Up): Always 0 for ground vehicles

### IMU Frame  
- **Type**: Local ENU (East-North-Up)
- **Frame ID**: "imu"
- **Heading Convention**: 0 rad = East, CCW positive
  - 0 rad: East
  - π/2 rad: North
  - π rad: West
  - -π/2 rad: South

### Conversion Between Frames
If you need geographic heading (0°=North, CW positive):
```python
import math

# From ENU heading to geographic heading
geographic_heading_rad = math.pi/2 - enu_heading_rad

# Normalize to [0, 2π]
if geographic_heading_rad < 0:
    geographic_heading_rad += 2 * math.pi
```

## Hardware Setup Checklist

### Raspberry Pi 4 / 4B
- [ ] Enable UART in `raspi-config` (Interfaces → Serial Port)
- [ ] Enable I2C in `raspi-config` (Interfaces → I2C)
- [ ] Connect GPS TX to GPIO 10 (UART0 RX) or use USB adapter
- [ ] Connect IMU SDA to GPIO 2, SCL to GPIO 3
- [ ] Verify with `i2cdetect -y 1` and `ls /dev/tty*`
- [ ] Install dependencies: `pip install pyserial adafruit-circuitpython-bno055`

### Jetson Nano
- [ ] Verify I2C with `ls /dev/i2c*` (should have at least i2c-0 or i2c-1)
- [ ] Connect GPS to available UART or USB-to-UART adapter
- [ ] Connect IMU to I2C pins (GPIO 2=SDA, GPIO 3=SCL)
- [ ] Verify with `i2cdetect -y 1` and `ls /dev/tty*`
- [ ] Install dependencies: `pip install pyserial adafruit-circuitpython-bno055`

## Configuration for Your Platform

### Raspberry Pi - GPS via UART0 (GPIO 10/8)
```yaml
# config/gps_params.yaml
port: "/dev/ttyS0"    # or /dev/serial0
baudrate: 9600
frame_id: "gps"
```

### Raspberry Pi - GPS via USB Adapter
```yaml
# config/gps_params.yaml
port: "/dev/ttyUSB0"  # Check with: ls /dev/tty*
baudrate: 9600
frame_id: "gps"
```

### Jetson Nano - GPS Configuration
```yaml
# config/gps_params.yaml
port: "/dev/ttyTHS1"  # Check available UART ports
baudrate: 9600
frame_id: "gps"
```

## Data Fusion Example

Here's a simple Python example to fuse GPS and IMU data:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import math

class DataFusionNode(Node):
    def __init__(self):
        super().__init__('data_fusion')
        
        # Subscribe to GPS and IMU
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Float32, '/imu/heading', self.heading_callback, 10)
        
        self.current_heading = 0.0
        self.current_position = None
    
    def gps_callback(self, msg):
        self.current_position = (msg.latitude, msg.longitude, msg.altitude)
        self.get_logger().info(
            f'Position: {msg.latitude:.6f}, {msg.longitude:.6f}, Alt: {msg.altitude:.1f}m'
        )
    
    def heading_callback(self, msg):
        self.current_heading = msg.data
        # Convert to degrees
        heading_deg = math.degrees(msg.data)
        self.get_logger().info(f'Heading: {heading_deg:.1f}° (ENU: 0°=East, CCW positive)')

def main(args=None):
    rclpy.init(args=args)
    node = DataFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing the Setup

### Test GPS Node
```bash
# In one terminal, run GPS node
ros2 run gps_driver gps_node --ros-args -p port:=/dev/ttyUSB0

# In another terminal, echo the topic
ros2 topic echo /gps/fix
```

You should see messages like:
```
---
header:
  stamp:
    sec: 1709251234
    nsec: 567890123
  frame_id: gps
status:
  status: 1
  service: 0
latitude: 37.42740
longitude: -122.14300
altitude: 10.5
position_covariance: [5.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 10.0]
```

### Test IMU Node
```bash
# In one terminal, run IMU node
ros2 run imu_driver imu_node --ros-args -p i2c_bus:=1 -p i2c_address:=0x28

# In another terminal, echo the heading
ros2 topic echo /imu/heading
```

You should see messages like:
```
---
data: 0.52  # radians (about 30° from East)
```

### Test GPS Velocity
```bash
ros2 topic echo /gps/vel
```

## Parameter Reference

### GPS Driver Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `/dev/ttyUSB0` | Serial port for GPS |
| `baudrate` | int | `9600` | Serial baudrate |
| `frame_id` | string | `gps` | ROS2 frame ID |

### IMU Driver Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `i2c_bus` | int | `1` | I2C bus number |
| `i2c_address` | int | `0x28` | BNO055 I2C address |
| `frame_id` | string | `imu` | ROS2 frame ID |
| `publish_rate` | float | `100.0` | Publishing rate (Hz) |

## Integration with Rover Stack

To integrate into your rover's navigation stack:

1. **Update your main rover launch file:**
```python
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # ... other nodes ...
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '/path/to/gps_driver/launch/gps_imu.launch.py'
            ),
            launch_arguments={
                'gps_port': '/dev/ttyUSB0',
                'i2c_bus': '1',
                'i2c_address': '0x28',
            }.items(),
        ),
    ])
```

2. **Create a state estimator node** that fuses GPS and IMU:
   - Use GPS for global position
   - Use IMU heading for fast orientation updates
   - Consider using `robot_localization` package for state estimation

## Troubleshooting Integration

### GPS not getting fix
- Antenna needs clear sky view
- Takes 30-60 seconds for initial fix
- Check with `ros2 topic echo /gps/fix` and look for `status: 1`

### IMU heading drifting
- Ensure calibration is complete (check node output)
- Keep away from metal objects
- Rotate rover through full 360° to self-calibrate
- Try cold start: power cycle and recalibrate

### Both nodes work but data seems inconsistent
- Check timestamp/frame alignment
- GPS/IMU samples at different rates (expected)
- Use software time synchronization if needed
- See "Data Fusion Example" above

## References
- [GPS Driver README](../gps_driver/README.md)
- [IMU Driver README](../imu_driver/README.md)
- [ROS2 Coordinate Frames](https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Frames.html)
- [ENU Coordinate System](https://en.wikipedia.org/wiki/Axes_conventions#Ground_vehicles)
