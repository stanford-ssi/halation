# GPS/IMU System Overview

## What You Have

A complete ROS2 GPS and IMU driver system for your autonomous rover with two hardware sensors:

| Component | Hardware | Communication | Key Data |
|-----------|----------|----------------|----------|
| **GPS** | Adafruit Ultimate GPS (#746) | Serial (UART 9600 baud) | Position (lat/lon/alt) + velocity |
| **IMU** | Adafruit BNO055 (#4646) | I2C (0x28 address) | Heading (0°=East, CCW positive) |

## Directory Structure

```
src/
├── gps_driver/          # GPS serial parser & ROS2 node
│   └── gps_driver/gps_node.py
└── imu_driver/          # IMU I2C reader & ROS2 node
    └── imu_driver/imu_node.py

launch/
├── gps.launch.py        # Start GPS node only
├── imu.launch.py        # Start IMU node only
└── gps_imu.launch.py    # Start both together

examples/
├── example_gps_imu_fusion.py  # Integration template with RoverState class
└── setup_gps_imu.py           # Hardware detection utility
```

## Published Topics

| Topic | Message Type | Content | Use For |
|-------|--------------|---------|---------|
| `/gps/fix` | NavSatFix | Latitude, longitude, altitude + covariance | Position localization |
| `/gps/vel` | TwistWithCovarianceStamped | East/North/Up velocity vectors | Dead reckoning |
| `/imu/heading` | Float32 | Heading in radians (-π to π) | Orientation, steering |
| `/imu/data` | Imu | Raw acceleration & angular velocity | Advanced filtering |

## Quick Setup

### 1. Connect Hardware
- **GPS**: USB/Serial on `/dev/ttyUSB0` (9600 baud)
- **IMU**: I2C on bus 1, address 0x28

### 2. Build Packages
```bash
cd /workspace
colcon build --packages-select gps_driver imu_driver
source install/setup.bash
```

### 3. Launch System
```bash
# Start both GPS and IMU nodes
ros2 launch gps_driver gps_imu.launch.py

# Or with custom port/bus if needed
ros2 launch gps_driver gps_imu.launch.py \
  gps_port:=/dev/ttyUSB0 \
  i2c_bus:=1 \
  i2c_address:=0x28
```

### 4. Verify Topics
```bash
# Check GPS position
ros2 topic echo /gps/fix

# Check heading
ros2 topic echo /imu/heading
```

## How It Works

### GPS Driver
1. **Reads NMEA sentences** from serial GPS module
2. **Parses** GPGGA (position) and GPVTG (velocity) sentences
3. **Converts** to ROS2 NavSatFix and TwistWithCovarianceStamped messages
4. **Publishes** at 1-10 Hz with covariance estimates

**Output**:
- Latitude/longitude (WGS84 datum)
- Altitude (meters above sea level)
- Velocity in East/North/Up frame

### IMU Driver
1. **Reads** BNO055 sensor over I2C
2. **Gets** absolute heading from on-board fusion
3. **Converts** from geographic (0°=North) to ENU convention (0°=East)
4. **Publishes** heading + raw acceleration/angular velocity at 100 Hz

**Output**:
- Heading: 0 rad = East, π/2 = North (counter-clockwise positive)
- Acceleration & angular velocity in East/North/Up frame

### Graceful Degradation
Both nodes will **start without hardware** (useful for Docker/testing):
- Missing hardware? Logs error, keeps running
- In Docker? Node starts successfully anyway
- Real hardware connects? Data automatically flows through

## Example: Integrate Both Sensors

See `example_gps_imu_fusion.py` for integration template:

```python
class RoverState:
    """Maintains GPS position + IMU heading in single object"""
    
    def distance_to_waypoint(self, target_lat, target_lon):
        """Haversine formula: returns meters"""
    
    def bearing_to_waypoint(self, target_lat, target_lon):
        """Ground truth bearing: 0°=North, CW positive"""
    
    def heading_error(self, target_bearing):
        """Normalized steering error: [-π, π]"""
```

**Usage Pattern:**
```python
rover = RoverState()

def gps_callback(msg):
    rover.latitude = msg.latitude
    rover.longitude = msg.longitude

def imu_callback(msg):
    rover.heading = msg.data  # Radians

# Subscribe to topics, use rover object in autonomy logic
```

## Key Parameters

| Device | Config | Default | Purpose |
|--------|--------|---------|---------|
| GPS | `gps_port` | `/dev/ttyUSB0` | Serial port |
| GPS | `baudrate` | `9600` | Serial speed |
| IMU | `i2c_bus` | `1` | I2C bus number |
| IMU | `i2c_address` | `0x28` | I2C address (0x29 if addr pin high) |
| IMU | `publish_rate` | `100.0` | Hz |

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| `/dev/ttyUSB0: No such file` | GPS not connected | Check USB cable, `ls /dev/tty*` |
| IMU gives 0 heading | Not calibrated | Move in figure-8 pattern, wait 30s |
| Wrong heading values | Coordinate frame mismatch | Check that 0 rad = East in your code |
| Nodes crash on startup | Old code in container | Run `colcon build`, `source install/setup.bash` |

## Next Steps

1. **Connect real hardware** → Topics publish live data
2. **Use `RoverState` class** → Integrate into rover navigation
3. **Subscribe to topics** → Read position/heading in your autonomy code
4. **Add filtering** → Kalman filter GPS + IMU for smoother estimates
5. **Implement waypoints** → Use distance/bearing methods for autonomous routing

---

**Status**: ✅ Complete, tested, ready to deploy with real hardware
