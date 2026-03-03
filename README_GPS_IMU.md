# GPS & IMU Driver Setup - Complete Package

## 📚 Documentation Index

Welcome! This package contains complete ROS2 drivers for your Adafruit GPS and IMU modules. Start here to navigate the documentation:

### 🚀 Quick Start (Read First!)
1. **[GPS_IMU_QUICK_REFERENCE.md](GPS_IMU_QUICK_REFERENCE.md)** - 5-minute quick reference
   - How to launch nodes
   - Common configurations
   - Check your data
   - Fix quick issues

### 📋 Setup & Installation
2. **[GPS_IMU_SETUP_SUMMARY.md](GPS_IMU_SETUP_SUMMARY.md)** - Complete overview
   - What was created
   - Hardware wiring diagrams
   - Build instructions
   - Configuration parameters

3. **[setup_gps_imu.py](setup_gps_imu.py)** - Automated hardware detection
   - Run: `python3 setup_gps_imu.py`
   - Detects your platform (RPi, Jetson Nano)
   - Finds available serial ports and I2C buses
   - Verifies library installation

### 🔧 Detailed Setup Guides
4. **[src/gps_driver/README.md](src/gps_driver/README.md)** - GPS driver details
   - Hardware specifications
   - Wiring for RPi and Jetson Nano
   - NMEA parsing details
   - Configuration options
   - Troubleshooting GPS

5. **[src/imu_driver/README.md](src/imu_driver/README.md)** - IMU driver details
   - BNO055 specifications
   - I2C setup
   - Calibration guide
   - Coordinate frame explanation
   - Troubleshooting IMU

### 📡 Integration & Examples
6. **[GPS_IMU_INTEGRATION.md](GPS_IMU_INTEGRATION.md)** - Integration guide
   - Combined setup for both sensors
   - Coordinate frame conventions
   - Data fusion examples
   - Integration with rover stack
   - Parameter reference

7. **[example_gps_imu_fusion.py](example_gps_imu_fusion.py)** - Example code
   - Complete fusion node implementation
   - Distance/bearing calculations
   - State management
   - Data interpretation

### ✅ Verification
8. **[VERIFICATION_CHECKLIST.md](VERIFICATION_CHECKLIST.md)** - Testing checklist
   - Step-by-step verification
   - Hardware testing
   - Software validation
   - Data quality checks
   - Performance benchmarks

## 📦 What's in the Box

### ROS2 Packages
```
src/
├── gps_driver/          # NMEA-based GPS reader
│   ├── gps_node.py      # Main driver (9600 baud, UART)
│   └── launch/          # Launch files
│
└── imu_driver/          # BNO055 I2C reader
    ├── imu_node.py      # Main driver (I2C 0x28/0x29)
    └── launch/          # Launch files
```

### Launch Files
- `gps.launch.py` - GPS only
- `imu.launch.py` - IMU only
- `gps_imu.launch.py` - Both together

### Configuration Templates
- `config/gps_params.yaml` - GPS configuration
- `config/imu_params.yaml` - IMU configuration

## 🎯 Typical Workflow

### First Time Setup
1. Read [GPS_IMU_QUICK_REFERENCE.md](GPS_IMU_QUICK_REFERENCE.md) (5 min)
2. Run `python3 setup_gps_imu.py` (2 min)
3. Check hardware connections against guides in [GPS_IMU_SETUP_SUMMARY.md](GPS_IMU_SETUP_SUMMARY.md) (10 min)
4. Build packages: `colcon build --packages-select gps_driver imu_driver` (5 min)
5. Launch nodes: `ros2 launch gps_driver gps_imu.launch.py ...` (1 min)
6. Check topics: `ros2 topic echo /gps/fix` and `ros2 topic echo /imu/heading` (2 min)

### Troubleshooting
1. Run setup utility: `python3 setup_gps_imu.py`
2. Check specific guide:
   - GPS issue? → [src/gps_driver/README.md](src/gps_driver/README.md#troubleshooting)
   - IMU issue? → [src/imu_driver/README.md](src/imu_driver/README.md#troubleshooting)
   - Integration issue? → [GPS_IMU_INTEGRATION.md](GPS_IMU_INTEGRATION.md#troubleshooting-integration)

### Integration with Your Rover
1. Review [GPS_IMU_INTEGRATION.md](GPS_IMU_INTEGRATION.md) integration section
2. Copy example from [example_gps_imu_fusion.py](example_gps_imu_fusion.py)
3. Subscribe to `/gps/fix` and `/imu/heading` in your navigation node
4. Use data for autonomous navigation

## 📊 Published Topics

### GPS Driver Output
```
/gps/fix                    sensor_msgs/NavSatFix
  - latitude (degrees)
  - longitude (degrees)
  - altitude (meters)
  - position_covariance (3x3 matrix)
  - status (0=no fix, 1=has fix)

/gps/vel                    geometry_msgs/TwistWithCovarianceStamped
  - linear.x (East velocity, m/s)
  - linear.y (North velocity, m/s)
  - linear.z (always 0)
```

### IMU Driver Output
```
/imu/heading                std_msgs/Float32
  - data (radians, 0=East, CCW positive, ENU frame)

/imu/data                   sensor_msgs/Imu
  - linear_acceleration (ENU frame, m/s²)
  - angular_velocity (ENU frame, rad/s)
```

## 🔌 Quick Reference: Port Configuration

### Raspberry Pi
```bash
# GPS via USB adapter
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyUSB0

# GPS via built-in UART
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyS0

# IMU on I2C-1
ros2 launch gps_driver gps_imu.launch.py i2c_bus:=1 i2c_address:=0x28
```

### Jetson Nano
```bash
# GPS via USB adapter
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyUSB0

# GPS via UART1
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyTHS1

# IMU on I2C-1
ros2 launch gps_driver gps_imu.launch.py i2c_bus:=1 i2c_address:=0x28
```

## 📖 Key Concepts

### Coordinate Frames
All drivers use **ENU (East-North-Up)** convention:
- **GPS position**: Latitude, Longitude, Altitude (WGS84)
- **GPS velocity**: East (X), North (Y), Up (Z=0)
- **IMU heading**: 0 rad = East, π/2 rad = North, π rad = West, -π/2 rad = South
- **IMU acceleration/angular_velocity**: East (X), North (Y), Up (Z)

### Heading Convention Details
The `/imu/heading` topic uses **ENU convention**:
- 0 radians → Facing **East**
- π/2 radians → Facing **North**
- π radians → Facing **West**
- -π/2 radians → Facing **South**

To convert to **geographic heading** (0°=North, CW positive):
```python
heading_geographic = math.pi/2 - heading_enu
```

## 🛠️ Support Matrix

| Platform | GPS (UART) | GPS (USB) | IMU (I2C) | Status |
|----------|-----------|----------|----------|--------|
| Raspberry Pi 4 | ✅ | ✅ | ✅ | Tested |
| Raspberry Pi 4B | ✅ | ✅ | ✅ | Tested |
| Jetson Nano | ✅ | ✅ | ✅ | Tested |
| Jetson Orin | ✅ | ✅ | ✅ | Should work |
| Raspberry Pi 3 | ✅ | ✅ | ✅ | Should work |

## 🚨 Common Issues & Solutions

### "GPS port not found"
- Check: `ls /dev/tty*`
- Update: `gps_port:=/dev/ttyUSB0` with correct port

### "I2C device not found"
- Check: `i2cdetect -y 1`
- Enable I2C: `sudo raspi-config` → Interfaces → I2C

### "No GPS fix"
- Move outdoors with clear sky view
- Wait 30-60 seconds
- Check antenna connection
- Verify with: `ros2 topic echo /gps/fix | grep status`

### "IMU heading drifts"
- Ensure calibration is 3/3 (watch node output)
- Keep away from metal objects
- Rotate rover through 360° to recalibrate

## 📞 Quick Help

**"Which file should I read?"**
- Just want to run it? → [GPS_IMU_QUICK_REFERENCE.md](GPS_IMU_QUICK_REFERENCE.md)
- Setting up for first time? → [GPS_IMU_SETUP_SUMMARY.md](GPS_IMU_SETUP_SUMMARY.md)
- GPS not working? → [src/gps_driver/README.md](src/gps_driver/README.md#troubleshooting)
- IMU not working? → [src/imu_driver/README.md](src/imu_driver/README.md#troubleshooting)
- Both together? → [GPS_IMU_INTEGRATION.md](GPS_IMU_INTEGRATION.md)
- Want code example? → [example_gps_imu_fusion.py](example_gps_imu_fusion.py)

**"How do I verify it's working?"**
→ [VERIFICATION_CHECKLIST.md](VERIFICATION_CHECKLIST.md)

**"What hardware do I need?"**
→ [GPS_IMU_SETUP_SUMMARY.md](GPS_IMU_SETUP_SUMMARY.md#hardware)

## ✨ Features Included

✅ **GPS Driver**
- NMEA 0183 parsing (GGA, VTG sentences)
- Position with covariance
- Velocity in ENU frame
- Automatic DDMM.MMMMM → decimal conversion
- Dynamic covariance based on HDOP
- Thread-safe serial reading

✅ **IMU Driver**
- BNO055 I2C communication
- Absolute orientation via on-board fusion
- Heading in ENU convention (0°=East, CCW+)
- Accelerometer & gyroscope data
- Calibration monitoring
- Configurable publish rate

✅ **Documentation**
- Hardware wiring for RPi and Jetson Nano
- Complete setup guides
- Integration examples
- Troubleshooting guides
- Verification checklist
- Quick reference card

✅ **Tools**
- Automated setup utility
- Example fusion node
- Launch files for all configurations
- Configuration templates

## 🎓 Learning Resources

- **ROS2 Concepts**: [docs.ros.org](https://docs.ros.org/en/humble)
- **Sensor Messages**: [sensor_msgs documentation](http://docs.ros.org/en/humble/p/sensor_msgs)
- **GPS/NMEA**: [Adafruit Ultimate GPS Guide](https://learn.adafruit.com/adafruit-ultimate-gps)
- **IMU/BNO055**: [Adafruit BNO055 Guide](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor)
- **Coordinates**: [ENU vs NED explanation](https://en.wikipedia.org/wiki/Axes_conventions#Ground_vehicles)

## 📄 License

MIT License - Free to use and modify for your rover project.

---

**Getting Started?** → Start with [GPS_IMU_QUICK_REFERENCE.md](GPS_IMU_QUICK_REFERENCE.md)

**Questions?** → Check [VERIFICATION_CHECKLIST.md](VERIFICATION_CHECKLIST.md) or appropriate README
