# GPS and IMU Driver Setup - Complete Summary

## 📦 What Was Created

I've set up complete ROS2 driver packages for your autonomous rover with support for:
- **Adafruit Ultimate GPS Breakout (Product #746)** - PA1616S chipset
- **Adafruit 9-DOF IMU (Product #4646)** - BNO055 chipset

## 📂 Package Structure

```
/workspace/src/
├── gps_driver/                    # GPS driver package
│   ├── gps_driver/
│   │   ├── __init__.py
│   │   └── gps_node.py            # Main GPS node (NMEA parser, publishers)
│   ├── launch/
│   │   ├── gps.launch.py          # Launch GPS node alone
│   │   └── gps_imu.launch.py      # Launch both GPS and IMU
│   ├── config/
│   │   └── gps_params.yaml        # Configuration template
│   ├── README.md                  # GPS-specific documentation
│   ├── package.xml                # Package metadata
│   └── setup.py                   # Installation configuration
│
├── imu_driver/                    # IMU driver package
│   ├── imu_driver/
│   │   ├── __init__.py
│   │   └── imu_node.py            # Main IMU node (BNO055 I2C reader)
│   ├── launch/
│   │   └── imu.launch.py          # Launch IMU node
│   ├── config/
│   │   └── imu_params.yaml        # Configuration template
│   ├── README.md                  # IMU-specific documentation
│   ├── package.xml                # Package metadata
│   └── setup.py                   # Installation configuration
│
└── GPS_IMU_INTEGRATION.md         # Integration guide
└── setup_gps_imu.py              # Platform detection utility
```

## 🎯 Published Topics

### GPS Driver
**`/gps/fix`** - `sensor_msgs/NavSatFix`
- Latitude, longitude, altitude with covariance
- Status indicating GPS fix quality
- Update rate: 1-10 Hz

**`/gps/vel`** - `geometry_msgs/TwistWithCovarianceStamped`
- Velocity in East-North-Up (ENU) frame
- Linear velocities in m/s
- Covariance matrices for uncertainty

### IMU Driver
**`/imu/heading`** - `std_msgs/Float32`
- Heading in radians (your primary requirement)
- Convention: 0 rad = East, CCW positive, ENU frame
- Range: [-π, π] radians
- Update rate: 100 Hz (configurable)

**`/imu/data`** - `sensor_msgs/Imu`
- Accelerometer data (East, North, Up axes)
- Gyroscope data (angular velocities)
- Update rate: 100 Hz (configurable)

## 🔧 Hardware Wiring

### GPS Module (UART)
**Raspberry Pi:**
```
GPS VIN    → Pi 5V (Pin 2/4)
GPS GND    → Pi GND (Pin 6/9/14/20/25/30/34/39)
GPS TX     → Pi GPIO 10 (UART0 RX) or USB adapter
GPS RX     → Pi GPIO 8 (UART0 TX) or USB adapter
```

**Jetson Nano:**
```
GPS VIN    → Nano 5V (Pin 2)
GPS GND    → Nano GND (Pin 6/9/14/25/30/34/39)
GPS TX/RX  → UART1 pins or USB adapter
```

### IMU Module (I2C)
**Both Raspberry Pi and Jetson Nano:**
```
IMU VIN    → 5V or 3.3V (Pin 2/4)
IMU GND    → GND (Pin 6/9/14/20/25/30/34/39)
IMU SDA    → GPIO 2 (I2C1 SDA - Pin 3)
IMU SCL    → GPIO 3 (I2C1 SCL - Pin 5)
```

Alternatively, use STEMMA QT/Qwiic connectors if available on your board.

## 🚀 Quick Start

### 1. Install Dependencies
```bash
# For GPS (serial)
pip install pyserial

# For IMU (I2C, CircuitPython)
pip install adafruit-circuitpython-bno055
pip install adafruit-circuitpython-busio
```

### 2. Verify Hardware
```bash
# Run the setup utility to detect hardware
python3 /workspace/setup_gps_imu.py

# Or manually verify
# GPS: ls /dev/tty*
# IMU: i2cdetect -y 1
```

### 3. Build Packages
```bash
cd /workspace
colcon build --packages-select gps_driver imu_driver
source install/setup.bash
```

### 4. Launch Nodes
```bash
# Both together
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyUSB0 i2c_bus:=1 i2c_address:=0x28

# Or separately
# Terminal 1:
ros2 launch gps_driver gps.launch.py port:=/dev/ttyUSB0

# Terminal 2:
ros2 launch imu_driver imu.launch.py i2c_bus:=1 i2c_address:=0x28
```

### 5. Test Topics
```bash
# In new terminal:
ros2 topic echo /gps/fix
ros2 topic echo /imu/heading
```

## 📋 Configuration Parameters

### GPS Driver (`gps_params.yaml`)
```yaml
port: "/dev/ttyUSB0"    # Serial port
baudrate: 9600          # Baudrate
frame_id: "gps"         # TF frame
```

### IMU Driver (`imu_params.yaml`)
```yaml
i2c_bus: 1              # I2C bus number
i2c_address: 0x28       # BNO055 address (0x28 or 0x29)
frame_id: "imu"         # TF frame
publish_rate: 100.0     # Publishing frequency (Hz)
```

## 🔍 Key Features

### GPS Driver
✅ NMEA sentence parsing (GGA for fix, VTG for velocity)
✅ Automatic coordinate conversion (DDMM.MMMMM → decimal degrees)
✅ Dynamic covariance based on HDOP
✅ Velocity calculation from GPS track and speed
✅ Thread-safe serial reading

### IMU Driver
✅ On-board sensor fusion (BNO055 handles it)
✅ Automatic calibration monitoring
✅ ENU frame conversion for robotics
✅ Configurable publish rate
✅ Heading output in ENU convention (0°=East, CCW positive)

## 📚 Documentation

Three comprehensive README files are provided:

1. **[gps_driver/README.md](src/gps_driver/README.md)** - GPS-specific details
   - Hardware connections
   - NMEA sentence details
   - Troubleshooting GPS issues
   - Configuration examples

2. **[imu_driver/README.md](src/imu_driver/README.md)** - IMU-specific details
   - BNO055 setup and calibration
   - I2C configuration
   - Coordinate frame explanation
   - Troubleshooting IMU issues

3. **[GPS_IMU_INTEGRATION.md](GPS_IMU_INTEGRATION.md)** - Integration guide
   - Combined setup instructions
   - Coordinate frame conversion examples
   - Data fusion examples
   - Parameter reference table

## 🛠️ Utilities

### setup_gps_imu.py
Automated setup utility that:
- Detects your platform (RPi, Jetson Nano, etc.)
- Finds available serial ports
- Scans I2C buses and finds BNO055
- Verifies library installations
- Provides platform-specific recommendations

Run with:
```bash
python3 /workspace/setup_gps_imu.py
```

## 📡 Integration with Your Rover

To integrate into your existing rover stack:

1. **Update main rover launch file:**
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        '/workspace/src/gps_driver/launch/gps_imu.launch.py'
    ),
    launch_arguments={
        'gps_port': '/dev/ttyUSB0',
        'i2c_bus': '1',
        'i2c_address': '0x28',
    }.items(),
)
```

2. **Subscribe to topics in your navigation node:**
```python
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
self.create_subscription(Float32, '/imu/heading', self.heading_callback, 10)
```

## ⚙️ Platform-Specific Setup

### Raspberry Pi 4/4B
```bash
# Enable UART and I2C
sudo raspi-config
# Interfaces → Serial Port → Enable
# Interfaces → I2C → Enable
# Reboot

# Verify
i2cdetect -y 1
ls /dev/ttyS0  # UART
```

### Jetson Nano
```bash
# Check available I2C
ls /dev/i2c*

# Scan for devices
i2cdetect -y 1  # or i2cdetect -y 0

# Check serial ports
ls /dev/ttyTHS*
```

## 🐛 Troubleshooting

### GPS not getting fix
- Move antenna outdoors (clear sky needed)
- Wait 30-60 seconds for initial acquisition
- Check antenna connection
- Verify serial connection with: `screen /dev/ttyUSB0 9600`

### IMU heading inaccurate
- Ensure full calibration (watch node output for status)
- Keep away from metal objects
- Rotate rover through full 360° range
- Recalibrate by power cycling

### Nodes won't start
```bash
# Check topic existence
ros2 topic list

# Check for errors
ros2 launch gps_driver gps.launch.py [params]

# Check hardware
python3 setup_gps_imu.py
```

## 📦 Dependencies

**System packages (Ubuntu/Debian):**
```bash
sudo apt-get install -y python3-pip i2c-tools
```

**Python packages:**
```bash
pip install pyserial adafruit-circuitpython-bno055 adafruit-circuitpython-busio
```

**ROS2 packages:**
- `rclpy` - ROS2 Python client library
- `sensor_msgs` - Standard sensor message types
- `geometry_msgs` - Standard geometry message types
- `std_msgs` - Standard message types

All automatically installed via package dependencies.

## 📝 Message Format Examples

### GPS Fix Message
```yaml
{latitude: 37.427404, longitude: -122.143020, altitude: 10.5}
position_covariance: [5.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 10.0]
status: 1  # 1 = fix, 0 = no fix
```

### IMU Heading Message
```yaml
data: 0.785  # π/4 radians ≈ 45° from East toward North
```

## 🔗 References

- [Adafruit Ultimate GPS Guide](https://learn.adafruit.com/adafruit-ultimate-gps)
- [Adafruit BNO055 Guide](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor)
- [ROS2 Sensor Messages](http://docs.ros.org/en/humble/p/sensor_msgs/interfaces/sensor_msgs/msg/)
- [ENU Coordinate Convention](https://en.wikipedia.org/wiki/Axes_conventions#Ground_vehicles)

## ✅ Verification Checklist

- [ ] GPS module receives serial data (`screen /dev/ttyUSB0 9600`)
- [ ] IMU found on I2C bus (`i2cdetect -y 1` shows 28 or 29)
- [ ] Dependencies installed (`pip list | grep adafruit`)
- [ ] Packages build without errors (`colcon build`)
- [ ] GPS node publishes `/gps/fix` (`ros2 topic echo /gps/fix`)
- [ ] IMU node publishes `/imu/heading` (`ros2 topic echo /imu/heading`)
- [ ] Both nodes running together without conflicts
- [ ] Heading ranges from -π to π (not 0-2π)
- [ ] GPS gets fix after moving outdoors (takes 30-60 sec)
- [ ] IMU calibration reaches System=3/3 status

---

**Ready to test!** Follow the "Quick Start" section above to get your GPS and IMU nodes running.
