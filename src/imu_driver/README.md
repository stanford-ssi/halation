# IMU Driver README

## Overview
ROS2 driver for the **Adafruit 9-DOF Absolute Orientation IMU** (Adafruit #4646, BNO055 chipset).

### Features
- Reads absolute orientation from BNO055 sensor
- Publishes heading in radians (0=East, CCW positive, ENU convention)
- Publishes raw IMU data (accelerometer, gyroscope)
- On-board sensor fusion (no external algorithms needed)
- Calibration status monitoring
- Configurable I2C bus and address

## Hardware

### Adafruit BNO055 9-DOF Absolute Orientation IMU (Product #4646)
- **Communication**: I2C (address 0x28 or 0x29)
- **Sensors**: Accelerometer, Gyroscope, Magnetometer
- **Update Rate**: 100 Hz
- **Supply Voltage**: 3.3-5V DC
- **Interface**: STEMMA QT / Qwiic connectors (or header pins)

### Available Measurements
- Absolute orientation (Euler angles, Quaternions)
- Angular velocity (rad/s)
- Linear acceleration (m/s²)
- Magnetic field (µT)
- Temperature (°C)

### Wiring

#### For Raspberry Pi (I2C-1 at GPIO 2 & 3):
```
BNO055        Raspberry Pi
VIN      ←→  5V (Pin 2 or 4)
GND      ←→  GND (Pin 6, 9, 14, 20, 25, 30, 34, 39)
SDA      ←→  GPIO 2 (SDA) / Pin 3
SCL      ←→  GPIO 3 (SCL) / Pin 5
```

#### For Jetson Nano (I2C-1):
```
BNO055        Jetson Nano
VIN      ←→  5V (Pin 2)
GND      ←→  GND (Pin 6, 9, 14, 25, 30, 34, 39)
SDA      ←→  GPIO 2 (I2C1_SDA) / Pin 27
SCL      ←→  GPIO 3 (I2C1_SCL) / Pin 28
```

**Note**: Some Jetson Nano models may use I2C-0 instead. Check with `ls /dev/i2c*`.

## Setup

### 1. Enable I2C on Raspberry Pi
```bash
# Using raspi-config
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
```

Or manually:
```bash
# Edit /boot/config.txt
sudo nano /boot/config.txt
# Uncomment: dtparam=i2c_arm=on
```

Then reboot:
```bash
sudo reboot
```

### 2. Enable I2C on Jetson Nano
I2C is usually enabled by default. Verify:
```bash
ls /dev/i2c*
```

If not available, edit `/boot/dtb/tegra210-p3448-0000.dtb` (requires DTB compilation).

### 3. Verify I2C Connection

Install I2C tools:
```bash
sudo apt-get update
sudo apt-get install -y i2c-tools
```

Scan I2C buses:
```bash
# For Raspberry Pi (bus 1)
i2cdetect -y 1

# For Jetson Nano (try both)
i2cdetect -y 0
i2cdetect -y 1
```

You should see `28` or `29` (BNO055's address) in the output.

### 4. Install Python Dependencies

```bash
# Install CircuitPython library for BNO055
pip install adafruit-circuitpython-bno055

# Install board and busio libraries
pip install adafruit-circuitpython-busio
```

For Jetson Nano, you may also need:
```bash
pip install Adafruit_PureIO
```

### 5. Build and Run

Build the package:
```bash
cd /workspace
colcon build --packages-select imu_driver
```

Run the IMU node:
```bash
# Using launch file
ros2 launch imu_driver imu.launch.py i2c_bus:=1 i2c_address:=0x28

# Or run directly
ros2 run imu_driver imu_node --ros-args -p i2c_bus:=1 -p i2c_address:=0x28
```

## Published Topics

### /imu/heading (std_msgs/Float32)
Heading angle in radians

**Frame Convention:**
- **Standard**: 0 radians = East direction
- **Rotation**: Counter-clockwise (CCW) positive
- **Frame**: ENU (East-North-Up)
- **Range**: [-π, π] radians or 0 to 2π

**Conversion from Compass Heading:**
If you have geographic heading (0°=North, clockwise positive):
```
heading_enu = π/2 - heading_geographic
```

**Example Values:**
- 0 rad: East
- π/2 rad (1.57): North
- π rad (3.14): West  
- -π/2 rad (-1.57): South

### /imu/data (sensor_msgs/Imu)
Full IMU sensor data

**Message Fields:**
- `header.frame_id`: "imu"
- `header.stamp`: Timestamp
- `linear_acceleration`: x=East, y=North, z=Up (m/s²)
- `angular_velocity`: x=East, y=North, z=Up (rad/s)
- `orientation`: Not populated (use heading topic instead)

**Frame Convention (ENU):**
- X: East
- Y: North
- Z: Up

## Configuration

Create or edit `config/imu_params.yaml`:

```yaml
i2c_bus: 1              # I2C bus number (1 for RPi, check for Jetson)
i2c_address: 0x28       # BNO055 I2C address
frame_id: "imu"         # TF frame ID
publish_rate: 100.0     # Publishing rate in Hz
```

Pass parameters at runtime:
```bash
ros2 run imu_driver imu_node --ros-args \
  -p i2c_bus:=1 \
  -p i2c_address:=0x28 \
  -p frame_id:=imu \
  -p publish_rate:=100
```

## IMU Calibration

The BNO055 includes on-board sensor fusion, but it needs to be calibrated for best results.

### Calibration Status
The driver logs calibration status every 10 seconds:
```
[INFO] [imu_driver-1]: Calibration: System=3/3, Gyro=3/3, Accel=3/3, Mag=3/3
```

Status scale (0-3):
- **0**: Not calibrated
- **1**: Calibration in progress
- **2**: Calibration in progress (better)
- **3**: Fully calibrated

### Calibration Procedure

1. **Accelerometer Calibration** (place on flat surface):
   - Place on level surface for ~2 seconds
   - System learns gravity vector

2. **Magnetic Calibration** (remove metal):
   - Move sensor in a figure-8 pattern in the air
   - Helps magnetometer map local magnetic field
   - Remove any metal objects nearby

3. **Gyroscope Calibration** (keep still):
   - Keep sensor completely still for ~5 seconds
   - System measures gyro bias

**Best Practice**: Calibrate outdoors and away from metal objects for most accurate heading data.

## Troubleshooting

### I2C Device Not Found
```bash
# Check I2C bus
i2cdetect -y 1

# If address 0x28 not shown:
# - Check wiring (SDA/SCL correct pins)
# - Check voltage (3.3V or 5V)
# - Try address 0x29 (if addr pin is high)
# - Check I2C enabled in raspi-config or /boot/config.txt
```

### Permission Denied Error
```bash
sudo usermod -a -G i2c $USER
# Log out and back in, or reboot
```

### Inaccurate Heading
- Ensure full calibration (System=3/3)
- Keep away from metal objects and strong magnetic fields
- Rotate sensor through full range during operation
- Heading accuracy improves over time as sensor self-calibrates

### Sensor Not Responding
- Verify I2C address: `i2cdetect -y 1`
- Check power supply (3.3-5V stable)
- Try resetting sensor: power cycle for 10 seconds
- Check for loose connections (especially STEMMA QT connector)

## References
- [Adafruit BNO055 Guide](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor)
- [BNO055 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
- [CircuitPython Library](https://github.com/adafruit/Adafruit_CircuitPython_BNO055)
- [ROS2 Imu Message](http://docs.ros.org/en/humble/p/sensor_msgs/interfaces/sensor_msgs/msg/Imu.html)
