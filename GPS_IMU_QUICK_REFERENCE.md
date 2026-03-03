# GPS and IMU Quick Reference

## Start Your Nodes

### Option 1: Both together (easiest)
```bash
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyUSB0 i2c_bus:=1 i2c_address:=0x28
```

### Option 2: Separately in different terminals
```bash
# Terminal 1 - GPS
ros2 launch gps_driver gps.launch.py port:=/dev/ttyUSB0

# Terminal 2 - IMU  
ros2 launch imu_driver imu.launch.py i2c_bus:=1 i2c_address:=0x28
```

### Option 3: Manual (for testing)
```bash
# Terminal 1 - GPS
ros2 run gps_driver gps_node --ros-args -p port:=/dev/ttyUSB0

# Terminal 2 - IMU
ros2 run imu_driver imu_node --ros-args -p i2c_bus:=1 -p i2c_address:=0x28
```

## Check Your Data

### GPS Position
```bash
ros2 topic echo /gps/fix
```
Look for:
- `latitude`, `longitude` in degrees
- `altitude` in meters
- `status: 1` means GPS has a fix

### GPS Velocity
```bash
ros2 topic echo /gps/vel
```
Velocity in East-North-Up frame (m/s)

### IMU Heading
```bash
ros2 topic echo /imu/heading
```
Value in **radians**:
- `0.0` = East
- `1.57` = North (π/2)
- `3.14` = West (π)
- `-1.57` = South (-π/2)

### Full IMU Data
```bash
ros2 topic echo /imu/data
```
Accelerometer and gyroscope in ENU frame

## Find Your Hardware Ports

### Detect everything
```bash
python3 /workspace/setup_gps_imu.py
```

### Manual detection
```bash
# GPS serial ports
ls /dev/tty*

# IMU I2C buses
i2cdetect -y 1
# Look for 28 or 29 (BNO055 address)
```

## Common Configurations

### Raspberry Pi with USB GPS
```bash
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyUSB0 i2c_bus:=1 i2c_address:=0x28
```

### Raspberry Pi with built-in UART
```bash
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyS0 i2c_bus:=1 i2c_address:=0x28
```

### Jetson Nano with USB GPS
```bash
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyUSB0 i2c_bus:=1 i2c_address:=0x28
```

## Using Data in Your Code

### Subscribe to GPS
```python
from sensor_msgs.msg import NavSatFix

def gps_callback(self, msg):
    print(f"Position: {msg.latitude}, {msg.longitude}")
    print(f"Altitude: {msg.altitude}m")
    print(f"Has fix: {msg.status.status == 1}")

self.create_subscription(NavSatFix, '/gps/fix', gps_callback, 10)
```

### Subscribe to Heading
```python
from std_msgs.msg import Float32
import math

def heading_callback(self, msg):
    heading_rad = msg.data
    heading_deg = math.degrees(heading_rad)
    print(f"Heading: {heading_deg:.1f}° (ENU: 0°=East, CCW positive)")

self.create_subscription(Float32, '/imu/heading', heading_callback, 10)
```

## Fix Common Issues

### "Port not found"
```bash
# Check actual port
ls /dev/tty*
# Use the correct port in launch command
```

### "I2C device not found"
```bash
# Verify I2C is enabled on Raspberry Pi
sudo raspi-config  # → Interfaces → I2C → Enable

# Check bus
i2cdetect -y 1

# Check address (should see 28 or 29)
```

### "GPS no fix"
- Move **outside** with clear view of sky
- Wait 30-60 seconds
- Check antenna is connected properly

### "IMU heading drifts"
- Ensure calibration status is 3/3 (watch node output)
- Keep away from metal objects
- Rotate rover through full 360° to help calibration

## Coordinate Frames

### Your Rover (ENU convention)
```
        North (+Y)
          ▲
          |
West  ◀───+───▶ East  
(-X)  180°|  0° (+X)
          |
          ▼
        South
       270° or -90°
```

**Heading values** from `/imu/heading`:
- **0 rad** → Facing East
- **π/2 rad** → Facing North  
- **π rad** → Facing West
- **-π/2 rad** → Facing South

### GPS Velocities in `/gps/vel`
- `linear.x` → Motion East (positive = moving east)
- `linear.y` → Motion North (positive = moving north)
- `linear.z` → Always 0 (ground vehicle)

## Build Everything

```bash
cd /workspace
colcon build --packages-select gps_driver imu_driver
source install/setup.bash
```

## Performance

### GPS
- Updates: 1-10 Hz (configurable on module)
- Accuracy: ±3 meters typical
- Fix time: 30-60 seconds (cold start)

### IMU
- Updates: 100 Hz (configurable)
- Heading accuracy: ±5° (depends on calibration)
- Calibration: Full (System=3/3) recommended

## For More Details

- **GPS Setup**: [src/gps_driver/README.md](src/gps_driver/README.md)
- **IMU Setup**: [src/imu_driver/README.md](src/imu_driver/README.md)
- **Integration Guide**: [GPS_IMU_INTEGRATION.md](GPS_IMU_INTEGRATION.md)
- **Full Summary**: [GPS_IMU_SETUP_SUMMARY.md](GPS_IMU_SETUP_SUMMARY.md)

---
**Pro tip**: Save this file as a reference while testing!
