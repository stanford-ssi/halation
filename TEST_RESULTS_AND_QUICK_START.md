# GPS & IMU Drivers - Test Results & Quick Launch Guide

## ✅ Test Results: PASSED

```
GPS Driver:       5/5 tests passed ✓
IMU Driver:       5/5 tests passed ✓
Documentation:    7/7 files present ✓
─────────────────────────────────────
Total:            17/17 ✓
```

All packages are built, validated, and ready to use!

## 🚀 Launch Commands

### Option 1: Both GPS and IMU Together (Recommended)
```bash
source /workspace/install/setup.bash
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyUSB0 i2c_bus:=1 i2c_address:=0x28
```

### Option 2: GPS Only
```bash
source /workspace/install/setup.bash
ros2 launch gps_driver gps.launch.py port:=/dev/ttyUSB0
```

### Option 3: IMU Only
```bash
source /workspace/install/setup.bash
ros2 launch imu_driver imu.launch.py i2c_bus:=1 i2c_address:=0x28
```

## 📊 Monitor Data (in new terminals)

### GPS Position
```bash
ros2 topic echo /gps/fix
```
Expected output:
```
latitude: 37.4274
longitude: -122.1430
altitude: 10.5
status: 1  (1 = has fix)
```

### GPS Velocity
```bash
ros2 topic echo /gps/vel
```

### IMU Heading (radians, 0=East, CCW positive)
```bash
ros2 topic echo /imu/heading
```
Expected values:
```
data: 0.0     # Facing East
data: 1.57    # Facing North (π/2)
data: 3.14    # Facing West (π)
data: -1.57   # Facing South (-π/2)
```

### IMU Raw Data
```bash
ros2 topic echo /imu/data
```

## 🔌 Serial Port Configuration

Find your GPS serial port:
```bash
ls /dev/tty*
```

Common options:
- `/dev/ttyUSB0` - USB adapter (most common)
- `/dev/ttyS0` - Raspberry Pi built-in UART
- `/dev/ttyTHS1` - Jetson Nano UART

Update the launch command with your actual port:
```bash
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyUSB0
```

## 🎯 I2C Bus Configuration

Find your IMU I2C bus:
```bash
i2cdetect -y 1
# Look for address 28 or 29
```

If not found, try different buses:
```bash
i2cdetect -y 0
i2cdetect -y 2
```

Update the launch command with your bus:
```bash
ros2 launch gps_driver gps_imu.launch.py i2c_bus:=1 i2c_address:=0x28
```

## ✅ What to Check

- [ ] **GPS gets fix**: Watch `/gps/fix` - status should change from 0 to 1
  - Move outdoors with clear sky view
  - Wait 30-60 seconds for initial fix
  
- [ ] **IMU samples**: Watch `/imu/heading` - should update at ~100 Hz

- [ ] **Data quality**: 
  - GPS accuracy: ±3-10 meters typical
  - IMU heading: Should be smooth, not jittery
  - Calibration status in IMU node: Should reach System=3/3

## 🔧 Troubleshooting

### "Port not found" error
```bash
# Check available ports
ls /dev/tty*
# Update command with correct port
```

### "I2C device not found" error
```bash
# Enable I2C on Raspberry Pi
sudo raspi-config
# Go to: Interfaces → I2C → Enable
# Then reboot

# Verify I2C is working
i2cdetect -y 1
# Should show 28 or 29
```

### "No GPS fix after 2 minutes"
- Move antenna outdoors with clear sky view
- Check antenna connection
- Verify LED on GPS module is blinking

### "IMU heading seems wrong"
- Ensure calibration is complete (System=3/3)
- Keep away from metal objects
- Rotate rover through full 360° to recalibrate

## 📚 More Information

- **Quick Reference**: [GPS_IMU_QUICK_REFERENCE.md](GPS_IMU_QUICK_REFERENCE.md)
- **Full Setup Guide**: [GPS_IMU_SETUP_SUMMARY.md](GPS_IMU_SETUP_SUMMARY.md)
- **GPS Details**: [src/gps_driver/README.md](src/gps_driver/README.md)
- **IMU Details**: [src/imu_driver/README.md](src/imu_driver/README.md)
- **Integration Guide**: [GPS_IMU_INTEGRATION.md](GPS_IMU_INTEGRATION.md)
- **Full Testing Checklist**: [VERIFICATION_CHECKLIST.md](VERIFICATION_CHECKLIST.md)

## 🎓 Example Code

Subscribe to GPS and IMU in your node:

```python
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import math

class MyRoverNode(Node):
    def __init__(self):
        super().__init__('my_rover')
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Float32, '/imu/heading', self.heading_callback, 10)
    
    def gps_callback(self, msg):
        if msg.status.status == 1:  # Has fix
            print(f"Position: {msg.latitude:.6f}, {msg.longitude:.6f}")
            print(f"Altitude: {msg.altitude}m")
    
    def heading_callback(self, msg):
        # Convert to degrees
        heading_deg = math.degrees(msg.data)
        print(f"Heading: {heading_deg:.1f}° (ENU: 0°=East, CCW+)")
```

See [example_gps_imu_fusion.py](example_gps_imu_fusion.py) for complete example.

---

**Status**: ✅ Ready to use  
**Test Date**: March 2, 2026  
**Packages Built**: gps_driver, imu_driver
