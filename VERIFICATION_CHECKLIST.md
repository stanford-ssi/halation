# GPS & IMU Setup Verification Checklist

Use this checklist to verify your GPS and IMU drivers are properly configured and working.

## Prerequisites
- [ ] Raspberry Pi 4/4B or Jetson Nano with ROS2 installed
- [ ] Adafruit Ultimate GPS Breakout #746
- [ ] Adafruit BNO055 9-DOF IMU #4646
- [ ] USB-to-3.3V serial adapter OR established UART connection
- [ ] I2C connection verified with `i2cdetect`

## Step 1: Hardware Verification

### GPS Module
- [ ] GPS module powered (red LED on)
- [ ] GPS antenna connected
- [ ] Serial connection verified

**Test GPS serial connection:**
```bash
ls /dev/tty*  # Find your serial port
screen /dev/ttyUSB0 9600  # Or your actual port
# Should see sentences starting with $GPGGA, $GPVTG, etc.
# Press Ctrl+A then Ctrl+X to exit
```

### IMU Module
- [ ] IMU module powered
- [ ] I2C connection verified

**Test I2C connection:**
```bash
i2cdetect -y 1  # Or appropriate bus (0, 2, etc.)
# Should show 28 or 29 in the matrix
```

## Step 2: Software Installation

### Install Python dependencies
- [ ] `pip install pyserial` (for GPS)
- [ ] `pip install adafruit-circuitpython-bno055` (for IMU)
- [ ] `pip install adafruit-circuitpython-busio` (for IMU)

**Verify installation:**
```bash
python3 -c "import serial; print('pyserial OK')"
python3 -c "import adafruit_bno055; print('BNO055 OK')"
```

### Build ROS2 packages
```bash
cd /workspace
colcon build --packages-select gps_driver imu_driver
```
- [ ] GPS driver builds without errors
- [ ] IMU driver builds without errors
- [ ] No missing dependencies reported

## Step 3: Node Launch

### GPS Node
```bash
ros2 launch gps_driver gps.launch.py port:=/dev/ttyUSB0
```
- [ ] Node starts without errors
- [ ] No "port not found" messages
- [ ] See messages like "GPS Driver initialized on /dev/ttyUSB0"

### IMU Node
```bash
ros2 launch imu_driver imu.launch.py i2c_bus:=1 i2c_address:=0x28
```
- [ ] Node starts without errors
- [ ] No "I2C device not found" messages
- [ ] Calibration status prints every 10 seconds

### Combined Launch
```bash
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyUSB0 i2c_bus:=1 i2c_address:=0x28
```
- [ ] Both nodes start together
- [ ] No conflicts between nodes

## Step 4: Topic Verification

### GPS Topics
```bash
ros2 topic list | grep gps
```
- [ ] `/gps/fix` appears in list
- [ ] `/gps/vel` appears in list

**Check GPS Fix data:**
```bash
ros2 topic echo /gps/fix
```
- [ ] Messages arrive every 1-10 seconds
- [ ] `latitude` and `longitude` are non-zero
- [ ] `status.status` is 0 (no fix) or 1 (has fix)
- [ ] Position values reasonable for your location

**Check GPS Velocity data:**
```bash
ros2 topic echo /gps/vel
```
- [ ] Messages arrive (check timestamp changes)
- [ ] `linear.x` and `linear.y` are present
- [ ] Values are in reasonable range (-50 to +50 m/s for rover)

### IMU Topics
```bash
ros2 topic list | grep imu
```
- [ ] `/imu/heading` appears in list
- [ ] `/imu/data` appears in list

**Check IMU Heading data:**
```bash
ros2 topic echo /imu/heading
```
- [ ] Messages arrive at ~100 Hz (very frequent)
- [ ] `data` value is between -3.14 and +3.14 (radians)
- [ ] Value changes smoothly as you rotate rover

**Check IMU Raw data:**
```bash
ros2 topic echo /imu/data
```
- [ ] Messages arrive frequently
- [ ] `linear_acceleration` values change
- [ ] `angular_velocity` values change when rotating

## Step 5: GPS Fix Validation

### Get GPS Fix
- [ ] Move rover **outdoors** with clear view of sky
- [ ] Wait 30-60 seconds for initial fix
- [ ] Watch `/gps/fix` status change from 0 to 1

```bash
ros2 topic echo /gps/fix | grep -A5 status
```
Should eventually show:
```
status:
  status: 1
  service: 0
```

- [ ] Latitude/longitude change to your actual location
- [ ] Altitude shows reasonable value

**Pro tip:** If no fix after 1 minute:
- [ ] Check antenna connection
- [ ] Try different outdoor location
- [ ] Check GPS module LED (should blink slowly when locked)

## Step 6: IMU Calibration

Watch the node output:
```bash
ros2 launch imu_driver imu.launch.py i2c_bus:=1
```

Look for calibration status every 10 seconds:
```
[INFO] Calibration: System=X/3, Gyro=Y/3, Accel=Z/3, Mag=W/3
```

- [ ] System calibration reaches 3/3 (may take several minutes)
- [ ] Gyro is at 3/3 (usually fast)
- [ ] Accel is at 3/3 (place on flat surface)
- [ ] Mag reaches 3/3 (move in figure-8 pattern)

## Step 7: Data Quality

### GPS Accuracy
- [ ] Take GPS fix at known location
- [ ] Compare with Google Maps coordinates
- [ ] Difference should be within 3-10 meters

### IMU Heading Accuracy
- [ ] Note compass direction your rover faces
- [ ] Check `/imu/heading` value
- [ ] Rotate rover 90° and verify heading changes correctly
- [ ] Full 360° rotation should show heading cycling through -π to π

### Velocity Quality
- [ ] Have rover move in straight line
- [ ] `/gps/vel` should show movement in one direction
- [ ] Move east: `linear.x` increases
- [ ] Move north: `linear.y` increases

## Step 8: Integration Testing

### Both Running Together
```bash
ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyUSB0 i2c_bus:=1
```

In another terminal:
```bash
# Monitor combined data
ros2 topic echo /gps/fix -c 5  # Show 5 messages
ros2 topic echo /imu/heading -c 5
```

- [ ] Both topics publish simultaneously
- [ ] No message dropouts
- [ ] No mutual interference between drivers
- [ ] Timestamps are recent (not old data)

### Run Fusion Example
```bash
ros2 run example_gps_imu_fusion  # If added to setup.py
# Or run directly:
python3 /workspace/example_gps_imu_fusion.py
```

- [ ] Fusion node subscribes to both topics
- [ ] State updates appear every 2 seconds
- [ ] Position, heading, velocity all display correctly

## Step 9: Configuration

### Verify Parameters
```bash
ros2 param list /gps_node
ros2 param list /imu_node
```

- [ ] GPS parameters show port and baudrate
- [ ] IMU parameters show i2c_bus and address

### Set Parameters at Runtime
```bash
ros2 param set /gps_node baudrate 9600
ros2 param set /imu_node publish_rate 100
```

- [ ] Parameters update successfully
- [ ] Behavior changes accordingly

## Step 10: Logging & Debugging

### RQT Graph
```bash
ros2 run rqt_graph rqt_graph
```
- [ ] Shows `/gps_node` publishing to `/gps/fix` and `/gps/vel`
- [ ] Shows `/imu_node` publishing to `/imu/heading` and `/imu/data`
- [ ] Shows subscription nodes connected correctly

### Message Rates
```bash
ros2 topic hz /gps/fix
ros2 topic hz /imu/heading
```
- [ ] `/gps/fix`: 1-10 Hz
- [ ] `/imu/heading`: ~100 Hz

### CPU/Memory Usage
```bash
ros2 node info /gps_node
ros2 node info /imu_node
```
- [ ] No excessive CPU usage
- [ ] Memory stable

## Troubleshooting

If any checks fail, refer to:

1. **GPS Issues**: [src/gps_driver/README.md](src/gps_driver/README.md#troubleshooting)
2. **IMU Issues**: [src/imu_driver/README.md](src/imu_driver/README.md#troubleshooting)
3. **Integration Issues**: [GPS_IMU_INTEGRATION.md](GPS_IMU_INTEGRATION.md#troubleshooting-integration)

## Final Verification

- [ ] All hardware detected correctly
- [ ] Both drivers build and launch successfully
- [ ] GPS provides position with fix confirmation
- [ ] IMU provides calibrated heading
- [ ] Data appears on correct topics at correct rates
- [ ] No error messages in node output
- [ ] Nodes can run simultaneously without conflicts
- [ ] Integration example runs and shows fused state

## Performance Summary

Expected performance on Raspberry Pi 4 / Jetson Nano:

| Metric | Expected | Your Result |
|--------|----------|-------------|
| GPS Fix Time (cold start) | 30-60 sec | _____ |
| GPS Position Accuracy | ±3-10 m | _____ |
| GPS Update Rate | 1-10 Hz | _____ |
| IMU Heading Accuracy | ±5° (calibrated) | _____ |
| IMU Update Rate | ~100 Hz | _____ |
| CPU Usage (GPS) | <5% | _____ |
| CPU Usage (IMU) | <5% | _____ |
| Memory (both nodes) | <100 MB | _____ |

## Ready for Integration

Once all checks pass, you're ready to:
1. Add nodes to your rover's main launch file
2. Integrate with your navigation stack
3. Use GPS/IMU data in your autonomous routines

---
**Completed**: ___________  
**By**: ___________
