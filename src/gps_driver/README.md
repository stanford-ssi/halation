# GPS Driver README

## Overview
ROS2 driver for the **Adafruit Ultimate GPS Breakout** (Adafruit #746, MTK3339/PA1616S chipset).

### Features
- Reads NMEA sentences from the GPS module via serial/UART
- Publishes GPS fix data (latitude, longitude, altitude)
- Publishes GPS velocity data from VTG sentences
- Configurable serial port and baudrate
- Covariance-aware position and velocity estimates

## Hardware

### Adafruit Ultimate GPS Breakout (Product #746)
- **Communication**: UART/Serial (default 9600 baud)
- **Data Format**: NMEA 0183
- **Update Rate**: Up to 10 Hz
- **Position Accuracy**: ~3 meters (standard GPS accuracy)
- **Velocity Accuracy**: 0.1 m/s
- **Supply Voltage**: 3.0-5.5V DC

### Wiring

#### For Raspberry Pi:
```
GPS Module    Raspberry Pi
VIN      ←→  5V (Pin 2 or 4)
GND      ←→  GND (Pin 6, 9, 14, 20, 25, 30, 34, 39)
TX       ←→  GPIO 10 (RXD on UART0) 
RX       ←→  GPIO 8 (TXD on UART0)
```

#### For Jetson Nano:
```
GPS Module    Jetson Nano
VIN      ←→  5V (Pin 2)
GND      ←→  GND (Pin 6, 9, 14, 25, 30, 34, 39)
TX       ←→  GPIO 8 (UART1 RX)
RX       ←→  GPIO 10 (UART1 TX)
```

Or use USB-to-UART adapter connected to any USB port.

## Setup

### 1. Enable UART on Raspberry Pi
```bash
# Edit /boot/config.txt
sudo nano /boot/config.txt

# Add these lines:
enable_uart=1
dtoverlay=disable-bt
```

Then reboot:
```bash
sudo reboot
```

### 2. Enable I2C/Serial on Jetson Nano
The Jetson Nano has UART1 enabled by default. No additional configuration needed.

### 3. Verify GPS Connection

Identify the serial port:
```bash
# List all serial ports
ls -la /dev/ttyUSB*  # For USB adapter
ls -la /dev/ttyS*   # For built-in UART
```

Test the connection with minicom or screen:
```bash
# Using screen
screen /dev/ttyUSB0 9600

# Or using minicom
minicom -D /dev/ttyUSB0 -b 9600
```

You should see NMEA sentences like `$GPGGA...` and `$GPVTG...`.

### 4. Build and Run

Build the package:
```bash
cd /workspace
colcon build --packages-select gps_driver
```

Run the GPS node:
```bash
# Using launch file
ros2 launch gps_driver gps.launch.py port:=/dev/ttyUSB0

# Or run directly
ros2 run gps_driver gps_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=9600
```

## Published Topics

### /gps/fix (sensor_msgs/NavSatFix)
GPS position fix with covariance

**Message Fields:**
- `header.frame_id`: "gps"
- `header.stamp`: Timestamp
- `latitude`: Latitude in degrees
- `longitude`: Longitude in degrees
- `altitude`: Altitude in meters
- `position_covariance`: 3x3 covariance matrix (lat, lon, alt)
- `status.status`: StatusFix (0=no fix, 1=fix)

**Example:**
```
latitude: 37.4274
longitude: -122.1430
altitude: 10.5
```

### /gps/vel (geometry_msgs/TwistWithCovarianceStamped)
GPS velocity in ENU frame

**Message Fields:**
- `header.frame_id`: "gps"
- `header.stamp`: Timestamp
- `twist.twist.linear.x`: Velocity East (m/s)
- `twist.twist.linear.y`: Velocity North (m/s)
- `twist.twist.linear.z`: Always 0.0
- `twist.covariance`: 6x6 covariance matrix

**Frame Convention:**
- X (linear.x): East
- Y (linear.y): North
- Z (linear.z): Up (always 0)

## Configuration

Create or edit `config/gps_params.yaml`:

```yaml
port: "/dev/ttyUSB0"    # Serial port
baudrate: 9600          # Baud rate
frame_id: "gps"         # TF frame ID
```

Pass parameters at runtime:
```bash
ros2 run gps_driver gps_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=9600 \
  -p frame_id:=gps
```

## Troubleshooting

### No GPS fix
- Move antenna outdoors (GPS requires clear sky view)
- Wait 30-60 seconds for initial fix
- Check antenna connection
- Try cold start: power cycle the module and wait

### Serial port not found
- Check with `ls /dev/tty*`
- May need to install USB-to-UART driver
- Check device permissions: `sudo usermod -a -G dialout $USER`

### Erratic position data
- GPS accuracy is typically ±3 meters
- Use multiple position samples for better accuracy
- Enable WAAS/EGNOS if available in your region

## References
- [Adafruit Ultimate GPS Guide](https://learn.adafruit.com/adafruit-ultimate-gps)
- [MT3339 Datasheet](https://cdn-shop.adafruit.com/product-files/746/CD+PA1616S+Datasheet.v03.pdf)
- [ROS2 NavSatFix Message](http://docs.ros.org/en/humble/p/sensor_msgs/interfaces/sensor_msgs/msg/NavSatFix.html)
