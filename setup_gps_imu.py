#!/usr/bin/env python3
"""
Setup utility for GPS and IMU drivers.
Helps identify hardware, check connections, and configure parameters.
"""

import os
import subprocess
import sys

def run_command(cmd):
    """Run a shell command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout.strip(), result.returncode
    except Exception as e:
        return str(e), 1

def check_platform():
    """Detect the platform (RPi, Jetson Nano, etc.)"""
    output, _ = run_command("cat /proc/device-tree/model 2>/dev/null")
    
    if "Raspberry Pi" in output:
        return "Raspberry Pi", output
    elif "Jetson" in output:
        return "Jetson Nano", output
    else:
        return "Unknown", output

def check_serial_ports():
    """Find available serial ports"""
    ports = []
    
    # Check both /dev/ttyUSB* and /dev/ttyS*
    for cmd in ["ls /dev/ttyUSB* 2>/dev/null", "ls /dev/ttyS* 2>/dev/null", "ls /dev/ttyTHS* 2>/dev/null"]:
        output, _ = run_command(cmd)
        if output:
            ports.extend(output.split())
    
    return ports

def check_i2c_buses():
    """Find available I2C buses and devices"""
    buses = {}
    
    # Check available i2c buses
    output, code = run_command("ls /dev/i2c* 2>/dev/null")
    if code == 0 and output:
        for bus in output.split():
            bus_num = bus.split('-')[1]
            # Try to scan the bus
            scan_output, scan_code = run_command(f"i2cdetect -y {bus_num} 2>/dev/null")
            if scan_code == 0:
                buses[bus] = scan_output
            else:
                buses[bus] = "Cannot scan (i2c-tools not installed or permissions issue)"
    
    return buses

def check_uart_enabled():
    """Check if UART is enabled on Raspberry Pi"""
    config, _ = run_command("cat /boot/config.txt 2>/dev/null | grep -v '^#'")
    return "enable_uart=1" in config

def check_i2c_enabled():
    """Check if I2C is enabled on Raspberry Pi"""
    config, _ = run_command("cat /boot/config.txt 2>/dev/null | grep -v '^#'")
    return "i2c_arm=on" in config

def check_bno055_libraries():
    """Check if required libraries are installed"""
    try:
        import adafruit_bno055
        return True, "Installed"
    except ImportError:
        return False, "Not installed. Run: pip install adafruit-circuitpython-bno055"

def check_pyserial():
    """Check if pyserial is installed"""
    try:
        import serial
        return True, "Installed"
    except ImportError:
        return False, "Not installed. Run: pip install pyserial"

def print_header(text):
    """Print a formatted header"""
    print("\n" + "=" * 60)
    print(f"  {text}")
    print("=" * 60)

def main():
    print("\n" + "  GPS and IMU Driver Setup Utility".center(60, "="))
    print("=" * 60)
    
    # Platform detection
    print_header("Platform Detection")
    platform, model = check_platform()
    print(f"Detected Platform: {platform}")
    if model:
        print(f"Model: {model}")
    
    # Serial port detection
    print_header("Serial Ports (for GPS)")
    ports = check_serial_ports()
    if ports:
        print("Available serial ports:")
        for port in ports:
            print(f"  • {port}")
        print("\nConfigure GPS with one of these ports in your launch file.")
    else:
        print("❌ No serial ports found!")
        print("   For GPS via USB adapter: Connect adapter and check again")
        print("   For GPS via UART: Enable UART and reboot")
        if platform == "Raspberry Pi":
            print("   Command: sudo raspi-config → Interfaces → Serial Port → Enable")
    
    # I2C detection
    print_header("I2C Buses (for IMU)")
    buses = check_i2c_buses()
    
    if buses:
        for bus, devices in buses.items():
            print(f"\n{bus}:")
            print(devices if devices.startswith("     0") else f"  Error: {devices}")
            
            # Check for BNO055
            if "28" in str(devices) or "29" in str(devices):
                addr = "0x28" if "28" in str(devices) else "0x29"
                print(f"  ✓ BNO055 found at address {addr}")
            else:
                print("  ❌ BNO055 not found!")
    else:
        print("❌ No I2C buses found!")
        if platform == "Raspberry Pi":
            print("   Enable I2C: sudo raspi-config → Interfaces → I2C → Enable")
        print("   Check connections and reboot")
    
    # Installation check
    print_header("Required Libraries")
    
    has_bno055, bno055_msg = check_bno055_libraries()
    print(f"[{'✓' if has_bno055 else '✗'}] adafruit-circuitpython-bno055: {bno055_msg}")
    
    has_serial, serial_msg = check_pyserial()
    print(f"[{'✓' if has_serial else '✗'}] pyserial: {serial_msg}")
    
    # Platform-specific checks
    print_header("Platform-Specific Configuration")
    
    if platform == "Raspberry Pi":
        uart_enabled = check_uart_enabled()
        i2c_enabled = check_i2c_enabled()
        
        print(f"[{'✓' if uart_enabled else '✗'}] UART enabled in /boot/config.txt")
        print(f"[{'✓' if i2c_enabled else '✗'}] I2C enabled in /boot/config.txt")
        
        if not (uart_enabled and i2c_enabled):
            print("\n⚠️  Some interfaces are not enabled!")
            print("Run: sudo raspi-config")
            print("  → Interfaces → Serial Port → Enable")
            print("  → Interfaces → I2C → Enable")
            print("Then reboot: sudo reboot")
    
    elif platform == "Jetson Nano":
        print("✓ Jetson Nano detected")
        print("  UART and I2C are typically enabled by default")
        print("  Verify with: ls /dev/i2c* and ls /dev/tty*")
    
    # Build instructions
    print_header("Next Steps")
    print("""
1. Verify hardware connections:
   - GPS: Connected to identified serial port
   - IMU: Connected to identified I2C bus
   - Both: Power and ground connected

2. Build the packages:
   cd /workspace
   colcon build --packages-select gps_driver imu_driver
   source install/setup.bash

3. Update launch file with your port/bus numbers from above

4. Run the nodes:
   # GPS only
   ros2 launch gps_driver gps.launch.py port:=/dev/ttyUSB0
   
   # IMU only
   ros2 launch imu_driver imu.launch.py i2c_bus:=1 i2c_address:=0x28
   
   # Both together
   ros2 launch gps_driver gps_imu.launch.py gps_port:=/dev/ttyUSB0 i2c_bus:=1

5. Test topics:
   ros2 topic echo /gps/fix
   ros2 topic echo /imu/heading
""")
    
    print("=" * 60 + "\n")

if __name__ == "__main__":
    main()
