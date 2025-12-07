from rclpy.node import Node
from std_msgs.msg import String
import adafruit_dacx578
import board
import busio
import json
import rclpy

# Constant speed setting (0.0 to 1.0)
DRIVE_SPEED = 0.5

VOLTAGE_MAX = 4095  # DAC max value for speed

I2C_ADDRESS = 0x4c

# Motor channel configuration
# Left side: motors 1 & 3 (channels 0,1 and 4,5)
# Right side: motors 2 & 4 (channels 2,3 and 6,7)
MOTORS = {
    "left_front": {"speed_channel": 0, "direction_channel": 1, "inverted": False},
    "right_front": {"speed_channel": 2, "direction_channel": 3, "inverted": True},
    "left_back": {"speed_channel": 4, "direction_channel": 5, "inverted": False},
    "right_back": {"speed_channel": 6, "direction_channel": 7, "inverted": True},
}

i2c = busio.I2C(board.SCL, board.SDA)


class MotorControl(Node):
    def __init__(self):
        super().__init__("motor_control")

        self.dac = adafruit_dacx578.DACx578(i2c, address=I2C_ADDRESS)
        self.get_logger().info(f"Motor control initialized with I2C address {I2C_ADDRESS}")

        self.command_subscriber = self.create_subscription(
            String, "/motor_command", self.handle_command, 10
        )

        # Stop all motors on startup
        self.stop_all()

    def set_motor(self, motor_name: str, speed: float, forward: bool):
        """Set a motor's speed and direction.
        
        Args:
            motor_name: Key from MOTORS dict
            speed: 0.0 to 1.0
            forward: True for forward, False for backward
        """
        motor = MOTORS[motor_name]
        
        # Apply inversion if needed
        actual_forward = forward if not motor["inverted"] else not forward
        
        # Set speed via DAC
        dac_value = int(speed * VOLTAGE_MAX)
        self.dac.channels[motor["speed_channel"]].raw_value = dac_value
        
        # Set direction via DAC channel (4096 = high/forward, 0 = low/backward)
        direction_value = 4096 if actual_forward else 0
        self.dac.channels[motor["direction_channel"]].raw_value = direction_value
        
        self.get_logger().debug(f"{motor_name}: speed={dac_value}, forward={actual_forward}")

    def stop_all(self):
        """Stop all motors."""
        for motor_name in MOTORS:
            self.set_motor(motor_name, 0.0, True)
        self.get_logger().info("All motors stopped")

    def drive_forward(self):
        """Drive all motors forward."""
        for motor_name in MOTORS:
            self.set_motor(motor_name, DRIVE_SPEED, True)
        self.get_logger().info("Driving forward")

    def drive_backward(self):
        """Drive all motors backward."""
        for motor_name in MOTORS:
            self.set_motor(motor_name, DRIVE_SPEED, False)
        self.get_logger().info("Driving backward")

    def turn_left(self):
        """Turn left: right motors forward, left motors backward."""
        self.set_motor("left_front", DRIVE_SPEED, False)
        self.set_motor("left_back", DRIVE_SPEED, False)
        self.set_motor("right_front", DRIVE_SPEED, True)
        self.set_motor("right_back", DRIVE_SPEED, True)
        self.get_logger().info("Turning left")

    def turn_right(self):
        """Turn right: left motors forward, right motors backward."""
        self.set_motor("left_front", DRIVE_SPEED, True)
        self.set_motor("left_back", DRIVE_SPEED, True)
        self.set_motor("right_front", DRIVE_SPEED, False)
        self.set_motor("right_back", DRIVE_SPEED, False)
        self.get_logger().info("Turning right")

    def handle_command(self, msg: String):
        """Handle incoming motor commands."""
        try:
            data = json.loads(msg.data)
            command = data.get("data", "").lower()
        except json.JSONDecodeError:
            return self.get_logger().warn(f"Invalid JSON: {msg.data}")
        
        self.get_logger().info(f"Received command: {command}")

        if command == "forwards":
            self.drive_forward()
        elif command == "backwards":
            self.drive_backward()
        elif command == "left":
            self.turn_left()
        elif command == "right":
            self.turn_right()
        elif command == "stop":
            self.stop_all()
        else:
            self.get_logger().warn(f"Unknown command: {command}")


def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorControl()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()
