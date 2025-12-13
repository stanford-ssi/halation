from rclpy.node import Node
from std_msgs.msg import String
import adafruit_dacx578
import board
import busio
import json
import rclpy

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

        # Subscribe to vector commands (x, y from joystick)
        self.vector_subscriber = self.create_subscription(
            String, "/motor_vector", self.handle_vector, 10
        )
        
        # Subscribe to emergency stop
        self.estop_subscriber = self.create_subscription(
            String, "/motor_estop", self.handle_estop, 10
        )

        # Stop all motors on startup
        self.stop_all()

    def set_motor(self, motor_name: str, speed: float):
        """Set a motor's speed and direction.
        
        Args:
            motor_name: Key from MOTORS dict
            speed: -1.0 to 1.0 (negative = backward, positive = forward)
        """
        motor = MOTORS[motor_name]
        
        # Apply inversion if needed
        actual_speed = speed if not motor["inverted"] else -speed
        
        # Determine direction
        forward = actual_speed >= 0
        
        # Set speed via DAC (absolute value)
        dac_value = int(abs(actual_speed) * VOLTAGE_MAX)
        dac_value = min(dac_value, VOLTAGE_MAX)  # Clamp to max
        self.dac.channels[motor["speed_channel"]].raw_value = dac_value
        
        # Set direction via DAC channel (VOLTAGE_MAX = forward, 0 = backward)
        direction_value = VOLTAGE_MAX if forward else 0
        self.dac.channels[motor["direction_channel"]].raw_value = direction_value
        
        self.get_logger().debug(f"{motor_name}: speed={dac_value}, forward={forward}")

    def stop_all(self):
        """Stop all motors immediately."""
        for motor_name in MOTORS:
            self.set_motor(motor_name, 0.0)
        self.get_logger().info("All motors stopped")

    def drive(self, x: float, y: float):
        """Drive using differential steering.
        
        Args:
            x: -1.0 to 1.0 (negative = left, positive = right)
            y: -1.0 to 1.0 (negative = backward, positive = forward)
        """
        # Differential drive calculation
        # y controls forward/backward, x controls turning
        left_speed = y + x
        right_speed = y - x
        
        # Clamp speeds to -1.0 to 1.0
        max_magnitude = max(abs(left_speed), abs(right_speed), 1.0)
        left_speed /= max_magnitude
        right_speed /= max_magnitude
        
        # Apply to motors
        self.set_motor("left_front", left_speed)
        self.set_motor("left_back", left_speed)
        self.set_motor("right_front", right_speed)
        self.set_motor("right_back", right_speed)
        
        self.get_logger().debug(f"Drive: x={x:.2f}, y={y:.2f} -> L={left_speed:.2f}, R={right_speed:.2f}")

    def handle_vector(self, msg: String):
        """Handle incoming vector commands from joystick."""
        try:
            outer_data = json.loads(msg.data)
            # Handle nested JSON from rosbridge
            inner_data = json.loads(outer_data.get("data", "{}"))
            x = float(inner_data.get("x", 0))
            y = float(inner_data.get("y", 0))
        except (json.JSONDecodeError, TypeError, ValueError) as e:
            self.get_logger().warn(f"Invalid vector data: {msg.data} - {e}")
            return
        
        # Clamp values to valid range
        x = max(-1.0, min(1.0, x))
        y = max(-1.0, min(1.0, y))
        
        self.get_logger().info(f"Vector: x={x:.2f}, y={y:.2f}")
        self.drive(x, y)

    def handle_estop(self, msg: String):
        """Handle emergency stop."""
        self.get_logger().warn("E-STOP ACTIVATED")
        self.stop_all()


def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorControl()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()
