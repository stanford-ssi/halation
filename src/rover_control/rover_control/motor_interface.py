from rclpy.node import Node
from std_msgs.msg import Float32, String
import json
import rclpy


class MotorInterface(Node):
    def __init__(self):
        super().__init__("motor_interface")
        self.target_voltage = 0

        self.get_logger().info("Motor interface initialized")

        self.voltage_publisher = self.create_publisher(
            Float32, "target_voltage", 10)
        self.command_subscriber = self.create_subscription(
            String, "motor_command", self.handle_motor_command, 10
        )

    def handle_motor_command(self, msg):
        try:
            command = json.loads(msg.data)["command"]
            self.get_logger().info(f"Received motor command: {command}")

            if command == "forwards":
                self.set_voltage(1.0)
            elif command == "backwards":
                self.set_voltage(-1.0)
            elif command == "stop":
                self.set_voltage(0.0)
            else:
                self.get_logger().warn(f"Unknown motor command: {command}")
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid motor command: {msg.data}")

    def set_voltage(self, voltage):
        self.target_voltage = voltage
        self.get_logger().info(f"Target voltage set to {voltage}")
        self.voltage_publisher.publish(Float32(data=self.target_voltage))


def main(args=None):
    rclpy.init(args=args)
    motor_interface = MotorInterface()
    rclpy.spin(motor_interface)
    motor_interface.destroy_node()
    rclpy.shutdown()
