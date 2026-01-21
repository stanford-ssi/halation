import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RoverBringupNode(Node):
    def __init__(self):
        super().__init__("rover_bringup")
        self.get_logger().info("Rover bringup node initialized")
        self.get_logger().info("[PLACEHOLDER] Log hardware initialization status here")
        self.pinger = self.create_publisher(String, "ping", 10)
        PING_EVERY = 1.0
        self.timer = self.create_timer(PING_EVERY, self.ping)

    def ping(self):
        msg = String()
        now = self.get_clock().now().to_msg()
        msg.data = f"Ping {now.sec}"
        self.pinger.publish(msg)


def main():
    rclpy.init()
    node = RoverBringupNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
