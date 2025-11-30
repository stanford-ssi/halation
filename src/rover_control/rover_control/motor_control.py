from rclpy.node import Node
from std_msgs.msg import Float32
import adafruit_mcp4725
import board
import busio
import rclpy
import Jetson.GPIO as GPIO

VOLTAGE_RAMP_RATE = 10  # recalculations per second
PING_RATE = 0.1  # every n seconds

REAL_VOLTAGE = 5 # volts
VOLTAGE_MIN = -4095 # dac value
VOLTAGE_MAX = 4095 # dac value

I2C_ADDRESS = 0x60

i2c = busio.I2C(board.SCL, board.SDA)

class MotorControl(Node):
    def __init__(self):
        super().__init__("motor_control")
        self.dac_val = 0.0

        # i2c = busio.I2C(board.SCL, board.SDA)
        self.dac = adafruit_mcp4725.MCP4725(i2c, address=I2C_ADDRESS)
        # GPIO.output(GPIO_CHANNEL, GPIO.HIGH)

        self.get_logger().info(f"Motor control initialized with I2C address {I2C_ADDRESS}, {board.SCL}, {board.SDA}")

        self.voltage_publisher = self.create_publisher(Float32, "voltage", 10)
        self.voltage_subscriber = self.create_subscription(
            Float32, "target_voltage", self.handle_target_voltage, 10
        )

        self.create_timer(PING_RATE, self.ping_voltage)

    def handle_target_voltage(self, msg):
        target_dac = int((msg.data / REAL_VOLTAGE) * VOLTAGE_MAX)
        self.get_logger().info(f"Target voltage set to {target_dac}")
        if abs(self.dac_val - target_dac) < 0.01:
            return
        self.dac_val = max(min(target_dac, VOLTAGE_MAX), VOLTAGE_MIN)
        self.set_dac_val() 

    def set_dac_val(self):
        # direction = GPIO.HIGH if self.dac_val > 0 else GPIO.LOW
        # GPIO.output(GPIO_CHANNEL, direction) 
        print(f"Setting to: {int(abs(self.dac_val))} {type(self.dac_val)}")
        self.dac.raw_value = int(abs(self.dac_val))

    def ping_voltage(self):
        ping_val = (self.dac_val / abs(VOLTAGE_MAX)) * REAL_VOLTAGE
        ping_val = round(ping_val, 2)
        self.voltage_publisher.publish(Float32(data=ping_val))

    def __del__(self):
        # GPIO.cleanup()
        pass


def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorControl()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()
