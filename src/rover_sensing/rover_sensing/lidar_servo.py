#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO

SERVO_PIN = 7
PWM_FREQ = 50


class ServoDriver(Node):
    def __init__(self):
        super().__init__('lidar_servo')

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
        self.pwm.start(0)

        self.subscription = self.create_subscription(
            Float32, 'servo_angle', self.listener_callback, 10)

        self.get_logger().info("Servo Node Started. Listening on /servo_angle")

    def map_angle(self, angle):
        # Constrain angle 0-180
        angle = max(0, min(180, angle))
        # Map 0-180 deg to 600-2400 us (FS5103B specs)
        pulse_us = 600.0 + (angle / 180.0) * (2400.0 - 600.0)
        return (pulse_us / 20000.0) * 100.0

    def listener_callback(self, msg):
        duty = self.map_angle(msg.data)
        self.pwm.ChangeDutyCycle(duty)
        self.get_logger().info(f'Set angle: {msg.data}')

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("GPIO cleanup done.")


def main(args=None):
    rclpy.init(args=args)

    driver = ServoDriver()

    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.cleanup()
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
