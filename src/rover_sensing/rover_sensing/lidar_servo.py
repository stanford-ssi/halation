#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO

SERVO_PIN = 33
PWM_FREQ = 50


class ServoDriver(Node):
    def __init__(self):
        super().__init__('lidar_servo')

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
        self.pwm.start(3.0)

        self.duty_low = 0
        self.duty_high = 10
        self.current_high = False

        self.timer = self.create_timer(0.1, self.sweep_callback)

        self.get_logger().info("Servo Node Started. Sweeping up/down.")

    def sweep_callback(self):
        if self.current_high:
            self.pwm.ChangeDutyCycle(self.duty_low)
        else:
            self.pwm.ChangeDutyCycle(self.duty_high)
        self.current_high = not self.current_high

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
