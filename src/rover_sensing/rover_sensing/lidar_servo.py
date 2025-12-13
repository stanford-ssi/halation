#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO

SERVO_PIN = 33
PWM_FREQ = 50

# Pitch angle corresponding to duty cycle positions
PITCH_LOW = 0.0   # duty_low = 0
PITCH_HIGH = 20.0   # duty_high = 10


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

        # Publisher for current pitch angle (stamped via publish time)
        self.angle_pub = self.create_publisher(Float32, 'lidar_pitch', 10)

        self.timer = self.create_timer(0.1, self.sweep_callback)

        self.get_logger().info("Servo Node Started. Sweeping up/down.")

    def sweep_callback(self):
        if self.current_high:
            self.pwm.ChangeDutyCycle(self.duty_low)
            pitch = PITCH_LOW
        else:
            self.pwm.ChangeDutyCycle(self.duty_high)
            pitch = PITCH_HIGH
        self.current_high = not self.current_high

        # Publish pitch angle (timestamp is when message is published)
        msg = Float32()
        msg.data = pitch
        self.angle_pub.publish(msg)

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
