#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class MowerMotorControlNode(Node):
    def __init__(self):
        super().__init__('mower_motor_control')
        # Subscriber to the mower motor commands topic
        self.subscription = self.create_subscription(
            String,
            '/mower_motor_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Setup for two ESCs connected to pin 10 and pin 11
        self.ESC1_PIN = 10
        self.ESC2_PIN = 11
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ESC1_PIN, GPIO.OUT)
        GPIO.setup(self.ESC2_PIN, GPIO.OUT)
        self.esc1_pwm = GPIO.PWM(self.ESC1_PIN, 50)  # PWM at 50Hz
        self.esc2_pwm = GPIO.PWM(self.ESC2_PIN, 50)  # PWM at 50Hz
        self.esc1_pwm.start(0)
        self.esc2_pwm.start(0)

        self.get_logger().info("Mower Motor Control Node has been started.")
    
    def listener_callback(self, msg):
        command = msg.data
        if command == "start":
            self.get_logger().info("Starting mower motors")
            self.start_motors()
        elif command == "stop":
            self.get_logger().info("Stopping mower motors")
            self.stop_motors()
        else:
            self.get_logger().warn(f"Unknown command received: {command}")

    def start_motors(self):
        self.esc1_pwm.ChangeDutyCycle(950 / 20000 * 100)  # 950ms for starting signal
        self.esc2_pwm.ChangeDutyCycle(950 / 20000 * 100)
        time.sleep(0.5)

    def stop_motors(self):
        self.esc1_pwm.ChangeDutyCycle(800 / 20000 * 100)  # 800ms for stopping signal
        self.esc2_pwm.ChangeDutyCycle(800 / 20000 * 100)
        time.sleep(0.5)

    def destroy(self):
        self.esc1_pwm.stop()
        self.esc2_pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)

    mower_motor_control_node = MowerMotorControlNode()

    rclpy.spin(mower_motor_control_node)

    mower_motor_control_node.destroy_node()
    rclpy.shutdown()
    mower_motor_control_node.destroy()


if __name__ == '__main__':
    main()
