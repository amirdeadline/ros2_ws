#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class WeedControlStepperControlNode(Node):
    def __init__(self):
        super().__init__('weed_control_stepper_control')
        # Subscriber to the weed control stepper motor commands topic
        self.subscription = self.create_subscription(
            String,
            '/weed_control_stepper_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Setup for stepper motor control connected to Arduino
        self.STEPPER_ENA = 46  # Replace with correct GPIO pin for stepper motor ENA
        self.STEPPER_DIR = 22  # Direction pin
        self.STEPPER_PUL = 6   # Pulse pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.STEPPER_ENA, GPIO.OUT)
        GPIO.setup(self.STEPPER_DIR, GPIO.OUT)
        GPIO.setup(self.STEPPER_PUL, GPIO.OUT)
        self.pulse_delay = 0.01  # Delay for pulse

        self.get_logger().info("Weed Control Stepper Motor Control Node has been started.")
    
    def listener_callback(self, msg):
        command = int(msg.data)
        self.get_logger().info(f"Received stepper motor command: {command} steps")
        self.move_stepper_motor(command)

    def move_stepper_motor(self, steps):
        GPIO.output(self.STEPPER_ENA, GPIO.LOW)  # Enable stepper motor
        direction = GPIO.HIGH if steps > 0 else GPIO.LOW
        GPIO.output(self.STEPPER_DIR, direction)
        for _ in range(abs(steps)):
            GPIO.output(self.STEPPER_PUL, GPIO.HIGH)
            time.sleep(self.pulse_delay)
            GPIO.output(self.STEPPER_PUL, GPIO.LOW)
            time.sleep(self.pulse_delay)
        GPIO.output(self.STEPPER_ENA, GPIO.HIGH)  # Disable stepper motor

    def destroy(self):
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)

    weed_control_stepper_control_node = WeedControlStepperControlNode()

    rclpy.spin(weed_control_stepper_control_node)

    weed_control_stepper_control_node.destroy_node()
    rclpy.shutdown()
    weed_control_stepper_control_node.destroy()


if __name__ == '__main__':
    main()
