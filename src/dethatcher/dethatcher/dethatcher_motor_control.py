#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class DethatcherMotorControlNode(Node):
    def __init__(self):
        super().__init__('dethatcher_motor_control')
        # Subscriber to the dethatcher motor commands topic
        self.subscription = self.create_subscription(
            String,
            '/dethatcher_motor_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Set up GPIO pin for the ESC (connected to pin 9 on the Raspberry Pi)
        self.ESC_PIN = 9
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ESC_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(self.ESC_PIN, 50)  # PWM at 50Hz
        self.pwm.start(0)

        self.get_logger().info("Dethatcher Motor Control Node has been started.")
    
    def listener_callback(self, msg):
        command = msg.data
        if command == "start":
            self.get_logger().info("Starting dethatcher motor")
            self.start_motor()
        elif command == "stop":
            self.get_logger().info("Stopping dethatcher motor")
            self.stop_motor()
        else:
            self.get_logger().warn(f"Unknown command received: {command}")

    def start_motor(self):
        self.pwm.ChangeDutyCycle(950 / 20000 * 100)  # 950ms for starting signal
        time.sleep(0.5)

    def stop_motor(self):
        self.pwm.ChangeDutyCycle(800 / 20000 * 100)  # 800ms for stopping signal
        time.sleep(0.5)

    def destroy(self):
        self.pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)

    dethatcher_motor_control_node = DethatcherMotorControlNode()

    rclpy.spin(dethatcher_motor_control_node)

    dethatcher_motor_control_node.destroy_node()
    rclpy.shutdown()
    dethatcher_motor_control_node.destroy()


if __name__ == '__main__':
    main()
