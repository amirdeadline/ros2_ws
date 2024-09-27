#!/home/delta/lawnbot/venv/bin/python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import time

class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(Twist, '/motor_commands', 10)
        
        # Initialize pygame for keypress detection
        pygame.init()
        pygame.display.set_mode((100, 100))  # Required for key detection in pygame
        
        self.linear_speed = 0.35  # Speed for forward/reverse movement
        self.angular_speed = 0.5  # Speed for rotation
        self.command_active = False  # Flag to check if we are publishing commands
        
        self.run()

    def run(self):
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return

                keys = pygame.key.get_pressed()

                if keys[pygame.K_w]:
                    self.move_forward()
                elif keys[pygame.K_s]:
                    self.move_backward()
                elif keys[pygame.K_a]:
                    self.rotate_counterclockwise()
                elif keys[pygame.K_d]:
                    self.rotate_clockwise()
                else:
                    if self.command_active:
                        self.stop_movement()

                # Limit the rate at which messages are published
                time.sleep(0.1)

        except KeyboardInterrupt:
            self.get_logger().info("Shutting down Keyboard Controller Node.")

    def publish_command(self, linear_x, angular_z):
        # Publish a Twist message
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher.publish(twist)
        self.command_active = True
        self.get_logger().info(f"Published command: linear={linear_x}, angular={angular_z}")

    def move_forward(self):
        self.publish_command(self.linear_speed, 0.0)

    def move_backward(self):
        self.publish_command(-self.linear_speed, 0.0)

    def rotate_clockwise(self):
        self.publish_command(0.0, -self.angular_speed)

    def rotate_counterclockwise(self):
        self.publish_command(0.0, self.angular_speed)

    def stop_movement(self):
        # Stop all movement when no key is pressed
        self.publish_command(0.0, 0.0)
        self.command_active = False

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    keyboard_controller = KeyboardControllerNode()

    try:
        rclpy.spin(keyboard_controller)
    except KeyboardInterrupt:
        keyboard_controller.get_logger().info("Shutting down Keyboard Controller Node.")
    finally:
        keyboard_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
