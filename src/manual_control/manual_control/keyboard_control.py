#!/home/delta/lawnbot/venv/bin/python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
import pygame
import time

class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_motor = self.create_publisher(Twist, '/motor_commands', 10)
        self.publisher_wc = self.create_publisher(String, '/weed_control_la_commands', 10)
        self.publisher_stepper = self.create_publisher(Int32, '/weed_control_stepper_commands', 10)
        self.publisher_dethatcher_la = self.create_publisher(String, '/dethatcher_la_commands', 10)
        self.publisher_dethatcher_motor = self.create_publisher(String, '/dethatcher_motor_commands', 10)
        self.publisher_mower_la = self.create_publisher(String, '/mower_la_commands', 10)
        self.publisher_mower_motor = self.create_publisher(String, '/mower_motor_commands', 10)
        self.publisher_aerator_la = self.create_publisher(String, '/aerator_la_commands', 10)
        
        # Initialize pygame for keypress detection
        pygame.init()
        pygame.display.set_mode((100, 100))  # Required for key detection in pygame
        
        self.linear_speed = 0.35  # Speed for forward/reverse movement
        self.angular_speed = 0.5  # Speed for rotation
        self.command_active = False  # Flag to check if we are publishing motor commands
        self.weed_control_active = False  # Flag to check if we are publishing weed control commands
        self.stepper_active = False  # Flag to check if we are publishing stepper commands
        self.dethatcher_la_active = False  # Flag for dethatcher linear actuator commands
        self.mower_la_active = False  # Flag for mower linear actuator commands
        self.aerator_la_active = False  # Flag for aerator linear actuator commands

        # Print out available commands when the node starts
        self.print_help_menu()
        
        self.run()

    def print_help_menu(self):
        print("""
        Keyboard Control Help Menu:
        ---------------------------
        W: Move Forward
        S: Move Backward
        A: Rotate Counterclockwise
        D: Rotate Clockwise
        \u2191 Arrow Up: (WC)Extend Weed Control Linear Actuators (wc:e)
        \u2193 Arrow Down: (WC)Retract Weed Control Linear Actuators (wc:r)
        \u2192 Arrow Right: (WC)Step Right (1) for Weed Control Stepper Motor
        \u2190 Arrow Left: (WC)Step Left (-1) for Weed Control Stepper Motor
        P: (Dethatcher)Extend Dethatcher Linear Actuators (dethatcher:e)
        O: (Dethatcher)Retract Dethatcher Linear Actuators (dethatcher:r)
        I: (Dethatcher)Start Dethatcher Motor
        U: (Dethatcher)Stop Dethatcher Motor
        L: (Mower)Extend Mower Linear Actuators (mower:e)
        K: (Mower)Retract Mower Linear Actuators (mower:r)
        J: (Mower)Start Mower Motors (start)
        H: (Mower)Stop Mower Motors (stop)
        M: (Aerator)Extend Aerator Linear Actuators (aerator:e)
        N: (Aerator)Retract Aerator Linear Actuators (aerator:r)
        H: Show Help Menu
        Q: Quit
        """)
        self.get_logger().info("Help Menu Printed.")

    def run(self):
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return

                keys = pygame.key.get_pressed()

                # Motor control
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
                        self.stop_motor_movement()

                # Weed control LA commands
                if keys[pygame.K_UP]:
                    self.extend_weed_control()
                elif keys[pygame.K_DOWN]:
                    self.retract_weed_control()
                else:
                    if self.weed_control_active:
                        self.stop_weed_control()

                # Weed control stepper motor commands
                if keys[pygame.K_RIGHT]:
                    self.step_right()
                elif keys[pygame.K_LEFT]:
                    self.step_left()

                # Dethatcher LA control
                if keys[pygame.K_p]:
                    self.extend_dethatcher_la()
                elif keys[pygame.K_o]:
                    self.retract_dethatcher_la()
                else:
                    if self.dethatcher_la_active:
                        self.stop_dethatcher_la()

                # Dethatcher motor control
                if keys[pygame.K_i]:
                    self.start_dethatcher_motor()
                elif keys[pygame.K_u]:
                    self.stop_dethatcher_motor()

                # Mower LA control
                if keys[pygame.K_l]:
                    self.extend_mower_la()
                elif keys[pygame.K_k]:
                    self.retract_mower_la()
                else:
                    if self.mower_la_active:
                        self.stop_mower_la()

                # Mower motor control
                if keys[pygame.K_j]:
                    self.start_mower_motor()
                elif keys[pygame.K_h]:
                    self.stop_mower_motor()

                # Aerator LA control
                if keys[pygame.K_m]:
                    self.extend_aerator_la()
                elif keys[pygame.K_n]:
                    self.retract_aerator_la()
                else:
                    if self.aerator_la_active:
                        self.stop_aerator_la()

                # Show help menu
                if keys[pygame.K_h]:
                    self.print_help_menu()

                # Quit
                if keys[pygame.K_q]:
                    self.get_logger().info("Quitting Keyboard Controller Node.")
                    return

                # Limit the rate at which messages are published
                time.sleep(0.1)

        except KeyboardInterrupt:
            self.get_logger().info("Shutting down Keyboard Controller Node.")

    def publish_motor_command(self, linear_x, angular_z):
        # Publish a Twist message for motor commands
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher_motor.publish(twist)
        self.command_active = True
        self.get_logger().info(f"Published motor command: linear={linear_x}, angular={angular_z}")

    def publish_weed_control_command(self, command):
        # Publish a String message for weed control commands
        msg = String()
        msg.data = command
        self.publisher_wc.publish(msg)
        self.weed_control_active = True
        self.get_logger().info(f"Published weed control command: {command}")

    def publish_stepper_command(self, steps):
        # Publish an Int32 message for stepper motor commands
        msg = Int32()
        msg.data = steps
        self.publisher_stepper.publish(msg)
        self.get_logger().info(f"Published stepper motor command: {steps}")

    def publish_dethatcher_la_command(self, command):
        # Publish a String message for dethatcher linear actuator commands
        msg = String()
        msg.data = command
        self.publisher_dethatcher_la.publish(msg)
        self.dethatcher_la_active = True
        self.get_logger().info(f"Published dethatcher LA command: {command}")

    def publish_dethatcher_motor_command(self, command):
        # Publish a String message for dethatcher motor commands
        msg = String()
        msg.data = command
        self.publisher_dethatcher_motor.publish(msg)
        self.get_logger().info(f"Published dethatcher motor command: {command}")

    def publish_mower_la_command(self, command):
        # Publish a String message for mower linear actuator commands
        msg = String()
        msg.data = command
        self.publisher_mower_la.publish(msg)
        self.mower_la_active = True
        self.get_logger().info(f"Published mower LA command: {command}")

    def publish_mower_motor_command(self, command):
        # Publish a String message for mower motor commands
        msg = String()
        msg.data = command
        self.publisher_mower_motor.publish(msg)
        self.get_logger().info(f"Published mower motor command: {command}")

    def publish_aerator_la_command(self, command):
        # Publish a String message for aerator linear actuator commands
        msg = String()
        msg.data = command
        self.publisher_aerator_la.publish(msg)
        self.aerator_la_active = True
        self.get_logger().info(f"Published aerator LA command: {command}")

    # Motor functions
    def move_forward(self):
        self.publish_motor_command(self.linear_speed, 0.0)

    def move_backward(self):
        self.publish_motor_command(-self.linear_speed, 0.0)

    def rotate_clockwise(self):
        self.publish_motor_command(0.0, -self.angular_speed)

    def rotate_counterclockwise(self):
        self.publish_motor_command(0.0, self.angular_speed)

    def stop_motor_movement(self):
        # Stop all motor movement when no key is pressed
        self.publish_motor_command(0.0, 0.0)
        self.command_active = False

    # Weed control functions
    def extend_weed_control(self):
        self.publish_weed_control_command("wc:e")

    def retract_weed_control(self):
        self.publish_weed_control_command("wc:r")

    def stop_weed_control(self):
        # Stop weed control movement when no arrow key is pressed
        self.publish_weed_control_command("wc:s")
        self.weed_control_active = False

    # Weed control stepper motor functions
    def step_right(self):
        # Step the stepper motor right (1 step)
        self.publish_stepper_command(1)

    def step_left(self):
        # Step the stepper motor left (-1 step)
        self.publish_stepper_command(-1)

    # Dethatcher LA control functions
    def extend_dethatcher_la(self):
        self.publish_dethatcher_la_command("dethatcher:e")

    def retract_dethatcher_la(self):
        self.publish_dethatcher_la_command("dethatcher:r")

    def stop_dethatcher_la(self):
        # Stop dethatcher LA movement when neither 'P' nor 'O' is pressed
        self.publish_dethatcher_la_command("dethatcher:s")
        self.dethatcher_la_active = False

    # Dethatcher motor control functions
    def start_dethatcher_motor(self):
        self.publish_dethatcher_motor_command("start")

    def stop_dethatcher_motor(self):
        self.publish_dethatcher_motor_command("stop")

    # Mower LA control functions
    def extend_mower_la(self):
        self.publish_mower_la_command("mower:e")

    def retract_mower_la(self):
        self.publish_mower_la_command("mower:r")

    def stop_mower_la(self):
        # Stop mower LA movement when neither 'L' nor 'K' is pressed
        self.publish_mower_la_command("mower:s")
        self.mower_la_active = False

    # Mower motor control functions
    def start_mower_motor(self):
        self.publish_mower_motor_command("start")

    def stop_mower_motor(self):
        self.publish_mower_motor_command("stop")

    # Aerator LA control functions
    def extend_aerator_la(self):
        self.publish_aerator_la_command("aerator:e")

    def retract_aerator_la(self):
        self.publish_aerator_la_command("aerator:r")

    def stop_aerator_la(self):
        # Stop aerator LA movement when neither 'M' nor 'N' is pressed
        self.publish_aerator_la_command("aerator:s")
        self.aerator_la_active = False

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
