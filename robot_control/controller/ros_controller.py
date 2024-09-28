import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
import threading

# Function to spin the ROS2 node in a separate thread
def ros_spin_thread(node):
    rclpy.spin(node)

rclpy.init()

# Create ROS2 Node for controlling the robot
class ROSController(Node):
    def __init__(self):
        super().__init__('web_controller_node')
        # Drive control
        self.publisher_motor = self.create_publisher(Twist, '/motor_commands', 10)
        
        # Weed control
        self.publisher_wc = self.create_publisher(String, '/weed_control_la_commands', 10)
        self.publisher_stepper = self.create_publisher(Int32, '/weed_control_stepper_commands', 10)
        
        # Mower control
        self.publisher_mower = self.create_publisher(String, '/mower_la_commands', 10)
        self.publisher_mower_motor = self.create_publisher(String, '/mower_motor_commands', 10)

        # Dethatcher control
        self.publisher_dethatcher = self.create_publisher(String, '/dethatcher_la_commands', 10)
        self.publisher_dethatcher_motor = self.create_publisher(String, '/dethatcher_motor_commands', 10)

        # Aerator control
        self.publisher_aerator = self.create_publisher(String, '/aerator_la_commands', 10)

        # Battery control
        self.publisher_battery = self.create_publisher(String, '/battery_control', 10)

    # Drive Control
    def publish_motor_command(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher_motor.publish(twist)

    # Weed Control
    def publish_weed_control_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_wc.publish(msg)

    def publish_stepper_command(self, steps):
        msg = Int32()
        msg.data = steps
        self.publisher_stepper.publish(msg)

    # Mower Control
    def publish_mower_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_mower.publish(msg)

    def publish_mower_motor_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_mower_motor.publish(msg)

    # Dethatcher Control
    def publish_dethatcher_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_dethatcher.publish(msg)

    def publish_dethatcher_motor_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_dethatcher_motor.publish(msg)

    # Aerator Control
    def publish_aerator_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_aerator.publish(msg)

    # Battery Control (Lock/Unlock)
    def publish_battery_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_battery.publish(msg)

# Instantiate the ROS2 Node
controller_node = ROSController()

# Run ROS2 spinning in a separate thread
ros_thread = threading.Thread(target=ros_spin_thread, args=(controller_node,), daemon=True)
ros_thread.start()
