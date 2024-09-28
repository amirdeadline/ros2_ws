from django.http import JsonResponse
from .ros_controller import controller_node
from django.shortcuts import render
import psutil
import datetime
import subprocess
from django.http import JsonResponse
import rclpy
from std_msgs.msg import String
# from .ros_controller import log_collector

# Initialize ROS2 node in Django (if not already initialized)
# rclpy.init(args=None)


# View to render the control page
def index(request):
    return render(request, 'controller/index.html')

# Function to ping Google to check online status
def check_online_status():
    try:
        subprocess.check_output(['ping', '-c', '1', 'google.com'], stderr=subprocess.STDOUT)
        return "Online"
    except subprocess.CalledProcessError:
        return "Offline"

# Function to check if ROS2 is running
def check_ros_status():
    try:
        # Check if the ROS2 daemon is running by querying for active ROS2 nodes
        subprocess.check_output(['ros2', 'node', 'list'], stderr=subprocess.STDOUT)
        return "Running"
    except subprocess.CalledProcessError:
        return "Stopped"

# System status view
def system_status(request):
    # Check online status and ROS2 status
    internet_status = check_online_status()
    ros_status = check_ros_status()

    # Get current time
    current_time = datetime.datetime.now().strftime('%H:%M:%S')

    # Get CPU and RAM usage
    cpu_usage = psutil.cpu_percent(interval=1)
    ram_usage = psutil.virtual_memory().percent

    # Return data as JSON
    data = {
        'status': internet_status if internet_status == "Online" and ros_status == "Running" else "Offline",
        'time': current_time,
        'cpu': cpu_usage,
        'ram': ram_usage,
        'ros_status': ros_status,
        'internet_status': internet_status,
    }
    return JsonResponse(data)

# Drive control
def move(request):
    direction = request.GET.get('direction')
    if direction == 'forward':
        controller_node.publish_motor_command(0.35, 0.0)
    elif direction == 'backward':
        controller_node.publish_motor_command(-0.35, 0.0)
    elif direction == 'left':
        controller_node.publish_motor_command(0.0, 0.5)
    elif direction == 'right':
        controller_node.publish_motor_command(0.0, -0.5)
    elif direction == 'stop':
        controller_node.publish_motor_command(0.0, 0.0)
    return JsonResponse({'status': 'success'})

# Mower control
def mower_control(request):
    action = request.GET.get('action')
    if action == 'extend':
        controller_node.publish_mower_command('mower:e')  # Extend Mower LA
    elif action == 'retract':
        controller_node.publish_mower_command('mower:r')  # Retract Mower LA
    elif action == 'stop':
        controller_node.publish_mower_command('mower:s')  # Stop Mower LA
    elif action == 'motor_on':
        controller_node.publish_mower_motor_command('start')  # Start Mower Motors
    elif action == 'motor_off':
        controller_node.publish_mower_motor_command('stop')  # Stop Mower Motors
    return JsonResponse({'status': 'success'})

# Dethatcher control
def dethatcher_control(request):
    action = request.GET.get('action')
    if action == 'extend':
        controller_node.publish_dethatcher_command('dethatcher:e')  # Extend Dethatcher LA
    elif action == 'retract':
        controller_node.publish_dethatcher_command('dethatcher:r')  # Retract Dethatcher LA
    elif action == 'stop':
        controller_node.publish_dethatcher_command('dethatcher:s')  # Stop Dethatcher LA
    elif action == 'motor_on':
        controller_node.publish_dethatcher_motor_command('start')  # Start Dethatcher Motors
    elif action == 'motor_off':
        controller_node.publish_dethatcher_motor_command('stop')  # Stop Dethatcher Motors
    return JsonResponse({'status': 'success'})

# Aerator control
def aerator_control(request):
    action = request.GET.get('action')
    if action == 'extend':
        controller_node.publish_aerator_command('aerator:e')  # Extend Aerator LA
    elif action == 'retract':
        controller_node.publish_aerator_command('aerator:r')  # Retract Aerator LA
    elif action == 'stop':
        controller_node.publish_aerator_command('aerator:s')  # Stop Aerator LA
    return JsonResponse({'status': 'success'})

# Weed control
def weed_control(request):
    action = request.GET.get('action')
    if action == 'extend':
        controller_node.publish_weed_control_command('wc:e')  # Extend WC Linear Actuator
    elif action == 'retract':
        controller_node.publish_weed_control_command('wc:r')  # Retract WC Linear Actuator
    elif action == 'left':
        controller_node.publish_stepper_command(-1)  # Move WC Stepper Left
    elif action == 'right':
        controller_node.publish_stepper_command(1)   # Move WC Stepper Right
    elif action == 'stop':
        controller_node.publish_weed_control_command('wc:s')  # Stop WC movement
    return JsonResponse({'status': 'success'})

# Main battery control (lock/unlock)
def battery_control(request):
    action = request.GET.get('action')
    if action == 'lock':
        controller_node.publish_battery_command('lock')  # Lock the battery
    elif action == 'unlock':
        controller_node.publish_battery_command('unlock')  # Unlock the battery
    return JsonResponse({'status': 'success'})

# Logs
# def get_logs(request):
#     return JsonResponse({'logs': log_collector.logs})
# class LogCollector(Node):
#     def __init__(self):
#         super().__init__('log_collector')
#         self.logs = []
#         self.subscription = self.create_subscription(
#             String,
#             '/logs_system',
#             self.log_callback,
#             10)
#         self.subscription  # prevent unused variable warning

#     def log_callback(self, msg):
#         self.logs.append(msg.data)
#         if len(self.logs) > 100:  # Limit log history to 100 messages
#             self.logs.pop(0)

# log_collector = LogCollector()

# # Django view to get logs via AJAX
# def get_logs(request):
#     return JsonResponse({'logs': log_collector.logs})