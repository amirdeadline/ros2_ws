from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to each package's launch file
    driver_launch = os.path.join(get_package_share_directory('drive'), 'launch', 'drive_launch.py')
    aerator_launch = os.path.join(get_package_share_directory('aerator'), 'launch', 'aerator_launch.py')
    mower_launch = os.path.join(get_package_share_directory('mower'), 'launch', 'mower_launch.py')
    weed_control_launch = os.path.join(get_package_share_directory('weed_control'), 'launch', 'weed_control_launch.py')
    dethatcher_launch = os.path.join(get_package_share_directory('dethatcher'), 'launch', 'dethatcher_launch.py')

    # Include each launch file
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(aerator_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mower_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(weed_control_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dethatcher_launch)
        ),
    ])
