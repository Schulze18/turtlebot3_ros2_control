
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    empty_turtlebot3 = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(
                            get_package_share_directory('turtlebot3_description'), 'launch'), '/turtlebot3_empty.launch.py'])
                    )
            
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
            'turtlebot3_diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        empty_turtlebot3,
        load_joint_state_controller,
        load_diff_drive_controller,
    ])