import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    package_name='eurobot_launch'

    #Include the launch_sim.launch.py
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch', 'launch_sim.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Get the robot description
    urdf_path = os.path.join(get_package_share_directory('eurobot_description'), 'urdf', 'mon_robot.urdf')
    urdf_doc = xacro.parse(open(urdf_path, 'r'))
    xacro.process_doc(urdf_doc)
    robot_description = urdf_doc.toxml()

    robot_controller_config = os.path.join(get_package_share_directory('eurobot_description'), 'config', 'controller_config.yaml')

    return LaunchDescription([
        rsp,
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch', 'eurobot_state_publisher.py'
            )]),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/eurobot_teleop.py']
            )
        ),

        Node(
            package='eurobot_control',
            executable='eurobot_control_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                robot_controller_config
            ]
        )
    ])