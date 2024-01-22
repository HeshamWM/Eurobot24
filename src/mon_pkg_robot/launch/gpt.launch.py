import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name='mon_pkg_robot'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','sim.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazerbo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'
        )])
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot. 
    spawn_entity=Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-topic','robot_description',
                                 '-entity', 'mon_robot_mecanum'],
                                 output='screen')
    

    # Launch them all !
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,

        # Lancement du contrôleur matériel
        Node(
            package='ros2_control',
            executable='controller_manager',
            name='controller_manager',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': False},  # Ajoutez d'autres paramètres selon vos besoins
                {'hardware_interface':  'config/hardware.yaml'},
                {'controllers': 'config/controllers.yaml'}
                #{'controller_manager': {'namespace': 'my_robot'}}
            ],
        ),

        # Lancement du contrôleur de joint state
        Node(
            package='joint_state_controller',
            executable='joint_state_controller',
            name='joint_state_controller',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False}],
        ),

        # Lancement du contrôleur PID de vélocité pour chaque roue
        Node(
            package='velocity_controllers',
            executable='joint_velocity_controller',
            name='mecanum_wheel_fl_velocity_controller',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False}],
        ),
        # Ajoutez des nœuds similaires pour les autres contrôleurs de roues

        # Lancement du contrôleur de lecteur mécanum
        Node(
            package='differential_drive_controller',
            executable='differential_drive_controller',
            name='mecanum_drive_controller',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False}],
        ),
    ])