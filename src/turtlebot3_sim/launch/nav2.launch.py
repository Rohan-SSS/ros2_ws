import os
from time import sleep
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# This LD launches nav2, rviz2 and AMCL node
def generate_launch_description():

    # Packages 
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_turtlebot3_sim = get_package_share_directory('turtlebot3_sim')

    # Configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')

    # Launching nav2 node
    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': os.path.join(pkg_turtlebot3_sim, 'maps', 'turtleMap.yaml')
        }.items()
    )

    # Launching rviz2 node
    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d' + os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            )]
    )

    # Launchingg AMCL node
    set_init_amcl_pose_cmd = Node(
        package="turtlebot3_sim",
        executable="amcl_init_pose_publisher",
        name="amcl_init_pose_publisher",
        parameters=[{
            "x": -2.0,
            "y": -0.5,
        }]
    )

    ld = LaunchDescription()

    ld.add_action(nav2_launch_cmd)
    ld.add_action(set_init_amcl_pose_cmd)
    ld.add_action(rviz_launch_cmd)

    return ld