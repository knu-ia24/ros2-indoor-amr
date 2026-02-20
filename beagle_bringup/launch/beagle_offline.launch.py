import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node



def generate_launch_description():

    rviz_enabled = LaunchConfiguration('launch_rviz', default='false')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('beagle_bringup'), 'config', 'simple_offline_bringup.rviz')],
        condition=IfCondition(rviz_enabled),
    )

    beagle_core = Node(
        package='beagle_bringup',
        executable='beagle_core.py', #offline 비글 활성화  
    )


    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='false', description='Launch RViz'),
        beagle_core,
        rviz
    ])
