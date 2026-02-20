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

    moved_path_enabled = LaunchConfiguration('moved_path', default='false')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('beagle_bringup'),'launch','beagle_state_publisher.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'beagle','-x','0.000000','-y','-0.000000','-z','0.000000'],
                        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('beagle_gazebo'), 'config', 'just_for_gazebo.rviz')],
        condition=IfCondition(rviz_enabled),
    )

    moved_path = Node(
        package='beagle_something_else',
        executable='moved_path_odom',
        condition=IfCondition(moved_path_enabled),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(get_package_share_directory('beagle_gazebo'),
                                        'worlds','thin_maze.world'),''],# chagne here to change world 
            description='SDF world file'
        ),
        DeclareLaunchArgument('launch_rviz', default_value='false', description='Launch RViz'),
        DeclareLaunchArgument('moved_path', default_value='false', description='Run Moved Path'),
        rsp,
        gazebo,
        spawn_entity,
        moved_path,
        rviz
    ])
