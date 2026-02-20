from setuptools import find_packages, setup

package_name = 'beagle_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulation_simple_navigation.launch.py']),
        ('share/' + package_name + '/launch', ['launch/offline_simple_navigation.launch.py']),
        ('share/' + package_name + '/launch', ['launch/navigation_launch.py']),
        ('share/' + package_name + '/launch', ['launch/navigation_launch_sim.py']),
        ('share/' + package_name + '/params', ['params/nav2_params.yaml']),
        ('share/' + package_name + '/params', ['params/nav2_params_sim.yaml'])
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='whddn915@gmail.com',
    description='Beagle ROS2 navigation launch script',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "astar_path_planner=beagle_navigation.global_path_planning_astar:main",
            "pure_pursuit_path_tracker=beagle_navigation.path_tracker_pure_pursuit:main",
            "astar_path_planner_narrow=beagle_navigation.global_path_planning_astar_narrow:main"
        ],
    },
)

