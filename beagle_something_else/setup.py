from setuptools import find_packages, setup

package_name = 'beagle_something_else'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='whddn915@gmail.com',
    description='TODO: Package description',
    license='Apache2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "moved_path_odom=beagle_something_else.beagle_moved_path_with_odom:main",
            "moved_path_pose=beagle_something_else.beagle_moved_path_with_pose:main"
        ],
    },
)
