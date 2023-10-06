import os
from glob import glob
from setuptools import setup

package_name = 'figueroa_ros2_project'
lib = 'figueroa_ros2_project/lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, lib],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "resource"), glob("resource/*.png")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedro',
    maintainer_email='lan.fig.pf@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'printer_sim_node = figueroa_ros2_project.printer_sim_node:main',
            'motor_x = figueroa_ros2_project.crane_motor_x:main',
            'motor_y = figueroa_ros2_project.crane_motor_y:main',
            'robot_logic = figueroa_ros2_project.crane_robot_logic:main',
            'service_node = figueroa_ros2_project.shape_service:main',  
        ],
    },
)
