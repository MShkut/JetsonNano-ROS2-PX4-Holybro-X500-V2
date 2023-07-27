from setuptools import setup
import os
from glob import glob

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mitch',
    maintainer_email='mitch@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_sim_controller = drone_control.my_robot_sim_controller:main',
            'land = drone_control.land:main',
            'offboard_aruco = drone_control.offboard_aruco:main',
            'offboard_hover = drone_control.offboard_hover:main',
            'offboard_test = drone_control.offboard_test:main'
        ],
    },
)
