from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch/'),
         glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rxricardo',
    maintainer_email='rxricardo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            'control_packbot1 = my_robot_controller.control_packbot_1:main',
            'control_packbot2 = my_robot_controller.control_packbot_2:main',

            'marker_detector1 = my_robot_controller.marker_detector_1:main',
            'marker_detector2 = my_robot_controller.marker_detector_2:main',
            
            'nav_to_pose = my_robot_controller.nav_to_pose:main',
            'transformation = my_robot_controller.transformation:main',
    
        ],
    },
)
