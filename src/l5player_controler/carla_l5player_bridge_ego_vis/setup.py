from setuptools import setup
import os
from glob import glob
package_name = 'carla_l5player_bridge_ego_vis'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zww',
    maintainer_email='zhuwang2515@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'longitudinal_control_pid = carla_l5player_bridge_ego_vis.longitudinal_control_pid:main',
            'reference_line_recorder = carla_l5player_bridge_ego_vis.reference_line_recorder:main',
            'carla_l5player_vis_ego_vehicle = carla_l5player_bridge_ego_vis.carla_l5player_vis_ego_vehicle:main',
        ],
    },
)
