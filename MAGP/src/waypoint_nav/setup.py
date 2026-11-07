from glob import glob

from setuptools import setup

package_name = 'waypoint_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'waypoint_editor = waypoint_nav.waypoint_editor:main',
            'map_editor = waypoint_nav.map_editor:main',
            'pure_pursuit = waypoint_nav.pure_pursuit:main',
            'rviz_waypoints = waypoint_nav.rviz_waypoints:main',
            'global_localize = waypoint_nav.global_localize:main',
        ],
    },
)
