from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_racing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
       
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
      
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'racing_node = f1tenth_racing.racing_node:main',
            'pure_pursuit_node = f1tenth_racing.pure_pursuit_node:main',
            'hybrid_controller_node = f1tenth_racing.hybrid_controller_node:main',
            'map_controller_node = f1tenth_racing.map_controller_node:main',
            'stanley_controller_node = f1tenth_racing.stanley_controller_node:main',
            'lap_timer_node = f1tenth_racing.lap_timer_node:main',
        ],
    },
)
