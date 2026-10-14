from glob import glob
import os
from setuptools import setup

package_name = 'contest_autonomy'
setup(
    name=package_name, version='1.0.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'auto_track = contest_autonomy.auto_track:main',
        'hybrid_driver = contest_autonomy.hybrid_driver:main',
        'mission_guard = contest_autonomy.mission_guard:main',
    ]},
)
