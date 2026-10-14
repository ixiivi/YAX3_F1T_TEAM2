import os
from glob import glob
from setuptools import setup

package_name = 'race_stack'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
         [f for f in glob('maps/*') if os.path.isfile(f)]),
        # maps/<map_name>/ subdirectories (e.g. inner_bound.npy for Stanley Q2)
        # normpath 필수: glob('maps/*/')는 끝에 '/'가 붙어 basename이 빈 문자열이 됨
        *[(os.path.join('share', package_name, 'maps',
                        os.path.basename(os.path.normpath(d))),
           [f for f in glob(os.path.join(d, '*')) if os.path.isfile(f)])
          for d in glob('maps/*/') if os.path.isdir(d)],
        (os.path.join('share', package_name, 'waypoints'),
         [f for f in glob('waypoints/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Na',
    maintainer_email='nykwak0525@yonsei.ac.kr',
    description='RoboRacer 2026 one-click race stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gap_follower       = race_stack.gap_follower:main',
            'pure_pursuit       = race_stack.pure_pursuit:main',
            'stanley_controller = race_stack.stanley_controller:main',
            'map_controller     = race_stack.map_controller:main',
            'safety_node        = race_stack.safety_node:main',
            'mission_manager    = race_stack.mission_manager:main',
            'keyboard_teleop    = race_stack.keyboard_teleop:main',
            'joystick_teleop    = race_stack.joystick_teleop:main',
            'waypoint_logger    = race_stack.waypoint_logger:main',
            'gen_waypoints      = race_stack.gen_waypoints:main',
            'gen_waypoints_kappa = race_stack.gen_waypoints_kappa:main',
            'gen_raceline_tumftm = race_stack.gen_raceline_tumftm:main',
            'sim_bridge         = race_stack.sim_bridge:main',
            'imu_odom           = race_stack.imu_odom:main',
            'scan_deskew        = race_stack.scan_deskew:main',
            'auto_localize      = race_stack.auto_localize:main',
            'localization_monitor = race_stack.localization_monitor:main',
            'frenet_planner     = race_stack.frenet_planner:main',
            'slip_logger        = race_stack.slip_logger:main',
            'slip_friction      = race_stack.slip_friction:main',
            'slip_replay        = race_stack.slip_replay:main',
            'odom_scale         = race_stack.odom_scale:main',
            'ontrack_sysid      = race_stack.ontrack_sysid:main',
            'curvature_probe    = race_stack.curvature_probe:main',
            'set_speed_kappa    = race_stack.set_speed_kappa:main',
            'steer_trim         = race_stack.steer_trim:main',
            'gen_lut            = race_stack.vehicle_sim:main',
            'vehicle_sim_node   = race_stack.vehicle_sim_node:main',
            'sysid_nn           = race_stack.sysid_nn:main',
        ],
    },
)
