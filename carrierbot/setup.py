import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'carrierbot'

setup(
    name=package_name,
    version='1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include all launch files from this package
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        # include all config files from this package
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*')))
    ],
    install_requires=['setuptools', 'protobuf', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='Henrik Sand',
    maintainer_email='henrik.sand@tum.de',
    description='-',
    license='-',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "diablo_odometry = carrierbot.get_Odom:main",
            "current_pose = carrierbot.get_Pose:main",
            "current_path = carrierbot.get_Path:main",
            "current_path_sim = carrierbot.get_Path_sim:main",
            "pp_control = carrierbot.PP_Controller:main",
            "motionctrl_diablo = carrierbot.Motion_Ctrl_diablo:main",
            "motionctrl_sim = carrierbot.Motion_Ctrl_sim:main",
            "current_image = carrierbot.get_Image:main",
            "current_AprilTag = carrierbot.get_AprilTag:main",
            "state_machine = carrierbot.state_machine:main",
            "state_machine_sim = carrierbot.state_machine_sim:main",
            "serial_reader = carrierbot.get_Sensors:main",
            "tracker_pose = carrierbot.get_Tracker:main",
            "panel_sim = carrierbot.get_Panel_sim:main"
        ],
    },
)
