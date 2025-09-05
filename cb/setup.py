import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'cb'

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
    maintainer='Dingzhi Zhang',
    maintainer_email='dingzhi.zhang@tum.de',
    description='-',
    license='-',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "diablo_odometry = cb.get_Odom:main",
            "current_pose = cb.get_Pose:main",
            "current_path = cb.get_Path:main",
            "current_path_sim = cb.get_Path_sim:main",
            "pp_control = cb.PP_Controller:main",
            "pp_control_adaptive = cb.PP_Controller_adaptive:main",
            "motionctrl_diablo = cb.Motion_Ctrl_diablo:main",
            "motionctrl_sim = cb.Motion_Ctrl_sim:main",
            "current_image = cb.get_Image:main",
            "current_image_PiCam = cb.get_Image_PiCam:main",
            "current_AprilTag = cb.get_AprilTag:main",
            "current_ussensor = cb.get_USSensor:main",
            "state_machine = cb.state_machine:main",
            "state_machine_sim = cb.state_machine_sim:main",
            "serial_reader = cb.get_Sensors:main",
            "tracker_pose = cb.get_Tracker:main",
            "panel_sim = cb.get_Panel_sim:main",
            "led_service_node = cb.show_Led:main",
            
            
        ],
    },
)
