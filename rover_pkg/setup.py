import os
from glob import glob
from setuptools import setup

package_name = 'rover_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zedrichu',
    maintainer_email='zedrichu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover = rover_pkg.rover:main',
            'cameras_publisher = rover_pkg.cameras_publisher:main',
            'gripper_camera = rover_pkg.gripper_camera:main',
            'monitor = rover_pkg.monitor:main',
            'new_rover = rover_pkg.new_rover:main',
            'nav_test_node = rover_pkg.nav_test_node:main',
            'hd_test_node = rover_pkg.hd_test_node:main',
	        'new_camera_cs = rover_pkg.new_camera_cs:main',
            'webrtc_camera_cs = rover_pkg.new_camera_webrtc:main',
            'test_cams = rover_pkg.test_cam:main',
            'test_nav2 = rover_pkg.test:main'
        ],
    },
)
