import os
from glob import glob
from setuptools import setup

package_name = 'digit_json'
submodule_action_commands = 'digit_json/digit_action_commands'
submodule_observation_publishers = 'digit_json/observation_publishers'
submodule_pkg_delivery = 'digit_json/package_delivery'
submodule_pkg_delivery_library = 'digit_json/package_delivery/library'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule_action_commands, submodule_observation_publishers, submodule_pkg_delivery, submodule_pkg_delivery_library],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/observation/*_launch.py')),
        (os.path.join('share', package_name), glob('launch/observation/perception/*_launch.py')),
        (os.path.join('share', package_name), glob('launch/observation/state/*_launch.py')),
        (os.path.join('share', package_name), glob('launch/digit_action_commands/*_launch.py')),
        (os.path.join('share', package_name), glob('launch/package_delivery/*_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grant Gibson',
    maintainer_email='grantgib@umich.edu',
    description='Publish Observation for Digit-v3 Release 2022.02.22',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_publisher = digit_json.observation_publishers.pointcloud_publisher:main',
            'fake_pointcloud_publisher = digit_json.observation_publishers.fake_pointcloud_publisher:main',
            'image_publisher = digit_json.observation_publishers.image_publisher:main',
            'state_publisher = digit_json.observation_publishers.state_publisher:main',
            'cmd_vel_subscriber = digit_json.digit_action_commands.cmd_vel_subscriber:main',
            'cmd_goto_subscriber = digit_json.digit_action_commands.cmd_goto_subscriber:main',
            'test_pub_cmd_vel = digit_json.digit_action_commands.test_pub_cmd_vel:main',
            'test_pub_cmd_goto = digit_json.digit_action_commands.test_pub_cmd_goto:main',
            'ford_ar_interface = digit_json.package_delivery.ford_ar_interface_v2:main',
        ],
    },
)
