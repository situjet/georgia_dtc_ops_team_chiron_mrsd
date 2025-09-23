from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mcap_receiver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DTC Team',
    maintainer_email='team@dtc.com',
    description='MCAP file receiver and processor for remote telemetry via radio',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcap_to_rostopic_replayer = mcap_receiver.mcap_to_rostopic_replayer:main',
            'mcap_to_udp_bridge = mcap_receiver.mcap_to_udp_bridge:main',
            'simple_mcap_processor = mcap_receiver.simple_mcap_processor:main',
            'mcap_receiver_node = mcap_receiver.mcap_receiver_node:main',
        ],
    },
)
