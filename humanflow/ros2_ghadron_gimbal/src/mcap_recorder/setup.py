from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mcap_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrsd',
    maintainer_email='yufanliu@andrew.cmu.edu',
    description='MCAP recorder for video and telemetry data',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcap_node = mcap_recorder.recorder:main'
        ],
    },
) 