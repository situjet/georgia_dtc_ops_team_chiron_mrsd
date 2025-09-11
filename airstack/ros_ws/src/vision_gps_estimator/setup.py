from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'vision_gps_estimator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yliu',
    maintainer_email='yliu@gatech.edu',
    description='Vision-based GPS target estimation using RTSP stream and MAVROS integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'integrated_node = vision_gps_estimator.integrated_node:main',
        ],
    },
)
