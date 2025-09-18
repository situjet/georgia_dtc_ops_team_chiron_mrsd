from setuptools import setup

package_name = 'burst_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=['burst_recorder_node'],
    package_dir={'': '.'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Burst mode video recorder for RTSP streams, storing to MCAP via ROS2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'burst_recorder_node = burst_recorder.burst_recorder_node:main',
        ],
    },
)
