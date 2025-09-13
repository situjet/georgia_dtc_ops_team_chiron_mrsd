from setuptools import find_packages, setup

package_name = 'burst_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Burst mode recorder for RTSP camera stream',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'burst_recorder_node = burst_recorder.burst_recorder_node:main',
        ],
    },
)
