from setuptools import setup

package_name = 'rtsp_forwarder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rtsp_forwarder.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ops',
    maintainer_email='ops@example.com',
    description='TCP proxy that forwards RTSP stream to operator network',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtsp_forwarder_node = rtsp_forwarder.rtsp_forwarder_node:main',
        ],
    },
)
