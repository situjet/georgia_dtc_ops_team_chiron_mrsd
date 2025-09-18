from setuptools import find_packages, setup

package_name = 'rtsp_streamer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rtsp_streamer.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='RTSP client that publishes JPEG compressed images',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtsp_streamer_node = rtsp_streamer.rtsp_streamer_node:main',
        ],
    },
)
