from setuptools import setup

package_name = 'tak_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='ROS 2 node to publish NavSatFix stream to WinTAK as CoT',
    license='MIT',
    entry_points={
        'console_scripts': [
            'navsatfix_to_wintak = tak_bridge.navsatfix_to_wintak:main',
        ],
    },
)
