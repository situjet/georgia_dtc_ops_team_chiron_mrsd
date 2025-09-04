from setuptools import setup
import os
from glob import glob

package_name = 'pose_estimator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'),
            glob('assets/models/*')),
        (os.path.join('share', package_name, 'data'),
            glob('assets/data/*')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'torch',
        'opencv-python',
        'scipy',
    ],
    zip_safe=True,
    maintainer='Yi Wu',
    maintainer_email='yiwu2@andrew.cmu.edu',
    description='Human pose detection node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_estimator = pose_estimator.pose_estimator:main'
        ],
    },
)