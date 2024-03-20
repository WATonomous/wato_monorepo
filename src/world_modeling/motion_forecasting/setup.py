from setuptools import setup
import os
from glob import glob

package_name = 'motion_forecasting'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominik',
    maintainer_email='dom_ritz@watonomous.ca',
    description='A ROS 2 package for motion forecasting in autonomous systems.',
    license='Specify your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_forecasting_node = motion_forecasting_node:main'
        ],
    },
)