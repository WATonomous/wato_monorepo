from setuptools import setup
from glob import glob
import os

package_name = 'carla_sample_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vishal Jayakumar',
    maintainer_email='v3jayaku@watonomous.ca',
    description='Sample python node to read data from CARLA',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_sample_node = carla_sample_node.carla_sample_node:main'
        ],
    },
)
