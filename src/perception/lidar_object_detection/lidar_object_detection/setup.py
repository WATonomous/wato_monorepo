import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'lidar_object_detection'
package_name = 'lidar_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dan Huynh',
    maintainer_email='danielrhuynh@watonomous.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = lidar_object_detection.test_node:main',
            'inference = lidar_object_detection.inference:main'
        ],
    },
)
