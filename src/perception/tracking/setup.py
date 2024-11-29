from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kishoreyogaraj',
    maintainer_email='kishore.yogaraj@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_node = tracking.tracker:main',
            'detection_publisher = tracking.detection_publisher:main',
            'tracker_visualization = tracking.tracker_visualization:main'
        ],
    },
)