from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'behaviour'

setup(
    name=package_name,
    version='0.0.0',
    packages=['behaviour'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bolty',
    maintainer_email='yoanjanan.thirumahan@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behaviour = behaviour.behaviour:main',
        ],
    },
)
