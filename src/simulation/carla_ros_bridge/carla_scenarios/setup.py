from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'carla_scenarios'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Scenario plugin system and scenario server node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scenario_server = carla_scenarios.scenario_server_node:main',
        ],
    },
)
