from setuptools import setup

package_name = 'lidar_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='s36gong, Steven Gong',
    maintainer_email='s36gong@uwaterloo.ca',
    description='lidar_object_detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_node = lidar_object_detection.reactive_node:main',
        ],
    },
)
