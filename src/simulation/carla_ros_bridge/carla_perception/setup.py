from setuptools import setup, find_packages

package_name = 'carla_perception'

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
    description='Sensor bridge nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_publisher = carla_perception.camera_publisher_node:main',
            'lidar_publisher = carla_perception.lidar_publisher_node:main',
            'bbox_publisher = carla_perception.bbox_publisher_node:main',
        ],
    },
)
