from setuptools import find_packages, setup

package_name = 'tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            't_node = tracking.tracker_nuscenes:main',
            'detection_publisher = tracking.detection_publisher:main'
        ],
    },
)