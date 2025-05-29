from setuptools import setup
import os
from glob import glob

package_name = 'depth_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'include'), glob('launch/include/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'torch',
        'open3d',
        'opencv-python'
    ],
    zip_safe=True,
    maintainer='Parasmai',
    maintainer_email='pconjeevaram@watonomous.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_estimation_node = depth_estimation.depth_estimation:main'
        ],
    },
)
