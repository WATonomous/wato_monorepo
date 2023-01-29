from setuptools import setup

package_name = 'sample'

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
    description='sample node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_node = sample.reactive_node:main',
        ],
    },
)
