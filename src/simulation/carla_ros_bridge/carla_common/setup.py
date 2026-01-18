from setuptools import setup, find_packages

package_name = "carla_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="WATonomous",
    maintainer_email="infra@watonomous.ca",
    description="Common utilities for CARLA ROS bridge packages",
    license="Apache-2.0",
)
