from glob import glob
from setuptools import find_packages, setup

setup(
    name="carla_fake_planner", version="0.1.0", packages=find_packages(),
    data_files=[("share/ament_index/resource_index/packages", ["resource/carla_fake_planner"]),
                ("share/carla_fake_planner", ["package.xml"]),
                ("share/carla_fake_planner/config", glob("config/*.yaml"))],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="WATonomous", maintainer_email="hello@watonomous.ca",
    description="Simulation-only trajectory injection", license="Apache-2.0",
    entry_points={"console_scripts": ["carla_fake_planner = carla_fake_planner.node:main"]},
)
