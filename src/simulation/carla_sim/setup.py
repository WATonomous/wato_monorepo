# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from setuptools import setup
from glob import glob
import os

package_name = "carla_sample_node"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vishal Jayakumar",
    maintainer_email="v3jayaku@watonomous.ca",
    description="Sample python node to read data from CARLA",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "carla_sample_node = carla_sample_node.carla_sample_node:main"
        ],
    },
)
