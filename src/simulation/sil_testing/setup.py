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
import os
from glob import glob

from setuptools import setup

package_name = "sil_testing"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.[pxy][yma]*")),
        (os.path.join("share", package_name, "config", "scenarios"), glob("config/scenarios/*.yaml")),
        (os.path.join("share", package_name, "config", "foxglove"), glob("config/foxglove/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="WATonomous",
    maintainer_email="hello@watonomous.ca",
    description="Software-In-the-Loop component testing harness for action components",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "trajectory_feeder_node = sil_testing.trajectory_feeder_node:main",
            "sil_monitor_node = sil_testing.sil_monitor_node:main",
            "scenario_runner_node = sil_testing.scenario_runner_node:main",
            "sil_analyzer = sil_testing.sil_analyzer:main",
        ],
    },
)
