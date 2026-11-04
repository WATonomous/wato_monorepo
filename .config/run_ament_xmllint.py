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

"""Run the upstream XML validator with locally stored ROS package schemas."""

import os
from pathlib import Path
import sys

from ament_xmllint.main import main

if __name__ == "__main__":
    # The validator changes directories for each XML file, so use an absolute URI.
    catalog = Path(__file__).resolve().parent / "ros_schema" / "catalog.xml"
    os.environ["XML_CATALOG_FILES"] = catalog.as_uri()
    sys.exit(main())
