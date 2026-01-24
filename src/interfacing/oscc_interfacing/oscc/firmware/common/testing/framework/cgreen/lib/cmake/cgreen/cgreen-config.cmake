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
# - Config file for the cgreen package
# It defines the following variables
#  CGREEN_CMAKE_DIR - include directories for cgreen
#  CGREEN_INCLUDE_DIRS - include directories for cgreen
#  CGREEN_LIBRARIES    - libraries to link against
#  CGREEN_EXECUTABLE   - the cgreen executable

get_filename_component( CGREEN_CMAKE_DIRS "${CMAKE_CURRENT_LIST_FILE}" PATH )

#  leave this up to cmake
find_path(CGREEN_INCLUDE_DIRS NAMES cgreen/cgreen.h)

set( CGREEN_LIBRARIES cgreen )
set( CGREEN_EXECUTABLE cgreen-runner )
