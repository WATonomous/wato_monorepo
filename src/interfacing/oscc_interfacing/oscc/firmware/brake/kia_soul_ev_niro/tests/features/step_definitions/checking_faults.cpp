// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

WHEN("^the operator applies (.*) to the brake pedal for (\\d+) ms$")
{
  REGEX_PARAM(int, brake_sensor_val);
  REGEX_PARAM(int, duration);

  g_mock_arduino_analog_read_return[0] = brake_sensor_val;
  g_mock_arduino_analog_read_return[1] = brake_sensor_val;

  g_mock_arduino_millis_return = 1;
  check_for_faults();

  g_mock_arduino_millis_return += duration;
  check_for_faults();
}
