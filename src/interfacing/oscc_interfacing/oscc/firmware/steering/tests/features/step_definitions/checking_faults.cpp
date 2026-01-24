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

WHEN("^the operator applies (.*) to the steering wheel$")
{
  REGEX_PARAM(int, steering_sensor_val);

  g_mock_arduino_analog_read_return[0] = steering_sensor_val;
  g_mock_arduino_analog_read_return[1] = 0;

  // need to call since newest read only contributes 1% for smoothing
  // and the average will start at 0.0 for the tests
  for (int i = 0; i < 200; ++i) {
    check_for_faults();
  }

  // set an elapsed time to account for hystoresis compensation
  g_mock_arduino_millis_return = Hysteresis_Time + 5;

  check_for_faults();
}
