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

WHEN("^a steering report is published$")
{
  g_steering_control_state.enabled = true;
  g_steering_control_state.operator_override = true;
  g_steering_control_state.dtcs = 0x55;

  publish_steering_report();
}

THEN("^a steering report should be put on the control CAN bus$")
{
  assert_that(g_mock_mcp_can_send_msg_buf_id, is_equal_to(OSCC_STEERING_REPORT_CAN_ID));
  assert_that(g_mock_mcp_can_send_msg_buf_ext, is_equal_to(CAN_STANDARD));
  assert_that(g_mock_mcp_can_send_msg_buf_len, is_equal_to(OSCC_STEERING_REPORT_CAN_DLC));
}

THEN("^the steering report's fields should be set$")
{
  oscc_steering_report_s * steering_report = (oscc_steering_report_s *)g_mock_mcp_can_send_msg_buf_buf;

  assert_that(steering_report->magic[0], is_equal_to(OSCC_MAGIC_BYTE_0));

  assert_that(steering_report->magic[1], is_equal_to(OSCC_MAGIC_BYTE_1));

  assert_that(steering_report->enabled, is_equal_to(g_steering_control_state.enabled));

  assert_that(steering_report->operator_override, is_equal_to(g_steering_control_state.operator_override));

  assert_that(steering_report->dtcs, is_equal_to(g_steering_control_state.dtcs));
}
