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

WHEN("^an OBD CAN frame is received on the OBD CAN bus$")
{
  g_mock_mcp_can_check_receive_return = CAN_MSGAVAIL;
  g_mock_mcp_can_read_msg_buf_id = KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID;

  republish_obd_frames_to_control_can_bus();
}

THEN("^an OBD CAN frame should be published to the Control CAN bus$")
{
  assert_that(g_mock_mcp_can_send_msg_buf_id, is_equal_to(KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID));
}
