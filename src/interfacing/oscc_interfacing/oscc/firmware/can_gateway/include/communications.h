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

/**
 * @file communications.h
 * @brief Communication functionality.
 *
 */

#ifndef _OSCC_CAN_GATEWAY_COMMUNICATIONS_H_
#define _OSCC_CAN_GATEWAY_COMMUNICATIONS_H_

#include "globals.h"

// ****************************************************************************
// Function:    check_for_module_reports
//
// Purpose:     Checks Control CAN bus for reports from the modules.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void check_for_module_reports(void);

// ****************************************************************************
// Function:    republish_obd_frames_to_control_can_bus
//
// Purpose:     Republish pertinent frames on the OBD CAN bus to the Control CAN
//              bus.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void republish_obd_frames_to_control_can_bus(void);

#endif /* _OSCC_CAN_GATEWAY_COMMUNICATIONS_H_ */
