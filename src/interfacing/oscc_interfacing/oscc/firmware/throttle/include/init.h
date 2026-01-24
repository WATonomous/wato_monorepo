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
 * @file init.h
 * @brief Initialization functionality.
 *
 */

#ifndef _OSCC_THROTTLE_INIT_H_
#define _OSCC_THROTTLE_INIT_H_

// ****************************************************************************
// Function:    init_globals
//
// Purpose:     Initialize values of global variables.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void init_globals(void);

// ****************************************************************************
// Function:    init_devices
//
// Purpose:     Initialize physical devices on or connected to the module.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void init_devices(void);

// ****************************************************************************
// Function:    init_communication_interfaces
//
// Purpose:     Initialize the communication interfaces used by the module.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void init_communication_interfaces(void);

// ****************************************************************************
// Function:    start_timers
//
// Purpose:     Start timers for report publishing and fault checking.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void start_timers(void);

#endif /* _OSCC_THROTTLE_INIT_H_ */
