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
 * @file arduino_init.h
 * @brief Arduino initialization.
 *
 */

#ifndef _OSCC_ARDUINO_INIT_H_
#define _OSCC_ARDUINO_INIT_H_

// ****************************************************************************
// Function:    init_arduino
//
// Purpose:     Perform initialization normally done by the compiler during
//              sketch compilation so that the firmware code can be a standard
//              main function with an infinite loop rather than an
//              Arduino-specific setup/loop system.
//
// Returns:     void
//
// Parameters:  void
//
// ****************************************************************************
void init_arduino(void);

#endif /* _OSCC_ARDUINO_INIT_H_ */
