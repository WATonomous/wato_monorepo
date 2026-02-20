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
 * @file magic.h
 * @brief Definitions of the magic bytes identifying messages as from OSCC.
 *
 */

#ifndef _OSCC_MAGIC_H_
#define _OSCC_MAGIC_H_

/*
 * @brief First magic byte used in commands and reports to distinguish CAN
 *        frame as coming from OSCC (and not OBD).
 *
 */
#define OSCC_MAGIC_BYTE_0 (0x05)

/*
 * @brief Second magic byte used in commands and reports to distinguish CAN
 *        frame as coming from OSCC (and not OBD).
 *
 */
#define OSCC_MAGIC_BYTE_1 (0xCC)

#endif /* _OSCC_MAGIC_H_ */
