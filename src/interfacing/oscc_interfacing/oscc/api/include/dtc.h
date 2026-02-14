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
 * @file dtc.h
 * @brief DTC macros.
 *
 */

#ifndef _OSCC_DTC_H_
#define _OSCC_DTC_H_

/*
 * @brief Set a DTC bit in a DTC bitfield.
 *
 */
#define DTC_SET(bitfield, dtc) ((bitfield) |= (1 << (dtc)))

/*
 * @brief Clear a DTC bit in a DTC bitfield.
 *
 */
#define DTC_CLEAR(bitfield, dtc) ((bitfield) &= ~(1 << (dtc)))

/*
 * @brief Check if a DTC bit in a DTC bitfield is set.
 *
 */
#define DTC_CHECK(bitfield, dtc) ((bitfield) & (1 << (dtc)))

#endif /* _OSCC_DTC_H_ */
