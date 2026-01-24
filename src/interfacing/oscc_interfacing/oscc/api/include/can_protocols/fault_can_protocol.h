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
 * @file fault_can_protocol.h
 * @brief Fault CAN Protocol.
 *
 */

#ifndef _OSCC_FAULT_CAN_PROTOCOL_H_
#define _OSCC_FAULT_CAN_PROTOCOL_H_

#include <stdint.h>

#include "magic.h"

/*
 * @brief CAN ID representing the range of global messages.
 *
 */
#define OSCC_FAULT_CAN_ID_INDEX (0xA0)

/*
 * @brief Fault report message (CAN frame) ID.
 *
 */
#define OSCC_FAULT_REPORT_CAN_ID (0xAF)

/*
 * @brief Fault report message (CAN frame) length.
 *
 */
#define OSCC_FAULT_REPORT_CAN_DLC (8)

typedef enum
{
  FAULT_ORIGIN_BRAKE,
  FAULT_ORIGIN_STEERING,
  FAULT_ORIGIN_THROTTLE
} fault_origin_id_t;

#pragma pack(push)
#pragma pack(1)

/**
 * @brief Fault report message data.
 *
 * Message size (CAN frame DLC): \ref OSCC_FAULT_REPORT_CAN_DLC
 *
 */
typedef struct
{
  uint8_t magic[2]; /*!< Magic number identifying CAN frame as from OSCC.
                       *   Byte 0 should be \ref OSCC_MAGIC_BYTE_0.
                       *   Byte 1 should be \ref OSCC_MAGIC_BYTE_1. */

  uint32_t fault_origin_id; /*!< ID of the module that is sending out the fault. */

  uint8_t dtcs; /*!< DTC bitfield of the module that is sending out the fault. */

  uint8_t reserved; /*!< Reserved */
} oscc_fault_report_s;

#pragma pack(pop)

#endif /* _OSCC_FAULT_CAN_PROTOCOL_H_ */
