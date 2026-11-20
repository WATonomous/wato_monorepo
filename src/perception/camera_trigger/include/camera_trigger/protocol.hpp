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

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <variant>

// Parser for the report lines sent by the camera trigger board. No ROS dependencies.
// Protocol definition: docs/serial_protocol.md

namespace camera_trigger
{

/// Largest accepted line, terminator included.
inline constexpr std::size_t kMaxLineLength = 256;

/// Protocol version this parser understands.
inline constexpr uint32_t kProtocolVersion = 1;

/// Why a line was rejected.
enum class ParseError
{
  kTooLong,  ///< longer than kMaxLineLength
  kBadFraming,  ///< no "\r\n" terminator, or no "*XX" checksum suffix
  kNonAscii,  ///< contains a byte outside printable ASCII
  kBadChecksum,  ///< checksum does not match the line body
  kUnknownType,  ///< well-formed line of a type this parser doesn't handle (e.g. STAT); ignored
  kBadVersion,  ///< protocol version other than kProtocolVersion
  kBadField,  ///< wrong number of fields, or a field that is empty, malformed or out of range
};

/// Short identifier for logs and diagnostics, e.g. "bad_checksum".
const char * toString(ParseError error);

/// A TRG line: one trigger pulse.
struct TriggerReport
{
  uint32_t seq{0};
  uint32_t pps_count{0};
  uint8_t pulse_idx{0};
  bool pps_locked{false};
  uint16_t strobe_mask{0};
  bool strobe_mask_valid{false};
};

/// A BOOT line: the board has (re)started.
struct BootReport
{
  std::string fw_version;
};

using ParseResult = std::variant<TriggerReport, BootReport, ParseError>;

/// Parses one complete line, terminator included (one UDP datagram carries one line).
/// @param pulses_per_second pulse_idx must be below this
ParseResult parseLine(std::string_view line, uint32_t pulses_per_second);

/// XOR of all bytes of `body` (NMEA-style checksum).
uint8_t checksum(std::string_view body);

/// Builds a complete line "<body>*XX\r\n".
std::string frameLine(std::string_view body);

}  // namespace camera_trigger
