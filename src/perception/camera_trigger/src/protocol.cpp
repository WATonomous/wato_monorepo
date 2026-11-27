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

#include "camera_trigger/protocol.hpp"

#include <charconv>
#include <cstdint>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace camera_trigger
{

namespace
{

constexpr std::string_view kTerminator = "\r\n";
constexpr std::string_view kNotMeasured = "---";

bool isPrintableAscii(char c)
{
  const auto byte = static_cast<unsigned char>(c);
  return byte >= 0x20 && byte <= 0x7E;
}

int hexValue(char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return -1;
}

// Decimal unsigned integer that must fill the whole field: no sign, no spaces, no overflow.
bool parseUnsigned(std::string_view text, uint32_t & out)
{
  if (text.empty()) {
    return false;
  }
  const char * const end = text.data() + text.size();
  const auto [ptr, ec] = std::from_chars(text.data(), end, out);
  return ec == std::errc() && ptr == end;
}

std::vector<std::string_view> split(std::string_view text, char separator)
{
  std::vector<std::string_view> fields;
  std::size_t start = 0;
  while (true) {
    const std::size_t pos = text.find(separator, start);
    if (pos == std::string_view::npos) {
      fields.push_back(text.substr(start));
      return fields;
    }
    fields.push_back(text.substr(start, pos - start));
    start = pos + 1;
  }
}

// 3 hex digits, or "---" when the firmware doesn't measure strobes.
bool parseStrobeMask(std::string_view text, uint16_t & mask, bool & valid)
{
  if (text == kNotMeasured) {
    mask = 0;
    valid = false;
    return true;
  }
  if (text.size() != 3) {
    return false;
  }
  uint16_t value = 0;
  for (const char c : text) {
    const int digit = hexValue(c);
    if (digit < 0) {
      return false;
    }
    value = static_cast<uint16_t>((value << 4) | digit);
  }
  mask = value;
  valid = true;
  return true;
}

ParseResult parseTrigger(const std::vector<std::string_view> & fields, uint32_t pulses_per_second)
{
  // TRG,<version>,<seq>,<pps_count>,<pulse_idx>,<pps_locked>,<strobe_mask>
  if (fields.size() != 7) {
    return ParseError::kBadField;
  }
  TriggerReport report;
  uint32_t pulse_idx = 0;
  if (
    !parseUnsigned(fields[2], report.seq) || !parseUnsigned(fields[3], report.pps_count) ||
    !parseUnsigned(fields[4], pulse_idx))
  {
    return ParseError::kBadField;
  }
  if (pulse_idx >= pulses_per_second || pulse_idx > UINT8_MAX) {
    return ParseError::kBadField;
  }
  report.pulse_idx = static_cast<uint8_t>(pulse_idx);

  if (fields[5] == "1") {
    report.pps_locked = true;
  } else if (fields[5] == "0") {
    report.pps_locked = false;
  } else {
    return ParseError::kBadField;
  }

  if (!parseStrobeMask(fields[6], report.strobe_mask, report.strobe_mask_valid)) {
    return ParseError::kBadField;
  }
  return report;
}

ParseResult parseBoot(const std::vector<std::string_view> & fields)
{
  // BOOT,<version>,<fw_version>
  if (fields.size() != 3 || fields[2].empty()) {
    return ParseError::kBadField;
  }
  return BootReport{std::string(fields[2])};
}

}  // namespace

const char * toString(ParseError error)
{
  switch (error) {
    case ParseError::kTooLong:
      return "too_long";
    case ParseError::kBadFraming:
      return "bad_framing";
    case ParseError::kNonAscii:
      return "non_ascii";
    case ParseError::kBadChecksum:
      return "bad_checksum";
    case ParseError::kUnknownType:
      return "unknown_type";
    case ParseError::kBadVersion:
      return "bad_version";
    case ParseError::kBadField:
      return "bad_field";
  }
  return "unknown_error";
}

uint8_t checksum(std::string_view body)
{
  uint8_t value = 0;
  for (const char c : body) {
    value = static_cast<uint8_t>(value ^ static_cast<uint8_t>(c));
  }
  return value;
}

std::string frameLine(std::string_view body)
{
  static constexpr char kHex[] = "0123456789ABCDEF";
  const uint8_t value = checksum(body);
  std::string line(body);
  line += '*';
  line += kHex[value >> 4];
  line += kHex[value & 0x0F];
  line += kTerminator;
  return line;
}

ParseResult parseLine(std::string_view line, uint32_t pulses_per_second)
{
  if (line.size() > kMaxLineLength) {
    return ParseError::kTooLong;
  }
  if (line.size() < kTerminator.size() || line.substr(line.size() - kTerminator.size()) != kTerminator) {
    return ParseError::kBadFraming;
  }
  line.remove_suffix(kTerminator.size());

  for (const char c : line) {
    if (!isPrintableAscii(c)) {
      return ParseError::kNonAscii;
    }
  }

  // "<body>*XX": the checksum is the last 3 characters.
  const std::size_t star = line.rfind('*');
  if (star == std::string_view::npos || line.size() - star != 3) {
    return ParseError::kBadFraming;
  }
  const int high = hexValue(line[star + 1]);
  const int low = hexValue(line[star + 2]);
  if (high < 0 || low < 0) {
    return ParseError::kBadFraming;
  }
  const std::string_view body = line.substr(0, star);
  if (checksum(body) != static_cast<uint8_t>((high << 4) | low)) {
    return ParseError::kBadChecksum;
  }

  const std::vector<std::string_view> fields = split(body, ',');
  const std::string_view type = fields[0];
  if (type.empty()) {
    return ParseError::kBadField;
  }
  if (type != "TRG" && type != "BOOT") {
    return ParseError::kUnknownType;
  }

  uint32_t version = 0;
  if (fields.size() < 2 || !parseUnsigned(fields[1], version)) {
    return ParseError::kBadField;
  }
  if (version != kProtocolVersion) {
    return ParseError::kBadVersion;
  }

  if (type == "TRG") {
    return parseTrigger(fields, pulses_per_second);
  }
  return parseBoot(fields);
}

}  // namespace camera_trigger
