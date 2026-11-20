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

#include <cctype>
#include <cstdint>
#include <string>
#include <variant>

#include <catch2/catch_test_macros.hpp>

#include "camera_trigger/protocol.hpp"

using camera_trigger::BootReport;
using camera_trigger::checksum;
using camera_trigger::frameLine;
using camera_trigger::kMaxLineLength;
using camera_trigger::ParseError;
using camera_trigger::parseLine;
using camera_trigger::ParseResult;
using camera_trigger::TriggerReport;

namespace
{

constexpr uint32_t kPulsesPerSecond = 20;

ParseResult parse(const std::string & line)
{
  return parseLine(line, kPulsesPerSecond);
}

// Parses a correctly framed body and returns the error (fails the test if it parsed).
ParseError errorFor(const std::string & line)
{
  const ParseResult result = parse(line);
  REQUIRE(std::holds_alternative<ParseError>(result));
  return std::get<ParseError>(result);
}

TriggerReport triggerFor(const std::string & line)
{
  const ParseResult result = parse(line);
  REQUIRE(std::holds_alternative<TriggerReport>(result));
  return std::get<TriggerReport>(result);
}

}  // namespace

TEST_CASE("Checksum is the XOR of the body bytes", "[protocol]")
{
  CHECK(checksum("") == 0x00);
  CHECK(checksum("A") == 0x41);
  CHECK(checksum("AB") == (0x41 ^ 0x42));
  CHECK(frameLine("AB") == "AB*03\r\n");
}

TEST_CASE("Valid TRG line is parsed", "[protocol]")
{
  const TriggerReport report = triggerFor(frameLine("TRG,1,1234,61,14,1,FFF"));
  CHECK(report.seq == 1234);
  CHECK(report.pps_count == 61);
  CHECK(report.pulse_idx == 14);
  CHECK(report.pps_locked);
  CHECK(report.strobe_mask == 0xFFF);
  CHECK(report.strobe_mask_valid);
}

TEST_CASE("TRG field values at their limits", "[protocol]")
{
  SECTION("largest uint32 seq and pps_count")
  {
    const TriggerReport report = triggerFor(frameLine("TRG,1,4294967295,4294967295,0,0,000"));
    CHECK(report.seq == 4294967295U);
    CHECK(report.pps_count == 4294967295U);
    CHECK_FALSE(report.pps_locked);
    CHECK(report.strobe_mask == 0);
    CHECK(report.strobe_mask_valid);
  }
  SECTION("last pulse index of the second")
  {
    CHECK(triggerFor(frameLine("TRG,1,0,0,19,1,FFF")).pulse_idx == 19);
  }
  SECTION("strobes not measured")
  {
    const TriggerReport report = triggerFor(frameLine("TRG,1,5,0,5,1,---"));
    CHECK_FALSE(report.strobe_mask_valid);
    CHECK(report.strobe_mask == 0);
  }
  SECTION("single strobe bits and lowercase hex")
  {
    CHECK(triggerFor(frameLine("TRG,1,5,0,5,1,004")).strobe_mask == 0x004);
    CHECK(triggerFor(frameLine("TRG,1,5,0,5,1,a5f")).strobe_mask == 0xA5F);
  }
}

TEST_CASE("Valid BOOT line is parsed", "[protocol]")
{
  const ParseResult result = parse(frameLine("BOOT,1,v1.2.0-abc"));
  REQUIRE(std::holds_alternative<BootReport>(result));
  CHECK(std::get<BootReport>(result).fw_version == "v1.2.0-abc");
}

TEST_CASE("Lowercase checksum digits are accepted", "[protocol]")
{
  // Find a line whose checksum contains a hex letter, then lowercase it.
  for (uint32_t seq = 0; seq < 256; ++seq) {
    std::string line = frameLine("TRG,1," + std::to_string(seq) + ",0,0,1,FFF");
    const std::size_t star = line.find('*');
    const auto is_letter = [](char c) { return c >= 'A' && c <= 'F'; };
    if (!is_letter(line[star + 1]) && !is_letter(line[star + 2])) {
      continue;
    }
    for (std::size_t i = star + 1; i <= star + 2; ++i) {
      line[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(line[i])));
    }
    CHECK(std::holds_alternative<TriggerReport>(parse(line)));
    return;
  }
  FAIL("no test line with a letter in its checksum");
}

TEST_CASE("Framing errors", "[protocol]")
{
  const std::string good = frameLine("TRG,1,1234,61,14,1,FFF");
  const std::string unterminated = good.substr(0, good.size() - 2);

  CHECK(errorFor(unterminated) == ParseError::kBadFraming);
  CHECK(errorFor(unterminated + "\n") == ParseError::kBadFraming);
  CHECK(errorFor("") == ParseError::kBadFraming);
  CHECK(errorFor("\r\n") == ParseError::kBadFraming);
  CHECK(errorFor("TRG,1,1234,61,14,1,FFF\r\n") == ParseError::kBadFraming);  // no checksum
  CHECK(errorFor("TRG,1,1234,61,14,1,FFF*5\r\n") == ParseError::kBadFraming);  // one digit
  CHECK(errorFor("TRG,1,1234,61,14,1,FFF*5G\r\n") == ParseError::kBadFraming);  // not hex
  CHECK(errorFor("TRG,1,1234,61,14,1,FFF*5C5\r\n") == ParseError::kBadFraming);  // three digits
}

TEST_CASE("Checksum mismatch is rejected", "[protocol]")
{
  std::string line = frameLine("TRG,1,1234,61,14,1,FFF");
  line[6] = (line[6] == '9') ? '8' : '9';  // corrupt one digit of seq
  CHECK(errorFor(line) == ParseError::kBadChecksum);
}

TEST_CASE("Non-ASCII bytes are rejected", "[protocol]")
{
  CHECK(errorFor(frameLine("TRG,1,1234,61,14,1,FF\x80")) == ParseError::kNonAscii);
  CHECK(errorFor(frameLine(std::string("TRG,1,12\0,61,14,1,FFF", 21))) == ParseError::kNonAscii);
}

TEST_CASE("Line length limit", "[protocol]")
{
  // "BOOT,1," + version + "*XX\r\n" = 7 + n + 5 bytes
  const std::string at_limit = frameLine("BOOT,1," + std::string(kMaxLineLength - 12, 'v'));
  REQUIRE(at_limit.size() == kMaxLineLength);
  CHECK(std::holds_alternative<BootReport>(parse(at_limit)));

  const std::string too_long = frameLine("BOOT,1," + std::string(kMaxLineLength - 11, 'v'));
  REQUIRE(too_long.size() == kMaxLineLength + 1);
  CHECK(errorFor(too_long) == ParseError::kTooLong);
}

TEST_CASE("Unknown line types are reported separately", "[protocol]")
{
  CHECK(errorFor(frameLine("STAT,1,120,LOCKED,15,0,0,0,0")) == ParseError::kUnknownType);
  CHECK(errorFor(frameLine("HELLO")) == ParseError::kUnknownType);
  CHECK(errorFor(frameLine("trg,1,1234,61,14,1,FFF")) == ParseError::kUnknownType);  // case-sensitive
  CHECK(errorFor(frameLine("")) == ParseError::kBadField);  // empty type
}

TEST_CASE("Protocol version", "[protocol]")
{
  CHECK(errorFor(frameLine("TRG,2,1234,61,14,1,FFF")) == ParseError::kBadVersion);
  CHECK(errorFor(frameLine("TRG,2,1234,61,14,1,FFF,extra")) == ParseError::kBadVersion);
  CHECK(errorFor(frameLine("BOOT,0,v1")) == ParseError::kBadVersion);
  CHECK(errorFor(frameLine("TRG,x,1234,61,14,1,FFF")) == ParseError::kBadField);
  CHECK(errorFor(frameLine("TRG")) == ParseError::kBadField);
}

TEST_CASE("Bad TRG fields", "[protocol]")
{
  SECTION("field count")
  {
    CHECK(errorFor(frameLine("TRG,1,1234,61,14,1")) == ParseError::kBadField);
    CHECK(errorFor(frameLine("TRG,1,1234,61,14,1,FFF,0")) == ParseError::kBadField);
  }
  SECTION("numbers")
  {
    CHECK(errorFor(frameLine("TRG,1,4294967296,61,14,1,FFF")) == ParseError::kBadField);  // overflow
    CHECK(errorFor(frameLine("TRG,1,-1,61,14,1,FFF")) == ParseError::kBadField);
    CHECK(errorFor(frameLine("TRG,1,+1,61,14,1,FFF")) == ParseError::kBadField);
    CHECK(errorFor(frameLine("TRG,1, 1,61,14,1,FFF")) == ParseError::kBadField);
    CHECK(errorFor(frameLine("TRG,1,,61,14,1,FFF")) == ParseError::kBadField);
    CHECK(errorFor(frameLine("TRG,1,12a,61,14,1,FFF")) == ParseError::kBadField);
    CHECK(errorFor(frameLine("TRG,1,0x10,61,14,1,FFF")) == ParseError::kBadField);
  }
  SECTION("pulse index must be below pulses_per_second")
  {
    CHECK(errorFor(frameLine("TRG,1,1234,61,20,1,FFF")) == ParseError::kBadField);
    CHECK(std::holds_alternative<TriggerReport>(parseLine(frameLine("TRG,1,1234,61,29,1,FFF"), 30)));
  }
  SECTION("pps_locked")
  {
    CHECK(errorFor(frameLine("TRG,1,1234,61,14,2,FFF")) == ParseError::kBadField);
    CHECK(errorFor(frameLine("TRG,1,1234,61,14,,FFF")) == ParseError::kBadField);
    CHECK(errorFor(frameLine("TRG,1,1234,61,14,true,FFF")) == ParseError::kBadField);
  }
  SECTION("strobe mask")
  {
    CHECK(errorFor(frameLine("TRG,1,1234,61,14,1,FF")) == ParseError::kBadField);
    CHECK(errorFor(frameLine("TRG,1,1234,61,14,1,1000")) == ParseError::kBadField);
    CHECK(errorFor(frameLine("TRG,1,1234,61,14,1,FFG")) == ParseError::kBadField);
    CHECK(errorFor(frameLine("TRG,1,1234,61,14,1,--")) == ParseError::kBadField);
    CHECK(errorFor(frameLine("TRG,1,1234,61,14,1,")) == ParseError::kBadField);
  }
}

TEST_CASE("Bad BOOT fields", "[protocol]")
{
  CHECK(errorFor(frameLine("BOOT,1,")) == ParseError::kBadField);
  CHECK(errorFor(frameLine("BOOT,1")) == ParseError::kBadField);
  CHECK(errorFor(frameLine("BOOT,1,v1,extra")) == ParseError::kBadField);
}

TEST_CASE("Truncated lines and garbage never parse", "[protocol]")
{
  const std::string good = frameLine("TRG,1,1234,61,14,1,FFF");
  // Every tail of a valid line (what arrives after a reconnect mid-line) must be rejected.
  for (std::size_t start = 1; start + 2 < good.size(); ++start) {
    INFO("tail starting at " << start << ": " << good.substr(start));
    CHECK(std::holds_alternative<ParseError>(parse(good.substr(start))));
  }
  CHECK(std::holds_alternative<ParseError>(parse("\x01\x02garbage\xff\r\n")));
  CHECK(std::holds_alternative<ParseError>(parse("*00\r\n")));
}

TEST_CASE("Every error has a name", "[protocol]")
{
  for (const ParseError error :
       {ParseError::kTooLong,
        ParseError::kBadFraming,
        ParseError::kNonAscii,
        ParseError::kBadChecksum,
        ParseError::kUnknownType,
        ParseError::kBadVersion,
        ParseError::kBadField})
  {
    CHECK(std::string(camera_trigger::toString(error)) != "unknown_error");
  }
}
