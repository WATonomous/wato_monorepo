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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "camera_trigger/protocol.hpp"
#include "camera_trigger/transport.hpp"

using camera_trigger::frameLine;
using camera_trigger::isValidIpv4;
using camera_trigger::kMaxLineLength;
using camera_trigger::ReadStatus;
using camera_trigger::ReceivedLine;
using camera_trigger::systemTimeNs;
using camera_trigger::UdpConfig;
using camera_trigger::UdpTransport;

namespace
{

constexpr int64_t kMs = 1000000;

// Sends one datagram from `from_ip` (any free port) to 127.0.0.1:to_port. Returns the sender's port.
uint16_t sendFrom(const std::string & from_ip, uint16_t to_port, const std::string & payload)
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  REQUIRE(fd >= 0);

  sockaddr_in local{};
  local.sin_family = AF_INET;
  REQUIRE(inet_pton(AF_INET, from_ip.c_str(), &local.sin_addr) == 1);
  REQUIRE(::bind(fd, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) == 0);

  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(to_port);
  REQUIRE(inet_pton(AF_INET, "127.0.0.1", &dest.sin_addr) == 1);
  const ssize_t sent =
    ::sendto(fd, payload.data(), payload.size(), 0, reinterpret_cast<const sockaddr *>(&dest), sizeof(dest));
  REQUIRE(sent == static_cast<ssize_t>(payload.size()));

  sockaddr_in bound{};
  socklen_t bound_len = sizeof(bound);
  REQUIRE(::getsockname(fd, reinterpret_cast<sockaddr *>(&bound), &bound_len) == 0);
  ::close(fd);
  return ntohs(bound.sin_port);
}

UdpConfig loopback(const std::string & source_ip = "")
{
  UdpConfig config;
  config.bind_address = "127.0.0.1";
  config.port = 0;  // any free port
  config.source_ip = source_ip;
  return config;
}

}  // namespace

TEST_CASE("One datagram is one line, with the kernel receive time", "[transport]")
{
  UdpTransport transport(loopback());
  std::string error;
  REQUIRE(transport.open(error));
  REQUIRE(transport.isOpen());
  REQUIRE(transport.boundPort() != 0);
  CHECK(transport.describe() == "udp 127.0.0.1:" + std::to_string(transport.boundPort()));

  const std::string line = frameLine("TRG,1,1,0,1,1,FFF");
  const int64_t before = systemTimeNs();
  const uint16_t sender_port = sendFrom("127.0.0.1", transport.boundPort(), line);

  ReceivedLine received;
  REQUIRE(transport.read(received, 1000) == ReadStatus::kLine);
  const int64_t after = systemTimeNs();

  CHECK(received.text == line);
  CHECK(received.source == "127.0.0.1:" + std::to_string(sender_port));
  CHECK(received.kernel_timestamp);
  CHECK(received.t_rx_ns >= before - kMs);
  CHECK(received.t_rx_ns <= after);
}

TEST_CASE("Nothing to read times out", "[transport]")
{
  UdpTransport transport(loopback());
  std::string error;
  REQUIRE(transport.open(error));
  ReceivedLine received;
  CHECK(transport.read(received, 20) == ReadStatus::kTimeout);
}

TEST_CASE("Oversize datagrams are dropped", "[transport]")
{
  UdpTransport transport(loopback());
  std::string error;
  REQUIRE(transport.open(error));
  ReceivedLine received;

  sendFrom("127.0.0.1", transport.boundPort(), std::string(kMaxLineLength, 'x'));
  CHECK(transport.read(received, 1000) == ReadStatus::kLine);
  CHECK(received.text.size() == kMaxLineLength);

  sendFrom("127.0.0.1", transport.boundPort(), std::string(kMaxLineLength + 1, 'x'));
  CHECK(transport.read(received, 1000) == ReadStatus::kTooLong);

  // Bigger than the 1024-byte receive buffer, so it arrives truncated. Kept under 1500 bytes:
  // some environments (e.g. WSL2) drop larger datagrams even on loopback.
  sendFrom("127.0.0.1", transport.boundPort(), std::string(1200, 'x'));
  CHECK(transport.read(received, 1000) == ReadStatus::kTooLong);
}

TEST_CASE("Only the configured sender is accepted", "[transport]")
{
  UdpTransport transport(loopback("127.0.0.2"));
  std::string error;
  REQUIRE(transport.open(error));
  const std::string line = frameLine("TRG,1,1,0,1,1,FFF");
  ReceivedLine received;

  sendFrom("127.0.0.1", transport.boundPort(), line);
  CHECK(transport.read(received, 1000) == ReadStatus::kForeignSender);
  CHECK(received.source.rfind("127.0.0.1:", 0) == 0);

  sendFrom("127.0.0.2", transport.boundPort(), line);
  CHECK(transport.read(received, 1000) == ReadStatus::kLine);
  CHECK(received.text == line);
}

TEST_CASE("Bad addresses and busy ports fail to open with a message", "[transport]")
{
  std::string error;

  UdpConfig bad_bind = loopback();
  bad_bind.bind_address = "10.8.0";
  UdpTransport bad_bind_transport(bad_bind);
  CHECK_FALSE(bad_bind_transport.open(error));
  CHECK(error.find("invalid bind address") != std::string::npos);

  UdpTransport bad_source_transport(loopback("not-an-ip"));
  CHECK_FALSE(bad_source_transport.open(error));
  CHECK(error.find("invalid source IP") != std::string::npos);

  UdpTransport first(loopback());
  REQUIRE(first.open(error));
  UdpConfig same_port = loopback();
  same_port.port = first.boundPort();
  UdpTransport second(same_port);
  CHECK_FALSE(second.open(error));
  CHECK(error.find("bind(") != std::string::npos);
  CHECK_FALSE(second.isOpen());
}

TEST_CASE("Reading a closed transport reports an error", "[transport]")
{
  UdpTransport transport(loopback());
  ReceivedLine received;
  CHECK(transport.read(received, 10) == ReadStatus::kError);

  std::string error;
  REQUIRE(transport.open(error));
  transport.close();
  CHECK_FALSE(transport.isOpen());
  CHECK(transport.read(received, 10) == ReadStatus::kError);
}

TEST_CASE("IPv4 address validation", "[transport]")
{
  CHECK(isValidIpv4("10.8.0.1"));
  CHECK(isValidIpv4("0.0.0.0"));
  CHECK_FALSE(isValidIpv4(""));
  CHECK_FALSE(isValidIpv4("10.8.0"));
  CHECK_FALSE(isValidIpv4("10.8.0.256"));
  CHECK_FALSE(isValidIpv4("localhost"));
}
