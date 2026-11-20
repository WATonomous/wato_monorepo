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

#include <cstdint>
#include <string>

// Where report lines come from. No ROS dependencies.
// UDP is the only transport for now; a serial fallback can be added as another Transport
// (see docs/open_items.md, F1) without touching the parser, the timing or the node logic.

namespace camera_trigger
{

/// One received report line.
struct ReceivedLine
{
  std::string text;  ///< the raw line, terminator included
  int64_t t_rx_ns{0};  ///< system-clock (CLOCK_REALTIME) receive time
  bool kernel_timestamp{false};  ///< t_rx_ns came from the kernel, not from reading the clock afterwards
  std::string source;  ///< sender, e.g. "10.8.0.50:41234"
};

enum class ReadStatus
{
  kLine,  ///< `line` holds one report line
  kTimeout,  ///< nothing arrived in time
  kTooLong,  ///< a datagram longer than kMaxLineLength was dropped
  kForeignSender,  ///< a datagram from a sender other than the configured source was dropped
  kError,  ///< the socket failed: close() and open() again
};

class Transport
{
public:
  virtual ~Transport() = default;

  /// Opens the link. On failure returns false and sets `error`.
  virtual bool open(std::string & error) = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;

  /// Waits up to `timeout_ms` for the next line. `line.source` is set for kLine and kForeignSender.
  virtual ReadStatus read(ReceivedLine & line, int timeout_ms) = 0;

  /// Endpoint for logs and diagnostics, e.g. "udp 0.0.0.0:5800".
  virtual std::string describe() const = 0;
};

struct UdpConfig
{
  std::string bind_address{"0.0.0.0"};  ///< local IPv4 address to listen on
  uint16_t port{5800};  ///< 0 picks a free port (tests)
  std::string source_ip;  ///< if set, datagrams from any other IPv4 address are dropped
};

/// Receives one report line per UDP datagram, with the kernel's receive timestamp.
class UdpTransport : public Transport
{
public:
  explicit UdpTransport(UdpConfig config);
  ~UdpTransport() override;

  UdpTransport(const UdpTransport &) = delete;
  UdpTransport & operator=(const UdpTransport &) = delete;

  bool open(std::string & error) override;
  void close() override;
  bool isOpen() const override;
  ReadStatus read(ReceivedLine & line, int timeout_ms) override;
  std::string describe() const override;

  /// Port actually bound (differs from the configured one when that was 0). 0 if not open.
  uint16_t boundPort() const
  {
    return bound_port_;
  }

private:
  UdpConfig config_;
  int fd_{-1};
  uint16_t bound_port_{0};
  bool filter_source_{false};
  uint32_t source_addr_{0};  ///< network byte order
};

/// True if `text` is a dotted-quad IPv4 address such as "10.8.0.1".
bool isValidIpv4(const std::string & text);

/// Current system-clock time in ns since the Unix epoch.
int64_t systemTimeNs();

}  // namespace camera_trigger
