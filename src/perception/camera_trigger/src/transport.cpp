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

#include "camera_trigger/transport.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <string>
#include <utility>

#include "camera_trigger/protocol.hpp"

namespace camera_trigger
{

namespace
{

// Big enough for any datagram we accept plus margin, so MSG_TRUNC only fires for real junk.
constexpr std::size_t kReceiveBufferSize = 1024;
constexpr std::size_t kControlBufferSize = 128;

std::string errnoText(const std::string & what)
{
  return what + ": " + std::strerror(errno);
}

std::string endpointText(uint32_t addr_be, uint16_t port_be)
{
  std::array<char, INET_ADDRSTRLEN> text{};
  in_addr addr{};
  addr.s_addr = addr_be;
  if (inet_ntop(AF_INET, &addr, text.data(), text.size()) == nullptr) {
    return "?";
  }
  return std::string(text.data()) + ":" + std::to_string(ntohs(port_be));
}

}  // namespace

bool isValidIpv4(const std::string & text)
{
  in_addr addr{};
  return inet_pton(AF_INET, text.c_str(), &addr) == 1;
}

int64_t systemTimeNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
    .count();
}

UdpTransport::UdpTransport(UdpConfig config)
: config_(std::move(config))
{}

UdpTransport::~UdpTransport()
{
  close();
}

bool UdpTransport::open(std::string & error)
{
  close();

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(config_.port);
  if (inet_pton(AF_INET, config_.bind_address.c_str(), &addr.sin_addr) != 1) {
    error = "invalid bind address '" + config_.bind_address + "'";
    return false;
  }

  filter_source_ = !config_.source_ip.empty();
  if (filter_source_) {
    in_addr source{};
    if (inet_pton(AF_INET, config_.source_ip.c_str(), &source) != 1) {
      error = "invalid source IP '" + config_.source_ip + "'";
      return false;
    }
    source_addr_ = source.s_addr;
  }

  fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd_ < 0) {
    error = errnoText("socket()");
    return false;
  }

  // Ask the kernel to stamp each datagram on arrival. Not fatal if unsupported: read() then falls
  // back to reading the clock, and reports that through ReceivedLine::kernel_timestamp.
  const int enable = 1;
  ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &enable, sizeof(enable));

  // No SO_REUSEADDR: a second process on the same port must fail loudly instead of silently
  // sharing (and stealing) the reports.
  if (::bind(fd_, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) != 0) {
    error = errnoText("bind(" + config_.bind_address + ":" + std::to_string(config_.port) + ")");
    close();
    return false;
  }

  sockaddr_in bound{};
  socklen_t bound_len = sizeof(bound);
  if (::getsockname(fd_, reinterpret_cast<sockaddr *>(&bound), &bound_len) == 0) {
    bound_port_ = ntohs(bound.sin_port);
  } else {
    bound_port_ = config_.port;
  }
  return true;
}

void UdpTransport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  bound_port_ = 0;
}

bool UdpTransport::isOpen() const
{
  return fd_ >= 0;
}

ReadStatus UdpTransport::read(ReceivedLine & line, int timeout_ms)
{
  if (fd_ < 0) {
    return ReadStatus::kError;
  }

  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  const int ready = ::poll(&pfd, 1, timeout_ms);
  if (ready == 0 || (ready < 0 && errno == EINTR)) {
    return ReadStatus::kTimeout;
  }
  if (ready < 0 || (pfd.revents & (POLLERR | POLLNVAL)) != 0) {
    return ReadStatus::kError;
  }

  std::array<char, kReceiveBufferSize> buffer{};
  alignas(cmsghdr) std::array<char, kControlBufferSize> control{};
  sockaddr_in sender{};
  iovec iov{};
  iov.iov_base = buffer.data();
  iov.iov_len = buffer.size();
  msghdr msg{};
  msg.msg_name = &sender;
  msg.msg_namelen = sizeof(sender);
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = control.data();
  msg.msg_controllen = control.size();

  // MSG_TRUNC: return the real datagram length even if it didn't fit in the buffer.
  const ssize_t received = ::recvmsg(fd_, &msg, MSG_DONTWAIT | MSG_TRUNC);
  if (received < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return ReadStatus::kTimeout;
    }
    return ReadStatus::kError;
  }

  line.kernel_timestamp = false;
  for (cmsghdr * cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
    if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_TIMESTAMPNS) {
      timespec stamp{};
      std::memcpy(&stamp, CMSG_DATA(cmsg), sizeof(stamp));
      line.t_rx_ns = static_cast<int64_t>(stamp.tv_sec) * 1000000000 + stamp.tv_nsec;
      line.kernel_timestamp = true;
    }
  }
  if (!line.kernel_timestamp) {
    line.t_rx_ns = systemTimeNs();
  }

  line.source = endpointText(sender.sin_addr.s_addr, sender.sin_port);
  if (filter_source_ && sender.sin_addr.s_addr != source_addr_) {
    return ReadStatus::kForeignSender;
  }

  const auto length = static_cast<std::size_t>(received);
  if (length > kMaxLineLength || (msg.msg_flags & MSG_TRUNC) != 0) {
    return ReadStatus::kTooLong;
  }
  line.text.assign(buffer.data(), length);
  return ReadStatus::kLine;
}

std::string UdpTransport::describe() const
{
  const uint16_t port = isOpen() ? bound_port_ : config_.port;
  return "udp " + config_.bind_address + ":" + std::to_string(port);
}

}  // namespace camera_trigger
