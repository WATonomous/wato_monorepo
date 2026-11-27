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

// Minimal IPv4 stack for sending report lines: just enough to be a good citizen on the sensor
// network, without an RTOS or LwIP.
//
//  - UDP send to one destination (the computer), one datagram per report line.
//  - ARP: asks for the destination's MAC address, answers requests for our own address, and
//    learns the destination MAC from any ARP packet it sends.
//  - ICMP echo: answers ping, so `ping <board IP>` works when bringing the board up.
//
// Static addressing on one subnet (no DHCP, no gateway). Frames are built in caller buffers;
// the port only moves bytes to and from the Ethernet MAC. Portable C, unit-tested on the host.

#ifndef CS_NET_H_
#define CS_NET_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define CS_IP4(a, b, c, d) \
  ((uint32_t)(((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (uint32_t)(d)))

#define CS_NET_MAX_FRAME 1514u  // Ethernet frame without FCS
#define CS_NET_MIN_FRAME 60u  // shorter frames are zero-padded

  typedef struct
  {
    uint8_t mac[6];
    uint32_t ip;  // host byte order, CS_IP4(...)
    uint32_t dest_ip;
    uint16_t dest_port;
    uint16_t src_port;
  } cs_net_cfg_t;

  typedef struct
  {
    cs_net_cfg_t cfg;
    uint8_t dest_mac[6];
    bool dest_mac_valid;
    uint16_t ip_id;
    uint32_t rx_arp;
    uint32_t rx_icmp;
    uint32_t rx_ignored;
  } cs_net_t;

  void cs_net_init(cs_net_t * net, const cs_net_cfg_t * cfg);

  // True once the destination MAC address is known.
  static inline bool cs_net_ready(const cs_net_t * net)
  {
    return net->dest_mac_valid;
  }

  // Forget the destination MAC (link went down), so it is resolved again.
  void cs_net_forget(cs_net_t * net);

  // Builds an ARP request for the destination address. Returns the frame length.
  size_t cs_net_build_arp_request(const cs_net_t * net, uint8_t * out, size_t cap);

  // Builds a UDP datagram to the destination. Returns 0 if the destination MAC is not known yet or
  // the payload does not fit.
  size_t cs_net_build_udp(cs_net_t * net, const uint8_t * payload, size_t len, uint8_t * out, size_t cap);

  // Handles one received frame. Returns the length of a reply to send (ARP reply or ICMP echo
  // reply, written to `out`), or 0 if nothing has to be sent.
  size_t cs_net_handle_frame(cs_net_t * net, const uint8_t * in, size_t len, uint8_t * out, size_t cap);

  // Internet checksum (RFC 1071) over `len` bytes, starting from a partial sum.
  uint16_t cs_net_checksum(const uint8_t * data, size_t len, uint32_t sum);

  // Locally administered unicast MAC address derived from the MCU's 96-bit unique ID, so every
  // board gets its own stable address without configuration.
  void cs_net_mac_from_uid(const uint32_t uid[3], uint8_t mac[6]);

#ifdef __cplusplus
}
#endif

#endif  // CS_NET_H_
