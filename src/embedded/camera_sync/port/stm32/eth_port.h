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

// Ethernet MAC + LAN8742A PHY, polled from the main loop (no Ethernet interrupt, so the network
// can never delay a trigger pulse). Same code for F7 and H7: both HALs have the same ETH API.

#ifndef ETH_PORT_H_
#define ETH_PORT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cs/cs_net.h"

#ifdef __cplusplus
extern "C"
{
#endif

  bool cs_eth_init(const uint8_t mac[6]);
  // Checks the PHY link; starts or stops the MAC on a change. Returns true while the link is up
  // and the MAC is running.
  bool cs_eth_poll_link(void);
  bool cs_eth_send(const uint8_t * frame, size_t len);
  // Reads up to a few received frames and lets `net` answer them (ARP, ping).
  void cs_eth_poll_rx(cs_net_t * net);

#ifdef __cplusplus
}
#endif

#endif  // ETH_PORT_H_
