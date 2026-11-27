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

#include "cs/cs_net.h"

#include <string.h>

enum
{
  ETH_HDR = 14,
  ARP_LEN = 28,
  IP_HDR = 20,
  UDP_HDR = 8,
  ICMP_HDR = 8,
  ETHERTYPE_IPV4 = 0x0800,
  ETHERTYPE_ARP = 0x0806,
  IP_PROTO_ICMP = 1,
  IP_PROTO_UDP = 17,
  ARP_REQUEST = 1,
  ARP_REPLY = 2,
  ICMP_ECHO_REPLY = 0,
  ICMP_ECHO_REQUEST = 8,
};

static const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint16_t rd16(const uint8_t * p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t rd32(const uint8_t * p)
{
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static void wr16(uint8_t * p, uint16_t v)
{
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)v;
}

static void wr32(uint8_t * p, uint32_t v)
{
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)v;
}

static size_t pad(uint8_t * out, size_t len, size_t cap)
{
  if (len < CS_NET_MIN_FRAME && cap >= CS_NET_MIN_FRAME) {
    memset(out + len, 0, CS_NET_MIN_FRAME - len);
    return CS_NET_MIN_FRAME;
  }
  return len;
}

static void write_eth(uint8_t * out, const uint8_t dst[6], const uint8_t src[6], uint16_t type)
{
  memcpy(out, dst, 6);
  memcpy(out + 6, src, 6);
  wr16(out + 12, type);
}

static void write_ip(uint8_t * ip, uint16_t total_len, uint16_t id, uint8_t proto, uint32_t src, uint32_t dst)
{
  ip[0] = 0x45;  // IPv4, 20-byte header
  ip[1] = 0;
  wr16(ip + 2, total_len);
  wr16(ip + 4, id);
  wr16(ip + 6, 0x4000);  // don't fragment
  ip[8] = 64;  // TTL
  ip[9] = proto;
  wr16(ip + 10, 0);
  wr32(ip + 12, src);
  wr32(ip + 16, dst);
  wr16(ip + 10, cs_net_checksum(ip, IP_HDR, 0));
}

uint16_t cs_net_checksum(const uint8_t * data, size_t len, uint32_t sum)
{
  size_t i = 0;
  for (; i + 1u < len; i += 2u) {
    sum += rd16(data + i);
  }
  if (i < len) {
    sum += (uint32_t)data[i] << 8;
  }
  while ((sum >> 16) != 0u) {
    sum = (sum & 0xFFFFu) + (sum >> 16);
  }
  return (uint16_t)~sum;
}

void cs_net_init(cs_net_t * net, const cs_net_cfg_t * cfg)
{
  memset(net, 0, sizeof(*net));
  net->cfg = *cfg;
}

void cs_net_forget(cs_net_t * net)
{
  net->dest_mac_valid = false;
}

static size_t build_arp(
  const cs_net_t * net,
  uint16_t oper,
  const uint8_t eth_dst[6],
  const uint8_t tha[6],
  uint32_t tpa,
  uint8_t * out,
  size_t cap)
{
  if (cap < CS_NET_MIN_FRAME) {
    return 0;
  }
  write_eth(out, eth_dst, net->cfg.mac, ETHERTYPE_ARP);
  uint8_t * a = out + ETH_HDR;
  wr16(a + 0, 1);  // Ethernet
  wr16(a + 2, ETHERTYPE_IPV4);
  a[4] = 6;
  a[5] = 4;
  wr16(a + 6, oper);
  memcpy(a + 8, net->cfg.mac, 6);
  wr32(a + 14, net->cfg.ip);
  memcpy(a + 18, tha, 6);
  wr32(a + 24, tpa);
  return pad(out, ETH_HDR + ARP_LEN, cap);
}

size_t cs_net_build_arp_request(const cs_net_t * net, uint8_t * out, size_t cap)
{
  static const uint8_t kZero[6] = {0};
  return build_arp(net, ARP_REQUEST, kBroadcast, kZero, net->cfg.dest_ip, out, cap);
}

size_t cs_net_build_udp(cs_net_t * net, const uint8_t * payload, size_t len, uint8_t * out, size_t cap)
{
  const size_t frame_len = ETH_HDR + IP_HDR + UDP_HDR + len;
  if (!net->dest_mac_valid || frame_len > CS_NET_MAX_FRAME || frame_len > cap || cap < CS_NET_MIN_FRAME) {
    return 0;
  }
  write_eth(out, net->dest_mac, net->cfg.mac, ETHERTYPE_IPV4);
  uint8_t * ip = out + ETH_HDR;
  write_ip(ip, (uint16_t)(IP_HDR + UDP_HDR + len), net->ip_id++, IP_PROTO_UDP, net->cfg.ip, net->cfg.dest_ip);

  uint8_t * udp = ip + IP_HDR;
  const uint16_t udp_len = (uint16_t)(UDP_HDR + len);
  wr16(udp + 0, net->cfg.src_port);
  wr16(udp + 2, net->cfg.dest_port);
  wr16(udp + 4, udp_len);
  wr16(udp + 6, 0);
  memcpy(udp + UDP_HDR, payload, len);

  // Pseudo-header: source, destination, protocol, UDP length.
  uint32_t sum = (net->cfg.ip >> 16) + (net->cfg.ip & 0xFFFFu) + (net->cfg.dest_ip >> 16) +
                 (net->cfg.dest_ip & 0xFFFFu) + IP_PROTO_UDP + udp_len;
  uint16_t csum = cs_net_checksum(udp, udp_len, sum);
  wr16(udp + 6, csum == 0u ? 0xFFFFu : csum);  // 0 means "no checksum" in UDP over IPv4
  return pad(out, frame_len, cap);
}

static size_t handle_arp(cs_net_t * net, const uint8_t * in, size_t len, uint8_t * out, size_t cap)
{
  if (len < ETH_HDR + ARP_LEN) {
    return 0;
  }
  const uint8_t * a = in + ETH_HDR;
  if (rd16(a + 0) != 1u || rd16(a + 2) != ETHERTYPE_IPV4 || a[4] != 6u || a[5] != 4u) {
    return 0;
  }
  net->rx_arp++;
  const uint16_t oper = rd16(a + 6);
  const uint8_t * sha = a + 8;
  const uint32_t spa = rd32(a + 14);
  const uint32_t tpa = rd32(a + 24);

  if (spa == net->cfg.dest_ip && spa != 0u) {
    memcpy(net->dest_mac, sha, 6);
    net->dest_mac_valid = true;
  }
  if (oper == ARP_REQUEST && tpa == net->cfg.ip) {
    return build_arp(net, ARP_REPLY, sha, sha, spa, out, cap);
  }
  return 0;
}

static size_t handle_ipv4(cs_net_t * net, const uint8_t * in, size_t len, uint8_t * out, size_t cap)
{
  if (len < ETH_HDR + IP_HDR) {
    return 0;
  }
  const uint8_t * ip = in + ETH_HDR;
  const size_t ihl = (size_t)(ip[0] & 0x0Fu) * 4u;
  const size_t total = rd16(ip + 2);
  if ((ip[0] >> 4) != 4u || ihl < IP_HDR || total < ihl + ICMP_HDR || ETH_HDR + total > len) {
    return 0;
  }
  if (rd32(ip + 16) != net->cfg.ip || ip[9] != IP_PROTO_ICMP) {
    return 0;
  }
  if ((rd16(ip + 6) & 0x3FFFu) != 0u || cs_net_checksum(ip, ihl, 0) != 0u) {
    return 0;  // fragment, or bad header checksum
  }
  const uint8_t * icmp = ip + ihl;
  const size_t icmp_len = total - ihl;
  if (icmp[0] != ICMP_ECHO_REQUEST || icmp[1] != 0u || cs_net_checksum(icmp, icmp_len, 0) != 0u) {
    return 0;
  }
  const size_t reply_len = ETH_HDR + IP_HDR + icmp_len;
  if (reply_len > cap || reply_len > CS_NET_MAX_FRAME) {
    return 0;
  }
  net->rx_icmp++;
  write_eth(out, in + 6, net->cfg.mac, ETHERTYPE_IPV4);
  uint8_t * rip = out + ETH_HDR;
  write_ip(rip, (uint16_t)(IP_HDR + icmp_len), net->ip_id++, IP_PROTO_ICMP, net->cfg.ip, rd32(ip + 12));
  uint8_t * ricmp = rip + IP_HDR;
  memcpy(ricmp, icmp, icmp_len);
  ricmp[0] = ICMP_ECHO_REPLY;
  wr16(ricmp + 2, 0);
  wr16(ricmp + 2, cs_net_checksum(ricmp, icmp_len, 0));
  return pad(out, reply_len, cap);
}

size_t cs_net_handle_frame(cs_net_t * net, const uint8_t * in, size_t len, uint8_t * out, size_t cap)
{
  if (len < ETH_HDR) {
    net->rx_ignored++;
    return 0;
  }
  const bool to_us = memcmp(in, net->cfg.mac, 6) == 0 || memcmp(in, kBroadcast, 6) == 0;
  if (!to_us) {
    net->rx_ignored++;
    return 0;
  }
  switch (rd16(in + 12)) {
    case ETHERTYPE_ARP:
      return handle_arp(net, in, len, out, cap);
    case ETHERTYPE_IPV4:
      return handle_ipv4(net, in, len, out, cap);
    default:
      net->rx_ignored++;
      return 0;
  }
}

void cs_net_mac_from_uid(const uint32_t uid[3], uint8_t mac[6])
{
  // FNV-1a over the 12 ID bytes, folded into 40 bits.
  uint64_t h = 0xcbf29ce484222325ull;
  for (int w = 0; w < 3; ++w) {
    for (int b = 0; b < 4; ++b) {
      h ^= (uint8_t)(uid[w] >> (8 * b));
      h *= 0x100000001b3ull;
    }
  }
  h ^= h >> 40;
  mac[0] = 0x02;  // locally administered, unicast
  for (int i = 1; i < 6; ++i) {
    mac[i] = (uint8_t)(h >> (8 * (i - 1)));
  }
}
