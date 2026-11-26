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
#include "cs_test.h"

static const uint8_t kBoardMac[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
static const uint8_t kHostMac[6] = {0x3c, 0xec, 0xef, 0x01, 0x02, 0x03};

static cs_net_t make_net(void)
{
  cs_net_cfg_t cfg = {
    .mac = {0}, .ip = CS_IP4(10, 8, 0, 60), .dest_ip = CS_IP4(10, 8, 0, 1), .dest_port = 5800, .src_port = 5800};
  memcpy(cfg.mac, kBoardMac, 6);
  cs_net_t net;
  cs_net_init(&net, &cfg);
  return net;
}

static uint16_t rd16(const uint8_t * p)
{
  return (uint16_t)((p[0] << 8) | p[1]);
}

// An ARP packet as the computer would send it.
static size_t host_arp(uint8_t * f, uint16_t oper, const uint8_t eth_dst[6], uint32_t tpa)
{
  memset(f, 0, 60);
  memcpy(f, eth_dst, 6);
  memcpy(f + 6, kHostMac, 6);
  f[12] = 0x08;
  f[13] = 0x06;
  uint8_t * a = f + 14;
  a[1] = 1;
  a[2] = 0x08;
  a[4] = 6;
  a[5] = 4;
  a[7] = (uint8_t)oper;
  memcpy(a + 8, kHostMac, 6);
  a[14] = 10;
  a[15] = 8;
  a[16] = 0;
  a[17] = 1;
  a[24] = (uint8_t)(tpa >> 24);
  a[25] = (uint8_t)(tpa >> 16);
  a[26] = (uint8_t)(tpa >> 8);
  a[27] = (uint8_t)tpa;
  return 60;
}

static void test_no_udp_before_arp(void)
{
  cs_net_t net = make_net();
  uint8_t out[CS_NET_MAX_FRAME];
  CHECK(!cs_net_ready(&net));
  CHECK_EQ_INT(cs_net_build_udp(&net, (const uint8_t *)"x", 1, out, sizeof(out)), 0);
}

static void test_arp_request_format(void)
{
  cs_net_t net = make_net();
  uint8_t f[CS_NET_MAX_FRAME];
  const size_t n = cs_net_build_arp_request(&net, f, sizeof(f));
  CHECK_EQ_INT(n, 60);
  for (int i = 0; i < 6; ++i) {
    CHECK_EQ_INT(f[i], 0xFF);
  }
  CHECK(memcmp(f + 6, kBoardMac, 6) == 0);
  CHECK_EQ_INT(rd16(f + 12), 0x0806);
  CHECK_EQ_INT(rd16(f + 14 + 6), 1);  // request
  CHECK(f[14 + 24] == 10 && f[14 + 25] == 8 && f[14 + 26] == 0 && f[14 + 27] == 1);
}

static void test_learns_mac_from_reply_and_sends_udp(void)
{
  cs_net_t net = make_net();
  uint8_t in[64];
  uint8_t out[CS_NET_MAX_FRAME];
  host_arp(in, 2, kBoardMac, CS_IP4(10, 8, 0, 60));
  CHECK_EQ_INT(cs_net_handle_frame(&net, in, 60, out, sizeof(out)), 0);
  CHECK(cs_net_ready(&net));

  const char * line = "TRG,1,1234,61,14,1,FFF*01\r\n";
  const size_t len = strlen(line);
  const size_t n = cs_net_build_udp(&net, (const uint8_t *)line, len, out, sizeof(out));
  CHECK_EQ_INT(n, 14 + 20 + 8 + len);
  CHECK(memcmp(out, kHostMac, 6) == 0);
  CHECK(memcmp(out + 6, kBoardMac, 6) == 0);
  CHECK_EQ_INT(rd16(out + 12), 0x0800);
  const uint8_t * ip = out + 14;
  CHECK_EQ_INT(ip[0], 0x45);
  CHECK_EQ_INT(rd16(ip + 2), 20 + 8 + len);
  CHECK_EQ_INT(ip[9], 17);
  CHECK_EQ_INT(cs_net_checksum(ip, 20, 0), 0);  // header checksum valid
  const uint8_t * udp = ip + 20;
  CHECK_EQ_INT(rd16(udp + 0), 5800);
  CHECK_EQ_INT(rd16(udp + 2), 5800);
  CHECK_EQ_INT(rd16(udp + 4), 8 + len);
  CHECK(memcmp(udp + 8, line, len) == 0);
  // UDP checksum valid over pseudo-header + datagram.
  uint32_t sum = 0x0A08 + 0x003C + 0x0A08 + 0x0001 + 17 + (uint32_t)(8 + len);
  CHECK_EQ_INT(cs_net_checksum(udp, 8 + len, sum), 0);
}

static void test_answers_arp_request_and_learns_requester(void)
{
  cs_net_t net = make_net();
  static const uint8_t kBcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t in[64];
  uint8_t out[CS_NET_MAX_FRAME];
  host_arp(in, 1, kBcast, CS_IP4(10, 8, 0, 60));
  const size_t n = cs_net_handle_frame(&net, in, 60, out, sizeof(out));
  CHECK_EQ_INT(n, 60);
  CHECK(memcmp(out, kHostMac, 6) == 0);
  CHECK_EQ_INT(rd16(out + 14 + 6), 2);  // reply
  CHECK(memcmp(out + 14 + 8, kBoardMac, 6) == 0);
  CHECK(cs_net_ready(&net));  // the computer asked, so its MAC is known

  // A request for another address is not answered.
  cs_net_t other = make_net();
  host_arp(in, 1, kBcast, CS_IP4(10, 8, 0, 61));
  CHECK_EQ_INT(cs_net_handle_frame(&other, in, 60, out, sizeof(out)), 0);
}

static void test_answers_ping(void)
{
  cs_net_t net = make_net();
  uint8_t in[128] = {0};
  uint8_t out[CS_NET_MAX_FRAME];
  memcpy(in, kBoardMac, 6);
  memcpy(in + 6, kHostMac, 6);
  in[12] = 0x08;
  uint8_t * ip = in + 14;
  const size_t icmp_len = 8 + 32;
  ip[0] = 0x45;
  ip[2] = 0;
  ip[3] = (uint8_t)(20 + icmp_len);
  ip[8] = 64;
  ip[9] = 1;
  ip[12] = 10;
  ip[13] = 8;
  ip[14] = 0;
  ip[15] = 1;
  ip[16] = 10;
  ip[17] = 8;
  ip[18] = 0;
  ip[19] = 60;
  uint16_t c = cs_net_checksum(ip, 20, 0);
  ip[10] = (uint8_t)(c >> 8);
  ip[11] = (uint8_t)c;
  uint8_t * icmp = ip + 20;
  icmp[0] = 8;
  icmp[4] = 0x12;
  icmp[5] = 0x34;
  icmp[7] = 1;
  for (size_t i = 8; i < icmp_len; ++i) {
    icmp[i] = (uint8_t)i;
  }
  c = cs_net_checksum(icmp, icmp_len, 0);
  icmp[2] = (uint8_t)(c >> 8);
  icmp[3] = (uint8_t)c;

  const size_t n = cs_net_handle_frame(&net, in, 14 + 20 + icmp_len, out, sizeof(out));
  CHECK_EQ_INT(n, 14 + 20 + icmp_len);
  CHECK(memcmp(out, kHostMac, 6) == 0);
  const uint8_t * rip = out + 14;
  CHECK_EQ_INT(cs_net_checksum(rip, 20, 0), 0);
  CHECK(rip[16] == 10 && rip[19] == 1);  // back to the sender
  const uint8_t * ricmp = rip + 20;
  CHECK_EQ_INT(ricmp[0], 0);
  CHECK_EQ_INT(cs_net_checksum(ricmp, icmp_len, 0), 0);
  CHECK(memcmp(ricmp + 4, icmp + 4, icmp_len - 4) == 0);

  // A corrupted echo request is ignored.
  icmp[9] ^= 0xFF;
  CHECK_EQ_INT(cs_net_handle_frame(&net, in, 14 + 20 + icmp_len, out, sizeof(out)), 0);
}

static void test_ignores_other_traffic(void)
{
  cs_net_t net = make_net();
  uint8_t in[64] = {0};
  uint8_t out[CS_NET_MAX_FRAME];
  memcpy(in, kHostMac, 6);  // not for us
  in[12] = 0x08;
  CHECK_EQ_INT(cs_net_handle_frame(&net, in, 60, out, sizeof(out)), 0);
  CHECK_EQ_INT(cs_net_handle_frame(&net, in, 5, out, sizeof(out)), 0);
  CHECK_EQ_INT(net.rx_ignored, 2);
}

static void test_mac_from_uid(void)
{
  const uint32_t a[3] = {0x00400030, 0x3438510B, 0x37383532};
  const uint32_t b[3] = {0x00400030, 0x3438510B, 0x37383533};
  uint8_t ma[6];
  uint8_t mb[6];
  cs_net_mac_from_uid(a, ma);
  cs_net_mac_from_uid(b, mb);
  CHECK_EQ_INT(ma[0], 0x02);  // locally administered unicast
  CHECK(memcmp(ma, mb, 6) != 0);
  uint8_t again[6];
  cs_net_mac_from_uid(a, again);
  CHECK(memcmp(ma, again, 6) == 0);
}

int main(void)
{
  RUN(test_no_udp_before_arp);
  RUN(test_arp_request_format);
  RUN(test_learns_mac_from_reply_and_sends_udp);
  RUN(test_answers_arp_request_and_learns_requester);
  RUN(test_answers_ping);
  RUN(test_ignores_other_traffic);
  RUN(test_mac_from_uid);
  TEST_MAIN_END();
}
