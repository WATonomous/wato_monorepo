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

#include "eth_port.h"

#include <string.h>

#include "board.h"
#include "cs_hw.h"

// Everything the Ethernet DMA reads or writes lives in .eth_dma, which each board's linker script
// places in RAM the DMA can reach (H7: D2 SRAM, not the DTCM; F7: SRAM2). The data cache stays
// off, so no cache maintenance is needed.
#define ETH_DMA_MEM __attribute__((section(".eth_dma"), aligned(32)))

#define RX_BUF_SIZE 1536u
#define RX_POOL (ETH_RX_DESC_CNT + 2u)

// LAN8742A registers
#define CS_PHY_BCR 0u
#define CS_PHY_BSR 1u
#define CS_PHY_SCSR 31u
#define CS_PHY_BCR_RESET 0x8000u
#define CS_PHY_BCR_AUTONEG 0x1000u
#define CS_PHY_BCR_RESTART_AN 0x0200u
#define CS_PHY_BSR_LINK 0x0004u
#define CS_PHY_SCSR_AUTODONE 0x1000u
#define CS_PHY_SCSR_SPEED_MASK 0x001Cu
#define CS_PHY_SCSR_10HD 0x0004u
#define CS_PHY_SCSR_100HD 0x0008u
#define CS_PHY_SCSR_10FD 0x0014u
#define CS_PHY_SCSR_100FD 0x0018u

static ETH_DMADescTypeDef g_rx_desc[ETH_RX_DESC_CNT] ETH_DMA_MEM;
static ETH_DMADescTypeDef g_tx_desc[ETH_TX_DESC_CNT] ETH_DMA_MEM;
static uint8_t g_rx_pool[RX_POOL][RX_BUF_SIZE] ETH_DMA_MEM;
static uint8_t g_tx_buf[CS_NET_MAX_FRAME + 2u] ETH_DMA_MEM;

static bool g_rx_used[RX_POOL];
static uint16_t g_rx_len[RX_POOL];
static bool g_rx_bad[RX_POOL];

static uint8_t g_reply[CS_NET_MAX_FRAME];
static uint8_t g_mac[6];
static ETH_HandleTypeDef g_heth;
static ETH_TxPacketConfigTypeDef g_txcfg;
static bool g_started;

// ---- RX buffer pool (HAL callbacks) ---------------------------------------------------------

static int pool_index(const void * buf)
{
  const uint8_t * p = (const uint8_t *)buf;
  if (p < &g_rx_pool[0][0] || p >= &g_rx_pool[RX_POOL - 1u][0] + RX_BUF_SIZE) {
    return -1;
  }
  return (int)((size_t)(p - &g_rx_pool[0][0]) / RX_BUF_SIZE);
}

static void pool_free(const void * buf)
{
  const int i = pool_index(buf);
  if (i >= 0) {
    g_rx_used[i] = false;
  }
}

void HAL_ETH_RxAllocateCallback(uint8_t ** buff)
{
  for (unsigned i = 0; i < RX_POOL; ++i) {
    if (!g_rx_used[i]) {
      g_rx_used[i] = true;
      g_rx_bad[i] = false;
      g_rx_len[i] = 0;
      *buff = g_rx_pool[i];
      return;
    }
  }
  *buff = NULL;  // the HAL retries on the next read
}

void HAL_ETH_RxLinkCallback(void ** pStart, void ** pEnd, uint8_t * buff, uint16_t Length)
{
  const int i = pool_index(buff);
  if (*pStart == NULL) {
    *pStart = buff;
    if (i >= 0) {
      g_rx_len[i] = Length;
    }
  } else {
    // A frame spanning several buffers is larger than anything we handle: drop it.
    const int first = pool_index(*pStart);
    if (first >= 0) {
      g_rx_bad[first] = true;
    }
    pool_free(buff);
  }
  *pEnd = buff;
}

// ---- MSP ------------------------------------------------------------------------------------

#define CS__ETH_PIN_INIT(code, a) \
  cs_gpio_init((code), GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CS_ETH_AF);

void HAL_ETH_MspInit(ETH_HandleTypeDef * heth)
{
  (void)heth;
#if defined(CS_FAMILY_H7)
  __HAL_RCC_ETH1MAC_CLK_ENABLE();
  __HAL_RCC_ETH1TX_CLK_ENABLE();
  __HAL_RCC_ETH1RX_CLK_ENABLE();
#else
  __HAL_RCC_ETHMAC_CLK_ENABLE();
  __HAL_RCC_ETHMACTX_CLK_ENABLE();
  __HAL_RCC_ETHMACRX_CLK_ENABLE();
#endif
  CS_ETH_PINS(CS__ETH_PIN_INIT, ~)
}

// ---- PHY ------------------------------------------------------------------------------------

static bool phy_read(uint32_t reg, uint32_t * value)
{
  return HAL_ETH_ReadPHYRegister(&g_heth, CS_ETH_PHY_ADDR, reg, value) == HAL_OK;
}

static bool phy_write(uint32_t reg, uint32_t value)
{
  return HAL_ETH_WritePHYRegister(&g_heth, CS_ETH_PHY_ADDR, reg, value) == HAL_OK;
}

static void phy_reset(void)
{
  if (!phy_write(CS_PHY_BCR, CS_PHY_BCR_RESET)) {
    return;
  }
  const uint32_t start = HAL_GetTick();
  uint32_t bcr = CS_PHY_BCR_RESET;
  while ((bcr & CS_PHY_BCR_RESET) != 0u && HAL_GetTick() - start < 500u) {
    if (!phy_read(CS_PHY_BCR, &bcr)) {
      return;
    }
  }
  phy_write(CS_PHY_BCR, CS_PHY_BCR_AUTONEG | CS_PHY_BCR_RESTART_AN);
}

// ---- API ------------------------------------------------------------------------------------

bool cs_eth_init(const uint8_t mac[6])
{
  memcpy(g_mac, mac, sizeof(g_mac));
  memset(g_rx_desc, 0, sizeof(g_rx_desc));
  memset(g_tx_desc, 0, sizeof(g_tx_desc));
  g_heth.Instance = ETH;
  g_heth.Init.MACAddr = g_mac;
  g_heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  g_heth.Init.RxDesc = g_rx_desc;
  g_heth.Init.TxDesc = g_tx_desc;
  g_heth.Init.RxBuffLen = RX_BUF_SIZE;
  if (HAL_ETH_Init(&g_heth) != HAL_OK) {
    return false;
  }
  memset(&g_txcfg, 0, sizeof(g_txcfg));
  g_txcfg.Attributes = ETH_TX_PACKETS_FEATURES_CRCPAD;
  g_txcfg.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  phy_reset();
  return true;
}

bool cs_eth_poll_link(void)
{
  uint32_t bsr = 0;
  // The link bit latches low: the second read gives the current state.
  if (!phy_read(CS_PHY_BSR, &bsr) || !phy_read(CS_PHY_BSR, &bsr)) {
    return g_started;
  }
  const bool link = (bsr & CS_PHY_BSR_LINK) != 0u;

  if (link && !g_started) {
    uint32_t scsr = 0;
    if (!phy_read(CS_PHY_SCSR, &scsr) || (scsr & CS_PHY_SCSR_AUTODONE) == 0u) {
      return false;  // auto-negotiation still running
    }
    ETH_MACConfigTypeDef mac;
    HAL_ETH_GetMACConfig(&g_heth, &mac);
    const uint32_t speed = scsr & CS_PHY_SCSR_SPEED_MASK;
    mac.Speed = (speed == CS_PHY_SCSR_100FD || speed == CS_PHY_SCSR_100HD) ? ETH_SPEED_100M : ETH_SPEED_10M;
    mac.DuplexMode =
      (speed == CS_PHY_SCSR_100FD || speed == CS_PHY_SCSR_10FD) ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;
    HAL_ETH_SetMACConfig(&g_heth, &mac);
    g_started = HAL_ETH_Start(&g_heth) == HAL_OK;
  } else if (!link && g_started) {
    HAL_ETH_Stop(&g_heth);
    g_started = false;
  }
  return g_started;
}

bool cs_eth_send(const uint8_t * frame, size_t len)
{
  if (!g_started || len > CS_NET_MAX_FRAME) {
    return false;
  }
  memcpy(g_tx_buf, frame, len);
  ETH_BufferTypeDef buf = {.buffer = g_tx_buf, .len = (uint32_t)len, .next = NULL};
  g_txcfg.Length = (uint32_t)len;
  g_txcfg.TxBuffer = &buf;
  // Blocking until the DMA has the frame: a few microseconds at 100 Mbit/s. A stuck MAC gives up
  // after 2 ms. Trigger timing is unaffected either way (interrupts stay enabled).
  const bool ok = HAL_ETH_Transmit(&g_heth, &g_txcfg, 2) == HAL_OK;
  HAL_ETH_ReleaseTxPacket(&g_heth);
  return ok;
}

void cs_eth_poll_rx(cs_net_t * net)
{
  if (!g_started) {
    return;
  }
  for (int n = 0; n < 4; ++n) {
    void * buf = NULL;
    if (HAL_ETH_ReadData(&g_heth, &buf) != HAL_OK || buf == NULL) {
      return;
    }
    const int i = pool_index(buf);
    if (i >= 0 && !g_rx_bad[i]) {
      const size_t reply = cs_net_handle_frame(net, (const uint8_t *)buf, g_rx_len[i], g_reply, sizeof(g_reply));
      if (reply > 0u) {
        cs_eth_send(g_reply, reply);
      }
    }
    pool_free(buf);
  }
}
