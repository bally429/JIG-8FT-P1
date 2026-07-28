/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x32
 * @date     20260728
 *
 * @note
 * JIG-8FT-P1 ESP32 UART1
 ******************************************************************************/
#ifndef TARGETDEV_H
#define TARGETDEV_H

#include "NuMicro.h"
#include "isp_user.h"

/* ===== [OTA P2] 取消 BSP 既有的 inpw/outpw/outps 巨集定義 =====
   BSP (M031Series.h) 將這三者定義為直接存取 volatile 指標的巨集，
   與 ISP 範例需要的 byte-wise LE 封包解析/組裝語義衝突。
   必須在此 #undef，否則 static inline 定義會被巨集展開而編譯失敗。 */
#undef inpw
#undef outpw
#undef outps

/* ===== [OTA P2] 補上 byte-wise little-endian 封包操作函式 =====
   以逐 byte 組裝避免 Cortex-M0+ unaligned access hardfault，
   並確保 ISP 協定的封包佈局與 host 端完全對稱。 */
static inline uint32_t inpw(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void outpw(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;       p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline void outps(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;       p[1] = (uint8_t)(v >> 8);
}

/* ===== [OTA P2] ISP 通訊改走 UART1 (PB2=RX, PB3=TX) =====
   ESP32 物理接在 UART1；原範例綁定 UART0/PB12/PB13 僅供 debug 觀測。 */
#define DetectPin                   PB12
#define UART_N                      UART1
#define UART_N_IRQHandler           UART13_IRQHandler
#define UART_N_IRQn                 UART13_IRQn
#define CONFIG_SIZE                 8   /* in bytes */

#endif /* TARGETDEV_H */