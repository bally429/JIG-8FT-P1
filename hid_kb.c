/******************************************************************************
 * @file    hid_kb.c
 * @brief   M031 純輸出版本 (修復 USB DMA 記憶體對齊，確保 CapsLock 完美同步)
 *****************************************************************************/
#include <string.h>
#include "NuMicro.h"
#include "hid_kb.h"

uint8_t volatile g_u8Suspend = 0;
uint8_t g_u8Idle = 0;

// 【修復核心】：USB 底層 DMA (USBD_MemCopy) 需要 4-Byte 記憶體對齊
// 使用陣列取代單一變數，保證 PC 傳來的 LED 狀態能被 100% 成功寫入不漏接！
__attribute__((aligned(4))) volatile uint8_t g_u8Led_Status[8] = {0};

void USBD_IRQHandler(void)
{
    uint32_t volatile u32IntSts = USBD_GET_INT_FLAG();
    uint32_t volatile u32State = USBD_GET_BUS_STATE();

    if (u32IntSts & USBD_INTSTS_FLDET) {
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);
        if (USBD_IS_ATTACHED()) { USBD_ENABLE_USB(); } 
        else { USBD_DISABLE_USB(); }
    }

    if (u32IntSts & USBD_INTSTS_BUS) {
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);
        if (u32State & USBD_STATE_USBRST) {
            USBD_ENABLE_USB();
            USBD_SwReset();
            g_u8Suspend = 0;
        }
        if (u32State & USBD_STATE_SUSPEND) {
            g_u8Suspend = 1;
            USBD_DISABLE_PHY();
        }
        if (u32State & USBD_STATE_RESUME) {
            USBD_ENABLE_USB();
            g_u8Suspend = 0;
        }
    }

    if(u32IntSts & USBD_INTSTS_SOF) { USBD_CLR_INT_FLAG(USBD_INTSTS_SOF); }
    if(u32IntSts & USBD_INTSTS_WAKEUP) { USBD_CLR_INT_FLAG(USBD_INTSTS_WAKEUP); }

    if (u32IntSts & USBD_INTSTS_USB) {
        if (u32IntSts & USBD_INTSTS_SETUP) {
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);
            USBD_ProcessSetupPacket();
        }
        if (u32IntSts & USBD_INTSTS_EP0) { USBD_CLR_INT_FLAG(USBD_INTSTS_EP0); USBD_CtrlIn(); }
        if (u32IntSts & USBD_INTSTS_EP1) { USBD_CLR_INT_FLAG(USBD_INTSTS_EP1); USBD_CtrlOut(); }
        if (u32IntSts & USBD_INTSTS_EP2) { USBD_CLR_INT_FLAG(USBD_INTSTS_EP2); EP2_Handler(); }
        
        if (u32IntSts & USBD_INTSTS_EP3) USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
        if (u32IntSts & USBD_INTSTS_EP4) USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        if (u32IntSts & USBD_INTSTS_EP5) USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
        if (u32IntSts & USBD_INTSTS_EP6) USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        if (u32IntSts & USBD_INTSTS_EP7) USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
    }
}

void EP2_Handler(void) { g_u8EP2Ready = 1; }

void HID_Init(void) {
    USBD->STBUFSEG = SETUP_BUF_BASE;
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);
    g_u8EP2Ready = 1;
}

void HID_ClassRequest(void) {
    uint8_t buf[8];
    USBD_GetSetupPacket(buf);

    if(buf[0] & 0x80) { // Device to Host
        if(buf[1] == GET_IDLE) {
            USBD_SET_PAYLOAD_LEN(EP1, buf[6]);
            USBD_PrepareCtrlIn(&g_u8Idle, buf[6]);
            USBD_PrepareCtrlOut(0, 0); 
        } else {
            USBD_SetStall(EP0); USBD_SetStall(EP1);
        }
    } else { // Host to Device
        if(buf[1] == SET_REPORT && buf[3] == 2) {
            // 將資料寫入 4-byte 對齊的陣列
            USBD_SET_DATA1(EP1);
            USBD_PrepareCtrlOut((uint8_t *)g_u8Led_Status, buf[6]); 
            USBD_PrepareCtrlIn(0, 0); 
        } 
        else if (buf[1] == SET_IDLE) {
            g_u8Idle = buf[3]; 
            USBD_SET_DATA1(EP0);
            USBD_SET_PAYLOAD_LEN(EP0, 0);
        } else {
            USBD_SetStall(EP0); USBD_SetStall(EP1);
        }
    }
}