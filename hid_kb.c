/******************************************************************************
 * @file    hid_kb.c
 * @brief   M031 純輸出版本 (無視 PC Caps/Num Lock 指令，防當機優化)
 *****************************************************************************/
#include <string.h>
#include "NuMicro.h"
#include "hid_kb.h"

uint8_t volatile g_u8Suspend = 0;
uint8_t g_u8Idle = 0;

// 【關鍵 1】建立一個資料黑洞，用來吸收 PC 傳來的 LED 燈號指令
static uint8_t dummy_led_buf[8]; 

void USBD_IRQHandler(void)
{
    uint32_t volatile u32IntSts = USBD_GET_INT_FLAG();
    uint32_t volatile u32State = USBD_GET_BUS_STATE();

    if (u32IntSts & USBD_INTSTS_FLDET) {
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);
        if (USBD_IS_ATTACHED()) {
            USBD_ENABLE_USB();
        } else {
            USBD_DISABLE_USB();
        }
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

    if(u32IntSts & USBD_INTSTS_SOF) {
        USBD_CLR_INT_FLAG(USBD_INTSTS_SOF);
    }

    if(u32IntSts & USBD_INTSTS_WAKEUP) {
        USBD_CLR_INT_FLAG(USBD_INTSTS_WAKEUP);
    }

    if (u32IntSts & USBD_INTSTS_USB) {
        if (u32IntSts & USBD_INTSTS_SETUP) {
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);
            USBD_ProcessSetupPacket();
        }

        if (u32IntSts & USBD_INTSTS_EP0) {
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1) {
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);
            USBD_CtrlOut();
        }

        if (u32IntSts & USBD_INTSTS_EP2) {
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            EP2_Handler();
        }
        
        // 清除其他用不到的 EP 旗標
        if (u32IntSts & USBD_INTSTS_EP3) USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
        if (u32IntSts & USBD_INTSTS_EP4) USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        if (u32IntSts & USBD_INTSTS_EP5) USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
        if (u32IntSts & USBD_INTSTS_EP6) USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        if (u32IntSts & USBD_INTSTS_EP7) USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
    }
}

void EP2_Handler(void)  /* Interrupt IN handler */
{
    g_u8EP2Ready = 1;
}

void HID_Init(void)
{
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /* EP2 ==> Interrupt IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    g_u8EP2Ready = 1;
}

void HID_ClassRequest(void)
{
    uint8_t buf[8];
    USBD_GetSetupPacket(buf);

    if(buf[0] & 0x80)    /* request data transfer direction: Device to Host */
    {
        switch(buf[1])
        {
            case GET_IDLE:
            {
                USBD_SET_PAYLOAD_LEN(EP1, buf[6]);
                USBD_PrepareCtrlIn(&g_u8Idle, buf[6]);
                USBD_PrepareCtrlOut(0, 0); 
                break;
            }
            default:
            {
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
    else
    {   /* Host to Device */
        switch(buf[1])
        {
            case SET_REPORT:
            {
                // 【關鍵 2】無條件接收 PC 送來的 Caps/Num 指令，並放進黑洞丟棄，然後回傳 ACK 讓 PC 滿意
                USBD_SET_DATA1(EP1);
                USBD_PrepareCtrlOut(dummy_led_buf, buf[6]); 
                USBD_PrepareCtrlIn(0, 0); 
                break;
            }
            case SET_IDLE:
            {
                g_u8Idle = buf[3]; 
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }
            default:
            {
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
}

// 【關鍵 3】完全刪除 HID_UpdateKbData() 函式，拔掉 printf 毒瘤！