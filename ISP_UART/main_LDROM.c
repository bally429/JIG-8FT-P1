/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x32
 * @date     20260728
 *
 * @note JIG-8FT-P1_ISP 底層

 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"
#include "uart_transfer.h"

/* UART0 polling 除錯輸出（不依賴中斷，不影響 UART1 ISP 通道） */
static void dbg_putc(char c)
{
    while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) { }
    UART0->DAT = (uint32_t)c;
}

static void dbg_puts(const char *s)
{
    while (*s) dbg_putc(*s++);
}

static void dbg_putdec(uint32_t v)
{
    char b[12];
    int n = 0;
    if (v == 0) { dbg_putc('0'); return; }
    while (v) { b[n++] = (char)('0' + v % 10); v /= 10; }
    while (n--) dbg_putc(b[n]);
}

void SYS_Init(void)
{
    SYS_UnlockReg();
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* UART1（ISP 通道，PB2=RX, PB3=TX） */
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk);
    SYS->GPB_MFPL |=  (SYS_GPB_MFPL_PB3MFP_UART1_TXD | SYS_GPB_MFPL_PB2MFP_UART1_RXD);

    /* UART0（除錯輸出，PB12=RX, PB13=TX） */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;
    CLK->CLKSEL1  = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0  = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |=  (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    SYS_LockReg();
}

int32_t main(void)
{
    SYS_Init();

    /* 初始化 UART0 供除錯輸出 */
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    UART0->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200));

    /* 初始化 UART1（ISP 通道） */
    UART_Init();

    dbg_puts("LDROM_START\r\n");

    /* 檢查 flash page size */
    if ((GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_I) || (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_G)) {
        if (FMC_FLASH_PAGE_SIZE != 2048) { while (SYS->PDID); }
    } else {
        if (FMC_FLASH_PAGE_SIZE != 512) { while (SYS->PDID); }
    }

    SYS_UnlockReg();   /* ← 新增：解除保護，使 FMC->ISPCTL 可寫入 */
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk);
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

    dbg_puts("UART1_READY\r\n");

    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL  = 0x00;
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    {
        uint8_t got_first = 0;
        while (1)
        {
            /* 診斷：UART1 ISR 收到第一個 byte 時印出 */
            if (bufhead > 0 && !got_first) {
                got_first = 1;
                dbg_puts("RX1\r\n");
            }

            if ((bufhead >= 4) || (bUartDataReady == TRUE))
            {
                uint32_t lcmd = inpw(uart_rcvbuf);
                if (lcmd == CMD_CONNECT)
                {
                    dbg_puts("CONNECT\r\n");
                    goto _ISP;
                }
                else
                {
                    bUartDataReady = FALSE;
                    bufhead = 0;
                }
            }

            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                dbg_puts("TIMEOUT_APROM\r\n");
                goto _APROM;
            }
        }
    }

_ISP:
    while (1)
    {
        if (bUartDataReady == TRUE)
        {
            bUartDataReady = FALSE;
            dbg_puts("RDY\r\n");
            ParseCmd(uart_rcvbuf, 64);
            dbg_puts("PARSED\r\n");
            PutString();
            dbg_puts("SENT\r\n");
        }
    }

_APROM:
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);
    while (1);
}