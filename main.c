/****************************************************************************
 * @file     JIG_8FT_P1.c
 * @version  v1.37.1
 * @Date     Fri Jun 12 2026 11:42:40 GMT+0800 (台北標準時間)
 * @brief    NuMicro generated code file
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (C) 2013-2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

/********************
MCU:M032SE3AE(LQFP64)
Pin Configuration:
Pin1:PB.6
Pin2:PB.5
Pin3:PB.4
Pin4:UART1_TXD
Pin5:UART1_RXD
Pin6:UART2_TXD
Pin7:UART2_RXD
Pin8:PA.11
Pin9:PA.10
Pin10:PA.9
Pin11:PA.8
Pin12:PF.6
Pin13:PF.14
Pin14:PF.5
Pin15:PF.4
Pin16:PF.3
Pin17:PF.2
Pin18:PC.7
Pin19:PC.6
Pin20:I2C1_SCL
Pin21:I2C1_SDA
Pin24:PD.15
Pin25:I2C0_SCL
Pin26:I2C0_SDA
Pin27:SPI0_SS
Pin28:SPI0_CLK
Pin29:PA.1
Pin30:SPI0_MOSI
Pin31:PF.15
Pin33:ICE_DAT
Pin34:ICE_CLK
Pin35:PC.5
Pin36:PC.4
Pin37:PWM1_CH2
Pin38:PWM1_CH3
Pin39:PC.1
Pin40:PC.0
Pin41:PD.3
Pin42:PD.2
Pin43:PD.1
Pin44:PD.0
Pin52:PC.14
Pin53:PWM1_CH0
Pin54:PB.14
Pin55:UART0_TXD
Pin56:UART0_RXD
Pin60:PB.11
Pin61:PB.10
Pin62:PB.9
Pin63:PB.8
Pin64:PB.7
Module Configuration:
PB.4(Pin:3)
PB.5(Pin:2)
PB.6(Pin:1)
PB.7(Pin:64)
PB.8(Pin:63)
PB.9(Pin:62)
PB.10(Pin:61)
PB.11(Pin:60)
PB.14(Pin:54)
UART1_RXD(Pin:5)
UART1_TXD(Pin:4)
UART2_RXD(Pin:7)
UART2_TXD(Pin:6)
PA.1(Pin:29)
PA.8(Pin:11)
PA.9(Pin:10)
PA.10(Pin:9)
PA.11(Pin:8)
PF.2(Pin:17)
PF.3(Pin:16)
PF.4(Pin:15)
PF.5(Pin:14)
PF.6(Pin:12)
PF.14(Pin:13)
PF.15(Pin:31)
PC.0(Pin:40)
PC.1(Pin:39)
PC.4(Pin:36)
PC.5(Pin:35)
PC.6(Pin:19)
PC.7(Pin:18)
PC.14(Pin:52)
I2C1_SCL(Pin:20)
I2C1_SDA(Pin:21)
PD.0(Pin:44)
PD.1(Pin:43)
PD.2(Pin:42)
PD.3(Pin:41)
PD.15(Pin:24)
I2C0_SCL(Pin:25)
I2C0_SDA(Pin:26)
SPI0_CLK(Pin:28)
SPI0_MOSI(Pin:30)
SPI0_SS(Pin:27)
ICE_CLK(Pin:34)
ICE_DAT(Pin:33)
PWM1_CH0(Pin:53)
PWM1_CH2(Pin:37)
PWM1_CH3(Pin:38)
UART0_RXD(Pin:56)
UART0_TXD(Pin:55)
GPIO Configuration:
PA.0:SPI0_MOSI(Pin:30)
PA.1:PA.1(Pin:29)
PA.2:SPI0_CLK(Pin:28)
PA.3:SPI0_SS(Pin:27)
PA.4:I2C0_SDA(Pin:26)
PA.5:I2C0_SCL(Pin:25)
PA.6:I2C1_SDA(Pin:21)
PA.7:I2C1_SCL(Pin:20)
PA.8:PA.8(Pin:11)
PA.9:PA.9(Pin:10)
PA.10:PA.10(Pin:9)
PA.11:PA.11(Pin:8)
PB.0:UART2_RXD(Pin:7)
PB.1:UART2_TXD(Pin:6)
PB.2:UART1_RXD(Pin:5)
PB.3:UART1_TXD(Pin:4)
PB.4:PB.4(Pin:3)
PB.5:PB.5(Pin:2)
PB.6:PB.6(Pin:1)
PB.7:PB.7(Pin:64)
PB.8:PB.8(Pin:63)
PB.9:PB.9(Pin:62)
PB.10:PB.10(Pin:61)
PB.11:PB.11(Pin:60)
PB.12:UART0_RXD(Pin:56)
PB.13:UART0_TXD(Pin:55)
PB.14:PB.14(Pin:54)
PB.15:PWM1_CH0(Pin:53)
PC.0:PC.0(Pin:40)
PC.1:PC.1(Pin:39)
PC.2:PWM1_CH3(Pin:38)
PC.3:PWM1_CH2(Pin:37)
PC.4:PC.4(Pin:36)
PC.5:PC.5(Pin:35)
PC.6:PC.6(Pin:19)
PC.7:PC.7(Pin:18)
PC.14:PC.14(Pin:52)
PD.0:PD.0(Pin:44)
PD.1:PD.1(Pin:43)
PD.2:PD.2(Pin:42)
PD.3:PD.3(Pin:41)
PD.15:PD.15(Pin:24)
PF.0:ICE_DAT(Pin:33)
PF.1:ICE_CLK(Pin:34)
PF.2:PF.2(Pin:17)
PF.3:PF.3(Pin:16)
PF.4:PF.4(Pin:15)
PF.5:PF.5(Pin:14)
PF.6:PF.6(Pin:12)
PF.14:PF.14(Pin:13)
PF.15:PF.15(Pin:31)
********************/

#include "M031Series.h"

void JIG_8FT_P1_init_i2c0(void)
{
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA5MFP_I2C0_SCL | SYS_GPA_MFPL_PA4MFP_I2C0_SDA);

    return;
}

void JIG_8FT_P1_deinit_i2c0(void)
{
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk);

    return;
}

void JIG_8FT_P1_init_i2c1(void)
{
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA7MFP_Msk | SYS_GPA_MFPL_PA6MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA7MFP_I2C1_SCL | SYS_GPA_MFPL_PA6MFP_I2C1_SDA);

    return;
}

void JIG_8FT_P1_deinit_i2c1(void)
{
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA7MFP_Msk | SYS_GPA_MFPL_PA6MFP_Msk);

    return;
}

void JIG_8FT_P1_init_ice(void)
{
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF1MFP_Msk | SYS_GPF_MFPL_PF0MFP_Msk);
    SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF1MFP_ICE_CLK | SYS_GPF_MFPL_PF0MFP_ICE_DAT);

    return;
}

void JIG_8FT_P1_deinit_ice(void)
{
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF1MFP_Msk | SYS_GPF_MFPL_PF0MFP_Msk);

    return;
}

void JIG_8FT_P1_init_pa(void)
{
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA11MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA8MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA11MFP_GPIO | SYS_GPA_MFPH_PA10MFP_GPIO | SYS_GPA_MFPH_PA9MFP_GPIO | SYS_GPA_MFPH_PA8MFP_GPIO);
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA1MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA1MFP_GPIO);

    return;
}

void JIG_8FT_P1_deinit_pa(void)
{
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA11MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA8MFP_Msk);
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA1MFP_Msk);

    return;
}

void JIG_8FT_P1_init_pb(void)
{
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB11MFP_Msk | SYS_GPB_MFPH_PB10MFP_Msk | SYS_GPB_MFPH_PB9MFP_Msk | SYS_GPB_MFPH_PB8MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB14MFP_GPIO | SYS_GPB_MFPH_PB11MFP_GPIO | SYS_GPB_MFPH_PB10MFP_GPIO | SYS_GPB_MFPH_PB9MFP_GPIO | SYS_GPB_MFPH_PB8MFP_GPIO);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB7MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB7MFP_GPIO | SYS_GPB_MFPL_PB6MFP_GPIO | SYS_GPB_MFPL_PB5MFP_GPIO | SYS_GPB_MFPL_PB4MFP_GPIO);

    return;
}

void JIG_8FT_P1_deinit_pb(void)
{
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB11MFP_Msk | SYS_GPB_MFPH_PB10MFP_Msk | SYS_GPB_MFPH_PB9MFP_Msk | SYS_GPB_MFPH_PB8MFP_Msk);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB7MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk);

    return;
}

void JIG_8FT_P1_init_pc(void)
{
    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC14MFP_Msk);
    SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC14MFP_GPIO);
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC7MFP_Msk | SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC7MFP_GPIO | SYS_GPC_MFPL_PC6MFP_GPIO | SYS_GPC_MFPL_PC5MFP_GPIO | SYS_GPC_MFPL_PC4MFP_GPIO | SYS_GPC_MFPL_PC1MFP_GPIO | SYS_GPC_MFPL_PC0MFP_GPIO);

    return;
}

void JIG_8FT_P1_deinit_pc(void)
{
    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC14MFP_Msk);
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC7MFP_Msk | SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk);

    return;
}

void JIG_8FT_P1_init_pd(void)
{
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPH |= (SYS_GPD_MFPH_PD15MFP_GPIO);
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD3MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD0MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD3MFP_GPIO | SYS_GPD_MFPL_PD2MFP_GPIO | SYS_GPD_MFPL_PD1MFP_GPIO | SYS_GPD_MFPL_PD0MFP_GPIO);

    return;
}

void JIG_8FT_P1_deinit_pd(void)
{
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD3MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD0MFP_Msk);

    return;
}

void JIG_8FT_P1_init_pf(void)
{
    SYS->GPF_MFPH &= ~(SYS_GPF_MFPH_PF15MFP_Msk | SYS_GPF_MFPH_PF14MFP_Msk);
    SYS->GPF_MFPH |= (SYS_GPF_MFPH_PF15MFP_GPIO | SYS_GPF_MFPH_PF14MFP_GPIO);
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF6MFP_Msk | SYS_GPF_MFPL_PF5MFP_Msk | SYS_GPF_MFPL_PF4MFP_Msk | SYS_GPF_MFPL_PF3MFP_Msk | SYS_GPF_MFPL_PF2MFP_Msk);
    SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF6MFP_GPIO | SYS_GPF_MFPL_PF5MFP_GPIO | SYS_GPF_MFPL_PF4MFP_GPIO | SYS_GPF_MFPL_PF3MFP_GPIO | SYS_GPF_MFPL_PF2MFP_GPIO);

    return;
}

void JIG_8FT_P1_deinit_pf(void)
{
    SYS->GPF_MFPH &= ~(SYS_GPF_MFPH_PF15MFP_Msk | SYS_GPF_MFPH_PF14MFP_Msk);
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF6MFP_Msk | SYS_GPF_MFPL_PF5MFP_Msk | SYS_GPF_MFPL_PF4MFP_Msk | SYS_GPF_MFPL_PF3MFP_Msk | SYS_GPF_MFPL_PF2MFP_Msk);

    return;
}

void JIG_8FT_P1_init_pwm1(void)
{
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB15MFP_PWM1_CH0);
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC3MFP_PWM1_CH2 | SYS_GPC_MFPL_PC2MFP_PWM1_CH3);

    return;
}

void JIG_8FT_P1_deinit_pwm1(void)
{
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk);
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk);

    return;
}

void JIG_8FT_P1_init_spi0(void)
{
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA0MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA3MFP_SPI0_SS | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA0MFP_SPI0_MOSI);

    return;
}

void JIG_8FT_P1_deinit_spi0(void)
{
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA0MFP_Msk);

    return;
}

void JIG_8FT_P1_init_uart0(void)
{
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB12MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB13MFP_UART0_TXD | SYS_GPB_MFPH_PB12MFP_UART0_RXD);

    return;
}

void JIG_8FT_P1_deinit_uart0(void)
{
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB12MFP_Msk);

    return;
}

void JIG_8FT_P1_init_uart1(void)
{
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB3MFP_UART1_TXD | SYS_GPB_MFPL_PB2MFP_UART1_RXD);

    return;
}

void JIG_8FT_P1_deinit_uart1(void)
{
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk);

    return;
}

void JIG_8FT_P1_init_uart2(void)
{
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB0MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB1MFP_UART2_TXD | SYS_GPB_MFPL_PB0MFP_UART2_RXD);

    return;
}

void JIG_8FT_P1_deinit_uart2(void)
{
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB0MFP_Msk);

    return;
}

void JIG_8FT_P1_init(void)
{
    //SYS->GPA_MFPH = 0x00000000UL;
    //SYS->GPA_MFPL = 0x88994404UL;
    //SYS->GPB_MFPH = 0xB0660000UL;
    //SYS->GPB_MFPL = 0x00006677UL;
    //SYS->GPC_MFPH = 0x00000000UL;
    //SYS->GPC_MFPL = 0x0000CC00UL;
    //SYS->GPD_MFPH = 0x00000000UL;
    //SYS->GPD_MFPL = 0x00000000UL;
    //SYS->GPF_MFPH = 0x00000000UL;
    //SYS->GPF_MFPL = 0x000000EEUL;

    JIG_8FT_P1_init_i2c0();
    JIG_8FT_P1_init_i2c1();
    JIG_8FT_P1_init_ice();
    JIG_8FT_P1_init_pa();
    JIG_8FT_P1_init_pb();
    JIG_8FT_P1_init_pc();
    JIG_8FT_P1_init_pd();
    JIG_8FT_P1_init_pf();
    JIG_8FT_P1_init_pwm1();
    JIG_8FT_P1_init_spi0();
    JIG_8FT_P1_init_uart0();
    JIG_8FT_P1_init_uart1();
    JIG_8FT_P1_init_uart2();

    return;
}

void JIG_8FT_P1_deinit(void)
{
    JIG_8FT_P1_deinit_i2c0();
    JIG_8FT_P1_deinit_i2c1();
    JIG_8FT_P1_deinit_ice();
    JIG_8FT_P1_deinit_pa();
    JIG_8FT_P1_deinit_pb();
    JIG_8FT_P1_deinit_pc();
    JIG_8FT_P1_deinit_pd();
    JIG_8FT_P1_deinit_pf();
    JIG_8FT_P1_deinit_pwm1();
    JIG_8FT_P1_deinit_spi0();
    JIG_8FT_P1_deinit_uart0();
    JIG_8FT_P1_deinit_uart1();
    JIG_8FT_P1_deinit_uart2();

    return;
}

/* 主程式進入點 */
int main(void)
{
    /* 解鎖受保護的暫存器 (新唐 MCU 必做) */
    SYS_UnlockReg();

    /* 執行系統初始化 (這裡面通常會包含時鐘設定與腳位設定) */
    // 如果 PinConfigure 有生成類似 SYS_Init() 的函數，請在這裡呼叫它
    
    /* 執行你的周邊初始化 (呼叫你剛剛改好的那些函數) */
    JIG_8FT_P1_init_pa();
    JIG_8FT_P1_init_pb();
    // 依需求補上其他 init 函數...

    /* 重新鎖定受保護的暫存器 */
    SYS_LockReg();

    /* 進入主迴圈 (程式永遠停在這裡面執行) */
    while(1)
    {
        // 這裡是你未來要放步進馬達控制或 WIFIBLE 邏輯的地方
    }
}
/*** (C) COPYRIGHT 2013-2026 Nuvoton Technology Corp. ***/
