/*
 * ===========================================================================================
 * Project: JIG-8FT-P1 _WIFIBLE (M031 端主程式 - 純讀卡機測試版)
 * MCU: Nuvoton M032SE3AE
 * ESP32: ESPWM32EN1
 * OLED: 3.2inch 256x64 mono white OLED Module (SSD1322)
 * RTC: RV-3028-C7
 * PowerMonitor: INA237 I2C Interface
 * 目前版本：v4.3 (2026/06/19)
 * 修改版本List 如下:
 * v2.5 (202606181820)
 * 1.優化 OLED.c 底層驅動效能，導入視窗定址與位元運算，提升渲染速度並保留灰階相容性
 * v3.5 (202606181930)
 * 1.新增二級滾輪選單 "Select Baud Rate"，支援動態選擇 UART2 包率
 * v3.8 (202606182130)
 * 1.移除各測試介面進入迴圈前的清屏，利用 Safe_Print_OLED 覆寫特性消除黑屏時間，提升轉場流暢度
 * v3.9 (202606182200)
 * 1.新增 OLED 灰階字體支援 (g_u8TextBrightness)，實現選單漸層淡化效果
 * v4.0 (202606182300)
 * 1.架構大重構：抽離 UI 渲染邏輯，新增 UI_Draw_Menu_State 與 UI_Menu_Scroll_Anim 函數
 * v4.2 (202606190100) 
 * 1.新增全域變數 g_u8BuzzerEnabled (預設為 0 靜音)，實現開機與平時操作無聲。
 * 2.音訊底層分流：新增 ForceBeep() 供重大警報使用不受靜音限制；一般 Beep() 則受靜音控制。
 * 3.主選單新增 "Buzzer Settings"，支援動態顯示狀態，PF.3 開啟、PF.4 關閉。
 * v4.3 (202606190200) [終極顯存革命]
 * 1.重構 OLED.c 導入 8KB Frame Buffer (OLED_GRAM) 雙緩衝機制，澈底消除 Screen Tearing 畫面閃爍。
 * 2.實作任意 Y 座標的軟體裁切引擎 (OLED_DrawChar)，實現字體滑入/滑出的真實物理位移。
 * 3.所有 UI 渲染皆改為「記憶體繪製 -> OLED_Update() 一次性刷新」的現代圖形架構。
 * ===========================================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "NuMicro.h"
#include "OLED.h"
#include "queue.h" 
#include "RV3028.h"

// =======================================================
// [外部變數與系統全域變數]
// =======================================================
uint8_t g_u8BuzzerEnabled = 0;

extern void vCheckingTimeOut(void);
extern uint8_t g_u8WiegandNum; 
QUEUE_U64_REFERENCE(au64WG1, 128);

extern uint8_t g_u8TK2Bit[128];
extern uint8_t TK2Cnt;
extern uint8_t g_u8TK2Step;

extern void vINA237_Init(void);
extern void set237Calibration_1A(void);
extern float getBusVoltage_V(void);
extern float getCurrent_mA(void);
extern float getPower_mW(void);

#define CMD_STX 0x02
#define CMD_CR  0x0D

#define RX0_BUF_SIZE 128
volatile uint8_t g_u0_rx_buf[RX0_BUF_SIZE];
volatile uint16_t g_u0_rx_head = 0;
volatile uint16_t g_u0_rx_tail = 0;

#define UART2_RX_BUF_SIZE 256
volatile uint8_t g_u2_rx_buf[UART2_RX_BUF_SIZE];
volatile uint16_t g_u2_rx_head = 0;
volatile uint16_t g_u2_rx_tail = 0;

// =======================================================
// [Function Prototypes]
// =======================================================
void SYS_Init(void);
void Setup_GPIO_Modes(void); 
void OLED_Force_Reset(void);         
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us); 
void ForceBeep(uint32_t ms); 
void Beep(uint32_t ms);      
void Trigger_RedLight_Alarm(void);

int UART0_Read_Byte(uint8_t *data);
void UART0_Flush_Rx_Buffer(void);
void UART1_Flush_Rx_Buffer(void); 

void Calculate_Checksum(const uint8_t *payload, uint16_t payload_len, char *out_chk);
int Execute_TR515_Command_With_Retry(const char *tx_payload);

void Interface_init(void); 
void UART_Monitor_Test(uint32_t u32BaudRate); 
void Wiegand_Monitor_Test(void);
void TK2_Monitor_Test(void);
void Decode_TK2_Raw(char* out_str);
void Safe_Print_OLED(int y, const char *fmt, ...); 
void Show_RTC_Time_Loop(uint32_t exit_btn_bit, const char* hint_text);

// =======================================================
// [v4.3 終極顯存 UI 引擎] 
// =======================================================
// 將字串格式化並畫進 RAM (帶動畫位移與裁切功能)
void Safe_Print_OLED_Smooth(int y, int min_y, int max_y, uint8_t brightness, const char *fmt, ...) {
    char temp_buf[128]; char full_line[33]; va_list argptr;
    va_start(argptr, fmt); vsnprintf(temp_buf, sizeof(temp_buf), fmt, argptr); va_end(argptr);
    int temp_len = strlen(temp_buf);
    for(int i = 0; i < 32; i++) {
        if(i < temp_len) { char c = temp_buf[i]; if(c < 0x20 || c > 0x7E) c = ' '; full_line[i] = c; } 
        else { full_line[i] = ' '; }
    }
    full_line[32] = '\0';
    OLED_PrintString(0, y, min_y, max_y, full_line, brightness);
}

// 標準靜態打印 (自動更新至畫面)
void Safe_Print_OLED(int y, const char *fmt, ...) {
    char temp_buf[128]; char full_line[33]; va_list argptr;
    va_start(argptr, fmt); vsnprintf(temp_buf, sizeof(temp_buf), fmt, argptr); va_end(argptr);
    int temp_len = strlen(temp_buf);
    for(int i = 0; i < 32; i++) {
        if(i < temp_len) { char c = temp_buf[i]; if(c < 0x20 || c > 0x7E) c = ' '; full_line[i] = c; } 
        else { full_line[i] = ' '; }
    }
    full_line[32] = '\0';
    OLED_PrintString(0, y, 0, 63, full_line, 0x0F);
}

// 繪製靜止選單
void UI_Draw_Menu_State(const char* title, const char** items, int num_items, int curr_idx) {
    int prev_idx = (curr_idx - 1 + num_items) % num_items;
    int next_idx = (curr_idx + 1) % num_items;

    OLED_Clear();
    Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, title);                  
    Safe_Print_OLED_Smooth(16, 16, 63, 0x04, "  %s", items[prev_idx]); 
    Safe_Print_OLED_Smooth(32, 16, 63, 0x0F, "> %s", items[curr_idx]); 
    Safe_Print_OLED_Smooth(48, 16, 63, 0x04, "  %s", items[next_idx]); 
    OLED_Update(); // 一次性刷新
}

// 物理級絲滑捲動動畫 (結合位移與光影暫留)
void UI_Menu_Scroll_Anim_Smooth(const char* title, const char** items, int num_items, int old_idx, int dir) {
    int p_idx = (old_idx - 1 + num_items) % num_items;
    int n_idx = (old_idx + 1) % num_items;
    int nn_idx = (old_idx + 2) % num_items; 
    int pp_idx = (old_idx - 2 + num_items * 2) % num_items; 

    // 每幀移動 4 個像素，完成 16 像素的字體高度需要 4 幀
    for (int offset = 0; offset <= 16; offset += 4) {
        OLED_Clear(); 
        Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, title); // 標題不動
        
        if (dir == 1) { // 往下滾動 (項目往上滑)
            Safe_Print_OLED_Smooth(16 - offset, 16, 63, 0x02, "  %s", items[p_idx]); // 往上滑入標題底部並被裁切
            Safe_Print_OLED_Smooth(32 - offset, 16, 63, 0x0F - (offset/2), "  %s", items[old_idx]);
            Safe_Print_OLED_Smooth(48 - offset, 16, 63, 0x04 + (offset/2), "> %s", items[n_idx]);
            Safe_Print_OLED_Smooth(64 - offset, 16, 63, 0x02, "  %s", items[nn_idx]); // 從螢幕底部滑出
        } else { // 往上滾動 (項目往下滑)
            Safe_Print_OLED_Smooth(0 + offset, 16, 63, 0x02, "  %s", items[pp_idx]); // 從標題下方滑出
            Safe_Print_OLED_Smooth(16 + offset, 16, 63, 0x04 + (offset/2), "> %s", items[p_idx]);
            Safe_Print_OLED_Smooth(32 + offset, 16, 63, 0x0F - (offset/2), "  %s", items[old_idx]);
            Safe_Print_OLED_Smooth(48 + offset, 16, 63, 0x02, "  %s", items[n_idx]);
        }
        OLED_Update(); // 瞬間更新整片螢幕
    }
}

// =======================================================
// [PinConfig 與底層硬體初始化]
// =======================================================
void WIFIBLE_ReaderTest_init(void) {
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk); 
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA5MFP_I2C0_SCL | SYS_GPA_MFPL_PA4MFP_I2C0_SDA);
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA7MFP_Msk | SYS_GPA_MFPL_PA6MFP_Msk); 
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA7MFP_I2C1_SCL | SYS_GPA_MFPL_PA6MFP_I2C1_SDA);
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF1MFP_Msk | SYS_GPF_MFPL_PF0MFP_Msk); 
    SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF1MFP_ICE_CLK | SYS_GPF_MFPL_PF0MFP_ICE_DAT);
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA11MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA8MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA11MFP_GPIO | SYS_GPA_MFPH_PA10MFP_GPIO | SYS_GPA_MFPH_PA9MFP_GPIO | SYS_GPA_MFPH_PA8MFP_GPIO);
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA1MFP_Msk); SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA1MFP_GPIO);
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk | SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB8MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB15MFP_GPIO | SYS_GPB_MFPH_PB14MFP_GPIO | SYS_GPB_MFPH_PB8MFP_GPIO);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB7MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB7MFP_GPIO | SYS_GPB_MFPL_PB6MFP_GPIO | SYS_GPB_MFPL_PB5MFP_GPIO | SYS_GPB_MFPL_PB4MFP_GPIO);
    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC14MFP_Msk); SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC14MFP_GPIO);
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC7MFP_Msk | SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC7MFP_GPIO | SYS_GPC_MFPL_PC6MFP_GPIO | SYS_GPC_MFPL_PC1MFP_GPIO | SYS_GPC_MFPL_PC0MFP_GPIO);
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD15MFP_Msk); SYS->GPD_MFPH |= (SYS_GPD_MFPH_PD15MFP_GPIO);
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD3MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD0MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD3MFP_GPIO | SYS_GPD_MFPL_PD2MFP_GPIO | SYS_GPD_MFPL_PD1MFP_GPIO | SYS_GPD_MFPL_PD0MFP_GPIO);
    SYS->GPF_MFPH &= ~(SYS_GPF_MFPH_PF15MFP_Msk | SYS_GPF_MFPH_PF14MFP_Msk); 
    SYS->GPF_MFPH |= (SYS_GPF_MFPH_PF15MFP_GPIO | SYS_GPF_MFPH_PF14MFP_GPIO);
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF6MFP_Msk | SYS_GPF_MFPL_PF5MFP_Msk | SYS_GPF_MFPL_PF4MFP_Msk | SYS_GPF_MFPL_PF3MFP_Msk | SYS_GPF_MFPL_PF2MFP_Msk);
    SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF6MFP_GPIO | SYS_GPF_MFPL_PF5MFP_GPIO | SYS_GPF_MFPL_PF4MFP_GPIO | SYS_GPF_MFPL_PF3MFP_GPIO | SYS_GPF_MFPL_PF2MFP_GPIO);
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA0MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA3MFP_SPI0_SS | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA0MFP_SPI0_MOSI);
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB12MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB13MFP_UART0_TXD | SYS_GPB_MFPH_PB12MFP_UART0_RXD);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB3MFP_UART1_TXD | SYS_GPB_MFPL_PB2MFP_UART1_RXD);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB0MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB1MFP_UART2_TXD | SYS_GPB_MFPL_PB0MFP_UART2_RXD);
}

void Setup_GPIO_Modes(void) {
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT); PB15 = 0; 
    GPIO_SetMode(PC, BIT1, GPIO_MODE_OUTPUT); GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT); 
    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT); GPIO_SetMode(PD, BIT15, GPIO_MODE_OUTPUT); 
    GPIO_SetMode(PF, BIT15, GPIO_MODE_OUTPUT); 
    GPIO_SetMode(PB, BIT0, GPIO_MODE_QUASI); GPIO_SetMode(PA, BIT1, GPIO_MODE_QUASI); 
    GPIO_SetMode(PD, BIT2, GPIO_MODE_QUASI); GPIO_SetMode(PD, BIT1, GPIO_MODE_QUASI); GPIO_SetMode(PD, BIT0, GPIO_MODE_QUASI); 
    GPIO_SetMode(PA, BIT8, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, BIT6, GPIO_MODE_QUASI); GPIO_SetMode(PF, BIT14, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, BIT5, GPIO_MODE_QUASI); GPIO_SetMode(PF, BIT3, GPIO_MODE_QUASI); GPIO_SetMode(PF, BIT4, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, BIT7, GPIO_MODE_OUTPUT); PC->DOUT &= ~BIT7; 
    PD->DOUT |= BIT15; PF->DOUT |= BIT15; PC->DOUT |= BIT1;  PD->DOUT |= BIT3;  PA->DOUT |= BIT8;
    PF->DOUT |= (BIT6 | BIT14 | BIT5 | BIT3 | BIT4);
}

void Interface_init(void){
    GPIO_SetMode(PB, BIT6, GPIO_MODE_OUTPUT); GPIO_SetMode(PB, BIT7, GPIO_MODE_OUTPUT); 
    GPIO_SetMode(PA, BIT11, GPIO_MODE_OUTPUT); GPIO_SetMode(PB, BIT4, GPIO_MODE_OUTPUT); 
    PB6 = 0; PB7 = 0; PA11 = 0; PB4 = 0;  
}

void SYS_Init(void) {
    SYS_UnlockReg();
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk); CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
    CLK->AHBCLK |= ((1ul << 0)|(1ul << 1)|(1ul << 2)|(1ul << 3)|(1ul << 5)); 
    CLK_EnableModuleClock(SPI0_MODULE); CLK_EnableModuleClock(I2C0_MODULE); CLK_EnableModuleClock(I2C1_MODULE); 
    CLK_EnableModuleClock(UART0_MODULE); CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_EnableModuleClock(UART1_MODULE); CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    CLK_EnableModuleClock(UART2_MODULE); CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
    CLK_EnableModuleClock(TMR0_MODULE); CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    SystemCoreClockUpdate(); WIFIBLE_ReaderTest_init();
    UART_Open(UART0, 115200); UART0->FIFO = (UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE; UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
    UART_Open(UART1, 19200); UART1->FIFO |= (UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk);
    UART_Open(UART2, 9600); UART2->FIFO = (UART2->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE; UART_EnableInt(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    I2C_Open(I2C0, 100000); NVIC_EnableIRQ(UART02_IRQn); SYS_LockReg();
}

void UART02_IRQHandler(void) {
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk)) {
        while(!UART_GET_RX_EMPTY(UART0)) {
            uint8_t c = UART_READ(UART0); uint16_t next_head = (g_u0_rx_head + 1) % RX0_BUF_SIZE;
            if (next_head != g_u0_rx_tail) { g_u0_rx_buf[g_u0_rx_head] = c; g_u0_rx_head = next_head; }
        }
    }
    uint32_t u32u2IntSts = UART2->INTSTS; uint32_t u32u2FIFOSts = UART2->FIFOSTS;
    if(u32u2FIFOSts & (UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_RXOVIF_Msk)) {
        UART2->FIFOSTS = (UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_RXOVIF_Msk); 
    }
    if(u32u2IntSts & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
        while((UART2->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) {
            uint8_t c = UART_READ(UART2); if (c == 0x00 || c == 0xFF) continue;
            uint16_t next_head = (g_u2_rx_head + 1) % UART2_RX_BUF_SIZE;
            if (next_head != g_u2_rx_tail) { g_u2_rx_buf[g_u2_rx_head] = c; g_u2_rx_head = next_head; }
        }
    }
}

int UART0_Read_Byte(uint8_t *data) {
    if (g_u0_rx_head == g_u0_rx_tail) return 0;
    *data = g_u0_rx_buf[g_u0_rx_tail]; g_u0_rx_tail = (g_u0_rx_tail + 1) % RX0_BUF_SIZE; return 1;
}

void UART0_Flush_Rx_Buffer(void) { __disable_irq(); g_u0_rx_head = 0; g_u0_rx_tail = 0; UART0->FIFO |= UART_FIFO_RXRST_Msk; __enable_irq(); }
void UART1_Flush_Rx_Buffer(void) {
    if(UART1->FIFOSTS & (UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk)) { UART1->FIFOSTS = (UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk); }
    while((UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) { char dummy = UART_READ(UART1); } UART1->FIFO |= UART_FIFO_RXRST_Msk; 
}

void Delay_us(uint32_t us) { CLK_SysTickDelay(us); }
void Delay_ms(uint32_t ms) { while (ms >= 100) { TIMER_Delay(TIMER0, 100 * 1000); ms -= 100; } if (ms > 0) TIMER_Delay(TIMER0, ms * 1000); }

// =======================================================
// [v4.2 音訊分流系統]
// =======================================================
void ForceBeep(uint32_t ms) { 
    uint32_t half_period_us = 185; uint32_t total_cycles = (ms * 1000) / (half_period_us * 2);
    for (uint32_t i = 0; i < total_cycles; i++) { PB15 = 1; Delay_us(half_period_us); PB15 = 0; Delay_us(half_period_us); } PB15 = 0; Delay_ms(50); 
}

void Beep(uint32_t ms) {
    if (g_u8BuzzerEnabled) ForceBeep(ms);
}

void Trigger_RedLight_Alarm(void) { 
    PC->DOUT |= BIT7; 
    Safe_Print_OLED(16, "COMM ERROR ALARM"); 
    OLED_Update();
    while(1) { ForceBeep(500); Delay_ms(500); }
}

void OLED_Force_Reset(void) { PD->DOUT |= BIT15; Delay_ms(50); PD->DOUT &= ~BIT15; Delay_ms(200); PD->DOUT |= BIT15; Delay_ms(200); }

void Show_RTC_Time_Loop(uint32_t exit_btn_bit, const char* hint_text) {
    RTC_TimeTypeDef current_time; 
    while(1) {
        OLED_Clear(); // 每幀清空顯存
        RV3028_GetTime(&current_time);
        Safe_Print_OLED(0,  "    --- RTC CURRENT TIME ---");
        Safe_Print_OLED(16, "           %04d/%02d/%02d", current_time.year, current_time.month, current_time.date);
        Safe_Print_OLED(32, "            %02d:%02d:%02d", current_time.hours, current_time.minutes, current_time.seconds);
        Safe_Print_OLED(48, hint_text);
        OLED_Update(); // 一次性更新螢幕
        
        if((PF->PIN & exit_btn_bit) == 0) { Delay_ms(50); if((PF->PIN & exit_btn_bit) == 0) { Beep(100); while((PF->PIN & exit_btn_bit) == 0) { Delay_ms(10); } break; } }
        Delay_ms(100); 
    }
}

// =======================================================
// [TR515 通訊協定與容錯機制]
// =======================================================
void Calculate_Checksum(const uint8_t *payload, uint16_t payload_len, char *out_chk) {
    uint16_t sum = 0; for (uint16_t i = 0; i < payload_len; i++) { sum += payload[i]; }
    sprintf(out_chk, "%02X", (uint8_t)(sum & 0xFF));
}

int Execute_TR515_Command_With_Retry(const char *tx_payload) {
    uint8_t retry_count = 0; const uint8_t MAX_RETRIES = 3; uint8_t tx_buffer[128]; char chk_str[3];
    Calculate_Checksum((const uint8_t*)tx_payload, strlen(tx_payload), chk_str); sprintf((char*)tx_buffer, "%c%s%s%c", CMD_STX, tx_payload, chk_str, CMD_CR);
    while (retry_count < MAX_RETRIES) {
        UART1_Flush_Rx_Buffer(); char *ptr = (char*)tx_buffer; while(*ptr) { while(UART_IS_TX_FULL(UART1)) {} UART_WRITE(UART1, *ptr++); }
        uint32_t timeout = 0; char rx_buffer[128]; int rx_idx = 0; int packet_complete = 0;
        while(timeout < 1000) { 
            if((UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) {
                char c = UART_READ(UART1);
                if(c == CMD_STX) { rx_idx = 0; continue; }
                if(c == CMD_CR) { rx_buffer[rx_idx] = '\0'; packet_complete = 1; break; }
                if(rx_idx < 127) rx_buffer[rx_idx++] = c;
            } else { Delay_ms(1); timeout++; }
        }
        if (packet_complete) {
            if (rx_buffer[0] == 'A') return 0; 
            else if (rx_buffer[0] == 'N') { if (strncmp((char *)rx_buffer, "N04", 3) == 0) { retry_count++; Delay_ms(10); continue; } else { return -1; } }
        } else { retry_count++; }
    } Trigger_RedLight_Alarm(); return -1;
}

// =======================================================
// [測試介面函數]
// =======================================================
void UART_Monitor_Test(uint32_t u32BaudRate) {
    OLED_Clear(); UART_DisableInt(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    Interface_init(); PB4 = 1;               
    UART_Open(UART2, u32BaudRate); UART2->FIFO = (UART2->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;
    UART_EnableInt(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    Safe_Print_OLED(0, "UART2 Mntr %u", u32BaudRate); Safe_Print_OLED(16, "Red button Power"); 
    OLED_Update(); Delay_ms(1500); 

    int rx_count = 0; char rx_buf[128]; uint32_t loop_tick = 0; int power_state = 0;

    while(1) {
        if ((PA->PIN & BIT8) == 0) {
            Delay_ms(50);
            if ((PA->PIN & BIT8) == 0) {
                power_state = !power_state; 
                if (power_state) { 
                    PC->DOUT |= BIT7; Beep(100); Safe_Print_OLED(32, "Powering ON..."); OLED_Update();
                    UART2->FIFO |= UART_FIFO_RXRST_Msk; UART2->FIFOSTS = (UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk);
                    __disable_irq(); g_u2_rx_head = 0; g_u2_rx_tail = 0; __enable_irq();
                    Safe_Print_OLED(32, "P: ON, Listening"); OLED_Update();
                } else { 
                    PC->DOUT &= ~BIT7; Beep(500); Safe_Print_OLED(32, "Power OFF"); OLED_Update();
                }
                while ((PA->PIN & BIT8) == 0) {} Delay_ms(50);
            }
        }

        if (power_state && (UART2->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) {
            int rx_len = 0; int rx_to = 0; memset(rx_buf, 0, sizeof(rx_buf));
            while(rx_len < 127) { 
                if((UART2->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) { 
                    char c = UART_READ(UART2); if(c >= 0x20 && c <= 0x7E) rx_buf[rx_len++] = c; else rx_buf[rx_len++] = '.'; rx_to = 0; 
                } else { Delay_ms(1); rx_to++; if(rx_to > 100) break; }
            }
            if (rx_len >= 4) {
                rx_buf[rx_len] = '\0'; rx_count++; Safe_Print_OLED(48, "%02d/%s", rx_count, rx_buf);
                float voltage = getBusVoltage_V(); float current = getCurrent_mA();
                if (current == 0.0f) { set237Calibration_1A(); current = getCurrent_mA(); }
                Safe_Print_OLED(0,  "%.2fV", voltage); Safe_Print_OLED(16, "%.2fmA", current); 
                OLED_Update(); Beep(50); loop_tick = 0; 
            }
        }
        if((PF->PIN & BIT5) == 0) { Delay_ms(50); if((PF->PIN & BIT5) == 0) { Beep(200); while((PF->PIN & BIT5)==0){} break; } }
        
        if (loop_tick >= 1000) { 
            float voltage = getBusVoltage_V(); float current = getCurrent_mA();
            if (current == 0.0f) { set237Calibration_1A(); current = getCurrent_mA(); }
            Safe_Print_OLED(0,  "%.2fV", voltage); Safe_Print_OLED(16, "%.2fmA", current); 
            OLED_Update(); loop_tick = 0; 
        }
        Delay_ms(1); loop_tick++;
    }
    PC->DOUT &= ~BIT7; OLED_Clear(); Safe_Print_OLED(0, "Monitor End"); OLED_Update(); Delay_ms(1000);
}

void Wiegand_Monitor_Test(void) {
    OLED_Clear(); Interface_init(); PB6 = 1; PA11 = 1; 
    GPIO_SetMode(PA, BIT10, GPIO_MODE_QUASI); GPIO_SetMode(PB, BIT5, GPIO_MODE_QUASI); GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5);
    Safe_Print_OLED(0, "Wiegand Monitor"); Safe_Print_OLED(16, "Red button Power"); 
    OLED_Update(); Delay_ms(1500); QUEUE_CLEAR(au64WG1); 

    int rx_count = 0; uint32_t loop_tick = 0; int power_state = 0; 

    while(1) {
        if ((PA->PIN & BIT8) == 0) {
            Delay_ms(50);
            if ((PA->PIN & BIT8) == 0) {
                power_state = !power_state; 
                if (power_state) { 
                    PC->DOUT |= BIT7; Beep(100); Safe_Print_OLED(32, "Powering ON..."); OLED_Update();
                    QUEUE_CLEAR(au64WG1); GPIO_CLR_INT_FLAG(PA, BIT10); GPIO_CLR_INT_FLAG(PB, BIT5);
                    GPIO_EnableInt(PA, 10, GPIO_INT_FALLING); GPIO_EnableInt(PB, 5, GPIO_INT_FALLING);  NVIC_EnableIRQ(GPIO_PAPBPGPH_IRQn); 
                    Safe_Print_OLED(32, "P: ON, Listening"); OLED_Update();
                } else { 
                    GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5); PC->DOUT &= ~BIT7; Beep(500); Safe_Print_OLED(32, "Power OFF      "); OLED_Update();
                }
                while ((PA->PIN & BIT8) == 0) {} Delay_ms(50);
            }
        }
        vCheckingTimeOut();
        if (!QUEUE_EMPTY(au64WG1)) { 
            uint64_t wg_data = QUEUE_PULL(au64WG1); 
            if (g_u8WiegandNum > 0) {
                rx_count++; Safe_Print_OLED(48, "%02d/W%02d:%llX", rx_count, g_u8WiegandNum, wg_data);
                float voltage = getBusVoltage_V(); float current = getCurrent_mA();
                if (current == 0.0f) { set237Calibration_1A(); current = getCurrent_mA(); }
                Safe_Print_OLED(0,  "%.2fV", voltage); Safe_Print_OLED(16, "%.2fmA", current); 
                OLED_Update(); Beep(50); loop_tick = 0; 
            }
        }
        if((PF->PIN & BIT5) == 0) { Delay_ms(50); if((PF->PIN & BIT5) == 0) { GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5); Beep(200); while((PF->PIN & BIT5)==0){} break; } }
        
        if (loop_tick >= 1000) {
            float voltage = getBusVoltage_V(); float current = getCurrent_mA();
            if (current == 0.0f) { set237Calibration_1A(); current = getCurrent_mA(); }
            Safe_Print_OLED(0,  "%.2fV", voltage); Safe_Print_OLED(16, "%.2fmA", current); 
            OLED_Update(); loop_tick = 0; 
        }
        Delay_ms(1); loop_tick++;
    }
    PC->DOUT &= ~BIT7; GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5); OLED_Clear(); Safe_Print_OLED(0, "Monitor End"); OLED_Update(); Delay_ms(1000);
}

void Decode_TK2_Raw(char* out_str) {
    int start_idx = -1; int out_idx = 0;
    for(int i = 0; i <= TK2Cnt - 5; i++) {
        if(g_u8TK2Bit[i] == 0x31 && g_u8TK2Bit[i+1] == 0x31 && g_u8TK2Bit[i+2] == 0x30 && g_u8TK2Bit[i+3] == 0x31 && g_u8TK2Bit[i+4] == 0x30) { start_idx = i; break; }
    }
    if (start_idx != -1) {
        for(int i = start_idx; i <= TK2Cnt - 5; i += 5) {
            int b0 = g_u8TK2Bit[i] - 0x30; int b1 = g_u8TK2Bit[i+1] - 0x30; int b2 = g_u8TK2Bit[i+2] - 0x30; int b3 = g_u8TK2Bit[i+3] - 0x30;
            uint8_t val = b0*1 + b1*2 + b2*4 + b3*8; char c = val + 0x30; out_str[out_idx++] = c;
            if (c == '?') break; if (out_idx >= 30) break; 
        }
    } else { strcpy(out_str, "NO_SS_ERR"); } out_str[out_idx] = '\0';
}

void TK2_Monitor_Test(void) {
    OLED_Clear(); Interface_init(); PB7 = 1; PA11 = 1; 
    GPIO_SetMode(PA, BIT10, GPIO_MODE_QUASI); GPIO_SetMode(PB, BIT5, GPIO_MODE_QUASI); GPIO_SetMode(PB, BIT8, GPIO_MODE_QUASI); 
    GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5);  GPIO_DisableInt(PB, 8);
    Safe_Print_OLED(0, "TK2 Monitor"); Safe_Print_OLED(16, "Red button Power"); 
    OLED_Update(); Delay_ms(1500);
    
    int rx_count = 0; uint32_t loop_tick = 0; int power_state = 0;
    uint8_t last_tk2_cnt = 0; uint32_t tk2_idle_tick = 0;

    while(1) {
        if ((PA->PIN & BIT8) == 0) {
            Delay_ms(50);
            if ((PA->PIN & BIT8) == 0) {
                power_state = !power_state; 
                if (power_state) { 
                    PC->DOUT |= BIT7; Beep(100); Safe_Print_OLED(32, "Powering ON..."); OLED_Update();
                    TK2Cnt = 0; last_tk2_cnt = 0; tk2_idle_tick = 0; g_u8TK2Step = 0;
                    memset((void *)g_u8TK2Bit, 0, sizeof(g_u8TK2Bit)); GPIO_CLR_INT_FLAG(PB, BIT5); 
                    GPIO_EnableInt(PB, 5, GPIO_INT_FALLING);  NVIC_EnableIRQ(GPIO_PAPBPGPH_IRQn); 
                    Safe_Print_OLED(32, "P: ON, Listening"); OLED_Update();
                } else { 
                    GPIO_DisableInt(PB, 5); PC->DOUT &= ~BIT7; Beep(500); Safe_Print_OLED(32, "Power OFF"); OLED_Update();
                }
                while ((PA->PIN & BIT8) == 0) {} Delay_ms(50);
            }
        }
        vCheckingTimeOut();
        if (TK2Cnt > 0) {
            if (TK2Cnt != last_tk2_cnt) { last_tk2_cnt = TK2Cnt; tk2_idle_tick = 0; } 
            else {
                tk2_idle_tick++;
                if (tk2_idle_tick > 50) { 
                    rx_count++; char tk2_str[64] = {0}; Decode_TK2_Raw(tk2_str); 
                    Safe_Print_OLED(48, "%02d/%s", rx_count, tk2_str); 
                    
                    float voltage = getBusVoltage_V(); float current = getCurrent_mA();
                    if (current == 0.0f) { set237Calibration_1A(); current = getCurrent_mA(); }
                    Safe_Print_OLED(0,  "%.2fV", voltage); Safe_Print_OLED(16, "%.2fmA", current); 
                    OLED_Update(); Beep(50); loop_tick = 0; 
                    TK2Cnt = 0; last_tk2_cnt = 0; g_u8TK2Step = 0; memset((void *)g_u8TK2Bit, 0, sizeof(g_u8TK2Bit));
                }
            }
        }
        if((PF->PIN & BIT5) == 0) { Delay_ms(50); if((PF->PIN & BIT5) == 0) { GPIO_DisableInt(PB, 5); Beep(200); while((PF->PIN & BIT5)==0){} break; } }
        
        if (loop_tick >= 1000) {
            float voltage = getBusVoltage_V(); float current = getCurrent_mA();
            if (current == 0.0f) { set237Calibration_1A(); current = getCurrent_mA(); }
            Safe_Print_OLED(0,  "%.2fV", voltage); Safe_Print_OLED(16, "%.2fmA", current); 
            OLED_Update(); loop_tick = 0; 
        }
        Delay_ms(1); loop_tick++;
    }
    PC->DOUT &= ~BIT7; GPIO_DisableInt(PB, 5); OLED_Clear(); Safe_Print_OLED(0, "Monitor End"); OLED_Update(); Delay_ms(1000);
}

// =======================================================
// Main 主程式 (v4.3 終極絲滑版)
// =======================================================
int main(void) {
    SYS_Init();
    Setup_GPIO_Modes();
    OLED_Force_Reset();
    vOLED_INIT();
    vINA237_Init();
    set237Calibration_1A();
    RV3028_Init();
    
    OLED_Clear();
    Safe_Print_OLED(0, "System Ready"); 
    Safe_Print_OLED(16, "JIG-8FT-P1 OK"); 
    OLED_Update(); // 一次性更新，拒絕閃爍
    
    Beep(500); Delay_ms(100); Beep(500); // 受控於 g_u8BuzzerEnabled
    Delay_ms(1000);
    
    Show_RTC_Time_Loop(BIT5, "     Yellow button > Menu"); 
    
    const char *menu_items[] = {
        "RS232 Monitor", 
        "Wiegand",
        "TK2",
        "What's the time?",
        "Buzzer Settings"  
    };
    const int NUM_ITEMS = sizeof(menu_items) / sizeof(menu_items[0]);
    int current_idx = 1; 

    const char *baud_items[] = {
        "115200, N, 8, 1",
        "9600, N, 8, 1",
        "19200, N, 8, 1",
        "38400, N, 8, 1"
    };
    const uint32_t baud_values[] = { 115200, 9600, 19200, 38400 };
    const int NUM_BAUDS = sizeof(baud_items) / sizeof(baud_items[0]);

    while(1) {
        UI_Draw_Menu_State("Select Function", menu_items, NUM_ITEMS, current_idx);
        int selected = 0;

        while(1) {
            if((PF->PIN & BIT3) == 0) { 
                Delay_ms(50);
                if((PF->PIN & BIT3) == 0) {
                    Beep(50); 
                    UI_Menu_Scroll_Anim_Smooth("Select Function", menu_items, NUM_ITEMS, current_idx, 1);
                    current_idx = (current_idx + 1) % NUM_ITEMS;
                    while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break; 
                }
            }
            if((PF->PIN & BIT4) == 0) { 
                Delay_ms(50);
                if((PF->PIN & BIT4) == 0) {
                    Beep(50); 
                    UI_Menu_Scroll_Anim_Smooth("Select Function", menu_items, NUM_ITEMS, current_idx, -1);
                    current_idx = (current_idx - 1 + NUM_ITEMS) % NUM_ITEMS;
                    while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break; 
                }
            }
            if((PF->PIN & BIT5) == 0) { 
                Delay_ms(50);
                if((PF->PIN & BIT5) == 0) {
                    Beep(200); while((PF->PIN & BIT5) == 0) { Delay_ms(10); } selected = 1; break; 
                }
            }
        }

        if (selected == 1) {
            if (current_idx == 0) {
                int baud_idx = 1; 
                int baud_selected = 0;
                while(1) {
                    UI_Draw_Menu_State("Select Baud Rate", baud_items, NUM_BAUDS, baud_idx);
                    while(1) {
                        if((PF->PIN & BIT3) == 0) {
                            Delay_ms(50);
                            if((PF->PIN & BIT3) == 0) {
                                Beep(50);
                                UI_Menu_Scroll_Anim_Smooth("Select Baud Rate", baud_items, NUM_BAUDS, baud_idx, 1);
                                baud_idx = (baud_idx + 1) % NUM_BAUDS;
                                while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break;
                            }
                        }
                        if((PF->PIN & BIT4) == 0) {
                            Delay_ms(50);
                            if((PF->PIN & BIT4) == 0) {
                                Beep(50); 
                                UI_Menu_Scroll_Anim_Smooth("Select Baud Rate", baud_items, NUM_BAUDS, baud_idx, -1);
                                baud_idx = (baud_idx - 1 + NUM_BAUDS) % NUM_BAUDS;
                                while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break;
                            }
                        }
                        if((PF->PIN & BIT5) == 0) {
                            Delay_ms(50);
                            if((PF->PIN & BIT5) == 0) {
                                Beep(200); while((PF->PIN & BIT5) == 0) { Delay_ms(10); } baud_selected = 1; break;
                            }
                        }
                    }
                    if (baud_selected == 1) break; 
                }
                UART_Monitor_Test(baud_values[baud_idx]);
            } 
            else if (current_idx == 1) { Wiegand_Monitor_Test(); } 
            else if (current_idx == 2) { TK2_Monitor_Test(); } 
            else if (current_idx == 3) { Show_RTC_Time_Loop(BIT5, "     Yellow button > Back"); }
            else if (current_idx == 4) { 
                int exit_buzzer = 0;
                while(1) {
                    OLED_Clear();
                    Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "Buzzer Settings");
                    if (g_u8BuzzerEnabled) Safe_Print_OLED_Smooth(16, 16, 63, 0x0F, "  Current: ON ");
                    else Safe_Print_OLED_Smooth(16, 16, 63, 0x04, "  Current: OFF");
                    Safe_Print_OLED_Smooth(32, 16, 63, 0x08, " PF.3 -> Turn ON");
                    Safe_Print_OLED_Smooth(48, 16, 63, 0x08, " PF.4 -> Turn OFF");
                    OLED_Update();
                    
                    while(1) {
                        if((PF->PIN & BIT3) == 0) { 
                            Delay_ms(50);
                            if((PF->PIN & BIT3) == 0) {
                                g_u8BuzzerEnabled = 1; ForceBeep(100); 
                                while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break; 
                            }
                        }
                        if((PF->PIN & BIT4) == 0) { 
                            Delay_ms(50);
                            if((PF->PIN & BIT4) == 0) {
                                g_u8BuzzerEnabled = 0; 
                                while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break; 
                            }
                        }
                        if((PF->PIN & BIT5) == 0) { 
                            Delay_ms(50);
                            if((PF->PIN & BIT5) == 0) {
                                Beep(200); while((PF->PIN & BIT5) == 0) { Delay_ms(10); } exit_buzzer = 1; break; 
                            }
                        }
                    }
                    if (exit_buzzer == 1) break;
                }
            }
        }
    }
}