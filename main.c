/*
 * ===========================================================================================
 * Project: JIG-8FT-P1 _WIFIBLE (M031 端主程式 - 穩健起點版)
 * MCU: Nuvoton M032SE3AE
 * OLED: 3.2inch 256x64 mono white OLED Module (SSD1322)
 * RTC: RV-3028-C7
 * PowerMonitor: INA237 I2C Interface
 * 目前版本：v5.0.5 (2026/06/25) [滾輪選單新增 UART1 測試]
 * 1.TXRX通訊正常02頭 0D結尾
 * 2.\02\52\65\63\65\69\76\65\64\20\4F\56\45\52\38\33\0D 
 * 3.尚未規劃通訊協定Communications protocol
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
// [治具實體 GPIO 按鍵對應與功能說明]
// =======================================================
// 1. PA.8  (SW_Power)     : 電源開關 / 測試觸發 (紅色按鈕 / 電源符號)
// 2. PF.5  (SW)           : 確定或離開 (黃色按鈕 / 圓形符號)
// 3. PF.4  (SW_Change)    : 滾輪選單往上移動、數值增加 (綠色按鈕 / "+" 符號)
// 4. PF.3  (SW_SET)       : 滾輪選單往下移動、數值遞減 (藍色按鈕 / "-" 符號)
// 5. PF.6  (SW_Interface) : 原本選介面用，目前滾輪選單暫時沒用到 (黑色按鈕 / "CIF" 字樣)
// 6. PF.14 (SW_Baud rate) : 原本選包率用，目前滾輪選單暫時沒用到 (白色按鈕 / 正方形符號)
// =======================================================

// =======================================================
// [系統全域設定與變數]
// =======================================================
volatile uint8_t g_u8BuzzerEnabled = 0; 

// 滿足 Keil5 專案左側 hid_kb.c 編譯需要的連結變數，作為發送通道 2 的就緒旗標
volatile uint8_t g_u8EP2Ready = 0; 

// =======================================================
// [UART1 Ring Buffer 設定]
// =======================================================
#define UART1_RX_BUF_SIZE 256
volatile uint8_t g_u1_rx_buf[UART1_RX_BUF_SIZE];
volatile uint16_t g_u1_rx_head = 0;
volatile uint16_t g_u1_rx_tail = 0;

// 定義 TR515 通訊協定字元
#define TR515_STX 0x02
#define TR515_CR  0x0D

// [v5.0.4] 宣告新唐 USB BSP 提供的外部變數與函數
extern const S_USBD_INFO_T gsInfo;
extern void HID_ClassRequest(void);
extern void HID_Init(void);

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
// [高頻電流滑動平均濾波器與峰值追蹤]
// =======================================================
#define CURRENT_FILTER_SIZE 50 
float g_fCurrentBuffer[CURRENT_FILTER_SIZE] = {0};
uint8_t g_u8CurrentFilterIdx = 0;
uint8_t g_u8FilterFilled = 0;

float g_fCurrentAvg = 0.0f; 
float g_fMaxCurrent = 0.0f;
float g_fMinCurrent = 9999.0f;

void Push_Current_Sample(float new_current) {
    g_fCurrentBuffer[g_u8CurrentFilterIdx] = new_current;
    g_u8CurrentFilterIdx = (g_u8CurrentFilterIdx + 1) % CURRENT_FILTER_SIZE;
    if (g_u8CurrentFilterIdx == 0) g_u8FilterFilled = 1;
    
    int count = g_u8FilterFilled ? CURRENT_FILTER_SIZE : g_u8CurrentFilterIdx;
    float sum = 0;
    for (int i = 0; i < count; i++) sum += g_fCurrentBuffer[i];
    g_fCurrentAvg = (count > 0) ? (sum / count) : new_current;
}

void Reset_Current_Filter(void) {
    memset(g_fCurrentBuffer, 0, sizeof(g_fCurrentBuffer));
    g_u8CurrentFilterIdx = 0;
    g_u8FilterFilled = 0;
    g_fCurrentAvg = 0.0f;
    g_fMaxCurrent = 0.0f;
    g_fMinCurrent = 9999.0f;
}

// =======================================================
// [Function Prototypes]
// =======================================================
void SYS_Init(void);
void Setup_GPIO_Modes(void); 
void OLED_Force_Reset(void);         
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us); 
void JigForceBeep(uint32_t ms); 
void JigBeep(uint32_t ms);      
void Trigger_RedLight_Alarm(void);

int UART0_Read_Byte(uint8_t *data);
void UART0_Flush_Rx_Buffer(void);
void UART1_Flush_Rx_Buffer(void); 

void Interface_init(void); 
void UART_Monitor_Test(uint32_t u32BaudRate); 
void Wiegand_Monitor_Test(void);
void TK2_Monitor_Test(void);
void Decode_TK2_Raw(char* out_str);

void Safe_Print_OLED_Smooth(int y, int min_y, int max_y, uint8_t brightness, const char *fmt, ...);
void Safe_Print_OLED(int y, const char *fmt, ...);
void Show_RTC_Time_Loop(uint32_t exit_btn_bit, const char* hint_text);

// =======================================================
// [核心共用模組 (Helper Functions)]
// =======================================================
int Check_Exit_Button(void) {
    // PF.5 (SW): 確定或離開 (黃色按鈕 / 圓形符號)
    if((PF->PIN & BIT5) == 0) { 
        Delay_ms(50); 
        if((PF->PIN & BIT5) == 0) { JigBeep(200); while((PF->PIN & BIT5)==0){} return 1; } 
    }
    return 0;
}

int Check_Reset_Button(void) {
    // PF.3 (SW_SET): 往下移動/遞減 (藍色按鈕 / "-"符號) -> 這裡用作重置電流峰值紀錄
    if((PF->PIN & BIT3) == 0) { 
        Delay_ms(50); 
        if((PF->PIN & BIT3) == 0) { JigBeep(50); g_fMaxCurrent = 0.0f; g_fMinCurrent = 9999.0f; while((PF->PIN & BIT3)==0){} return 1; } 
    }
    return 0;
}

int Check_Power_Toggle(int *power_state) {
    // PA.8 (SW_Power): 電源開關 (紅色按鈕 / 電源符號)
    if ((PA->PIN & BIT8) == 0) { 
        Delay_ms(50);
        if ((PA->PIN & BIT8) == 0) {
            *power_state = !(*power_state);
            while ((PA->PIN & BIT8) == 0) {} Delay_ms(50); return 1;
        }
    }
    return 0;
}

void Process_Background_Sampling(int power_state, uint32_t loop_tick) {
    if (power_state && (loop_tick % 20 == 0)) {
        float sample_c = getCurrent_mA();
        if (sample_c == 0.0f) { set237Calibration_1A(); sample_c = getCurrent_mA(); }
        Push_Current_Sample(sample_c);
        if (sample_c > g_fMaxCurrent) g_fMaxCurrent = sample_c;
        if (sample_c < g_fMinCurrent) g_fMinCurrent = sample_c;
    }
}

void Show_Test_Start_Screen(const char* title) {
    OLED_Clear();
    Safe_Print_OLED(0, "%s", title); 
    Safe_Print_OLED(16, "Red Btn (Power)"); 
    Safe_Print_OLED(32, "Blue Btn (-) Reset"); 
    OLED_Update(); 

    uint32_t wait_tick = 0;
    while(wait_tick < 5000) {
        if (Check_Exit_Button()) break; // PF.5 黃色按鈕提早離開
        Delay_ms(10); wait_tick += 10;
    }
    OLED_Clear();
    Reset_Current_Filter(); 
}

void Update_Dashboard_Display(int power_state, int rx_count, const char* specific_data_str) {
    float voltage = getBusVoltage_V(); 
    float inst_current = getCurrent_mA();
    if (inst_current == 0.0f) { set237Calibration_1A(); inst_current = getCurrent_mA(); }
        
    char l_buf[17], r_buf[17];
    OLED_Clear();
    
    snprintf(l_buf, 17, "AVG:%.1fmA", g_fCurrentAvg);
    snprintf(r_buf, 17, "Max:%.1fmA", g_fMaxCurrent);
    Safe_Print_OLED(0, "%-16s%s", l_buf, r_buf); 
    
    snprintf(l_buf, 17, "CUR:%.1fmA", inst_current);
    snprintf(r_buf, 17, "Min:%.0fmA", (g_fMinCurrent==9999.0f)?0:g_fMinCurrent);
    Safe_Print_OLED(16, "%-16s%s", l_buf, r_buf);
    
    snprintf(l_buf, 17, "%.2fV", voltage);
    Safe_Print_OLED(32, "%-16s[Power:%s]", l_buf, power_state?"ON ":"OFF");
    
    char r_data[32];
    snprintf(l_buf, 17, "               "); // 清空掃描機保留位
    if(strlen(specific_data_str) > 0 && rx_count >= 0) {
        snprintf(r_data, sizeof(r_data), "%02d/%s", rx_count, specific_data_str);
    } else {
        strncpy(r_data, specific_data_str, sizeof(r_data));
    }
    snprintf(r_buf, 17, "%s", r_data);
    Safe_Print_OLED(48, "%-16s%s", l_buf, r_buf); 
    
    OLED_Update(); 
}


// UART1 讀取單個字元
int UART1_Read_Byte(uint8_t *data) {
    if (g_u1_rx_head == g_u1_rx_tail) return 0;
    *data = g_u1_rx_buf[g_u1_rx_tail]; 
    g_u1_rx_tail = (g_u1_rx_tail + 1) % UART1_RX_BUF_SIZE; 
    return 1;
}

// 簡單的 UART1 發送字串函式
void UART1_Send_String(const char* str) {
    while(*str) {
        UART_WRITE(UART1, *str++);
        while(UART1->FIFOSTS & UART_FIFOSTS_TXFULL_Msk); // 等待TX FIFO緩衝區有空間
    }
}

// Command Handler：處理驗證過的 TR515 指令
void TR515_Command_Handler(const char* cmd) {
    JigBeep(100); // 接收成功嗶一聲
    OLED_Clear();
    Safe_Print_OLED(0, "--- UART1 RX ---");
    Safe_Print_OLED(16, "Format: TR515");
    Safe_Print_OLED(32, "Cmd: %s", cmd);
    OLED_Update();
}

// Parser：處理 Ring Buffer，過濾 STX 與 CR
void Process_UART1_TR515_Parser(void) {
    static uint8_t rx_packet[64];
    static uint8_t rx_idx = 0;
    static uint8_t is_stx_received = 0;
    uint8_t c;

    while(UART1_Read_Byte(&c)) {
        if (c == TR515_STX) {
            is_stx_received = 1;
            rx_idx = 0; // 重置指標，開始接收新封包
        } 
        else if (c == TR515_CR && is_stx_received) {
            rx_packet[rx_idx] = '\0'; // 補上字串結尾
            TR515_Command_Handler((char*)rx_packet); // 呼叫 Handler
            is_stx_received = 0; // 完成一次封包，重置狀態
        } 
        else if (is_stx_received) {
            if (rx_idx < sizeof(rx_packet) - 1) {
                rx_packet[rx_idx++] = c;
            } else {
                // 防呆：超過緩衝區長度，捨棄異常封包
                is_stx_received = 0; 
            }
        }
    }
}
// =======================================================
// [v5.0.4] USB HID Keyboard 輸出底層函式
// =======================================================
int Trigger_USB_HID_Key(uint8_t mod, uint8_t key) {
    uint8_t report[8] = {0};
    report[0] = mod;
    report[2] = key;
    
    uint32_t timeout = 0;
    while(g_u8EP2Ready == 0) {
        Delay_us(100);
        timeout++;
        // 防呆機制：若電腦沒準備好或沒插 USB，等待 50ms 後脫離避免卡死
        if(timeout > 500) return 0; 
    }
    
    g_u8EP2Ready = 0;
    // 將鍵盤封包寫入 USB 緩衝區並觸發發送 (EP2 為新唐鍵盤範例發送通道)
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), report, 8);
    USBD_SET_PAYLOAD_LEN(EP2, 8);
		
		return 1; // [修改] 回傳 1 表示成功
		
}


void Output_String_To_USB_HID(const char* str) {
    while(*str) {
        char c = *str;
        uint8_t mod = 0, key = 0;
        
        // 字母 (大寫需加上 Shift 修飾鍵 mod=0x02)
        if (c >= 'a' && c <= 'z') { key = c - 'a' + 0x04; }
        else if (c >= 'A' && c <= 'Z') { mod = 0x02; key = c - 'A' + 0x04; } 
        // 數字
        else if (c >= '1' && c <= '9') { key = c - '1' + 0x1E; }
        else if (c == '0') { key = 0x27; }
        // 特殊符號 (減號、底線)
        else if (c == '-') { key = 0x2D; }
        else if (c == '_') { mod = 0x02; key = 0x2D; } // Shift + '-' = '_'
        else if (c == ' ') { key = 0x2C; }
        
        if (key != 0) {
						// [修改] 如果發送失敗 (Timeout)，立刻中斷整個字串的發送迴圈
            if (!Trigger_USB_HID_Key(mod, key)) break;  //按下
            Delay_ms(2);
            if (!Trigger_USB_HID_Key(0, 0)) break;      //放開
            Delay_ms(2);
        }
        str++;
    }
}

// =======================================================
// [OLED UI 繪圖底層引擎] 
// =======================================================
void Safe_Print_OLED_Smooth(int y, int min_y, int max_y, uint8_t brightness, const char *fmt, ...) {
    char temp_buf[128]; char full_line[33]; va_list argptr;
    va_start(argptr, fmt); vsnprintf(temp_buf, sizeof(temp_buf), fmt, argptr); va_end(argptr);
    int temp_len = strlen(temp_buf);
    for(int i = 0; i < 32; i++) {
        if(i < temp_len) { char c = temp_buf[i]; if(c < 0x20 || c > 0x7E) c = ' '; full_line[i] = c; } else { full_line[i] = ' '; }
    }
    full_line[32] = '\0'; OLED_PrintString(0, y, min_y, max_y, full_line, brightness);
}

void Safe_Print_OLED(int y, const char *fmt, ...) {
    char temp_buf[128]; char full_line[33]; va_list argptr;
    va_start(argptr, fmt); vsnprintf(temp_buf, sizeof(temp_buf), fmt, argptr); va_end(argptr);
    int temp_len = strlen(temp_buf);
    for(int i = 0; i < 32; i++) {
        if(i < temp_len) { char c = temp_buf[i]; if(c < 0x20 || c > 0x7E) c = ' '; full_line[i] = c; } else { full_line[i] = ' '; }
    }
    full_line[32] = '\0'; OLED_PrintString(0, y, 0, 63, full_line, 0x0F);
}

void UI_Draw_Menu_State(const char* title, const char** items, int num_items, int curr_idx) {
    int prev_idx = (curr_idx - 1 + num_items) % num_items;
    int next_idx = (curr_idx + 1) % num_items;
    OLED_Clear();
    Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, title);                  
    Safe_Print_OLED_Smooth(16, 16, 63, 0x04, "  %s", items[prev_idx]); 
    Safe_Print_OLED_Smooth(32, 16, 63, 0x0F, "> %s", items[curr_idx]); 
    Safe_Print_OLED_Smooth(48, 16, 63, 0x04, "  %s", items[next_idx]); 
    OLED_Update(); 
}

void UI_Menu_Scroll_Anim_Smooth(const char* title, const char** items, int num_items, int old_idx, int dir) {
    int p_idx = (old_idx - 1 + num_items) % num_items;
    int n_idx = (old_idx + 1) % num_items;
    int nn_idx = (old_idx + 2) % num_items; 
    int pp_idx = (old_idx - 2 + num_items * 2) % num_items; 

    for (int offset = 0; offset <= 16; offset += 4) {
        OLED_Clear(); Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, title); 
        if (dir == 1) { 
            Safe_Print_OLED_Smooth(16 - offset, 16, 63, 0x02, "  %s", items[p_idx]); 
            Safe_Print_OLED_Smooth(32 - offset, 16, 63, 0x0F - (offset/2), "  %s", items[old_idx]);
            Safe_Print_OLED_Smooth(48 - offset, 16, 63, 0x04 + (offset/2), "> %s", items[n_idx]);
            Safe_Print_OLED_Smooth(64 - offset, 16, 63, 0x02, "  %s", items[nn_idx]); 
        } else { 
            Safe_Print_OLED_Smooth(0 + offset, 16, 63, 0x02, "  %s", items[pp_idx]); 
            Safe_Print_OLED_Smooth(16 + offset, 16, 63, 0x04 + (offset/2), "> %s", items[p_idx]);
            Safe_Print_OLED_Smooth(32 + offset, 16, 63, 0x0F - (offset/2), "  %s", items[old_idx]);
            Safe_Print_OLED_Smooth(48 + offset, 16, 63, 0x02, "  %s", items[n_idx]);
        }
        OLED_Update(); 
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
    
    // UART1 (PB.2 RX, PB.3 TX) - 為 ESP32 預留
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
    
    // 初始化 6 顆主要功能按鍵為輸入模式 (QUASI 模式)
    GPIO_SetMode(PB, BIT0, GPIO_MODE_QUASI); 
    GPIO_SetMode(PA, BIT1, GPIO_MODE_QUASI); 
    GPIO_SetMode(PD, BIT2, GPIO_MODE_QUASI); 
    GPIO_SetMode(PD, BIT1, GPIO_MODE_QUASI); 
    GPIO_SetMode(PD, BIT0, GPIO_MODE_QUASI); 
    
    // [註解優化] PA.8_SW_Power 紅色按鍵 (電源)
    GPIO_SetMode(PA, BIT8, GPIO_MODE_QUASI);
    
    // [註解優化] PF.6_SW_Interface 黑色按鍵 (CIF) / PF.14_SW_Baud_rate 白色按鍵 (方塊)
    GPIO_SetMode(PF, BIT6, GPIO_MODE_QUASI); 
    GPIO_SetMode(PF, BIT14, GPIO_MODE_QUASI);
    
    // [註解優化] PF.5_SW 黃色按鍵 (圓形/確認) / PF.4_SW_Change 綠色按鍵 (+) / PF.3_SW_SET 藍色按鍵 (-)
    GPIO_SetMode(PF, BIT5, GPIO_MODE_QUASI); 
    GPIO_SetMode(PF, BIT3, GPIO_MODE_QUASI); 
    GPIO_SetMode(PF, BIT4, GPIO_MODE_QUASI);
    
    GPIO_SetMode(PC, BIT7, GPIO_MODE_OUTPUT); PC->DOUT &= ~BIT7; 
    PD->DOUT |= BIT15; PF->DOUT |= BIT15; PC->DOUT |= BIT1;  PD->DOUT |= BIT3;  PA->DOUT |= BIT8;
    PF->DOUT |= (BIT6 | BIT14 | BIT5 | BIT3 | BIT4);

    // [v5.0.4] USB 腳位初始化 (PA.12 VBUS, PA.13 D-, PA.14 D+)
    // 使用絕對數值 (14ul) 避開不同版本 BSP 巨集名稱的歧異
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk);
    SYS->GPA_MFPH |= ((14ul << SYS_GPA_MFPH_PA12MFP_Pos) | (14ul << SYS_GPA_MFPH_PA13MFP_Pos) | (14ul << SYS_GPA_MFPH_PA14MFP_Pos));
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
    
    // [v5.0.4] 啟動 USB 模組時脈 (M032 使用內部 HIRC 即可)
    CLK_EnableModuleClock(USBD_MODULE);
    
    CLK->AHBCLK |= ((1ul << 0)|(1ul << 1)|(1ul << 2)|(1ul << 3)|(1ul << 5)); 
    CLK_EnableModuleClock(SPI0_MODULE); CLK_EnableModuleClock(I2C0_MODULE); CLK_EnableModuleClock(I2C1_MODULE); 
    CLK_EnableModuleClock(UART0_MODULE); CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_EnableModuleClock(UART1_MODULE); CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    CLK_EnableModuleClock(UART2_MODULE); CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
    CLK_EnableModuleClock(TMR0_MODULE); CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    SystemCoreClockUpdate(); WIFIBLE_ReaderTest_init();
    
    UART_Open(UART0, 115200); 
    UART0->FIFO = (UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE; 
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
    
		// 原本的程式碼：
    UART_Open(UART1, 115200); 
    UART1->FIFO = (UART1->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;
    
    // 【請補上這兩行來開啟中斷】：開啟 UART1 中斷
    UART_EnableInt(UART1, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    
    // 【修改這裡】M031 必須使用 UART13_IRQn
    NVIC_EnableIRQ(UART13_IRQn);
    
    UART_Open(UART2, 9600); 
    UART2->FIFO = (UART2->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE; 
    UART_EnableInt(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    
    I2C_Open(I2C0, 100000); 
    NVIC_EnableIRQ(UART02_IRQn); 
    SYS_LockReg();
}

// =======================================================
// [中斷處理模組]
// =======================================================
void UART13_IRQHandler(void) {
    uint32_t u32IntSts = UART1->INTSTS;
    uint32_t u32FIFOSts = UART1->FIFOSTS;
    
    // [優化] 多加了 UART_FIFOSTS_BIF_Msk (Break Interrupt)，防止傳輸線雜訊引發中斷卡死
    if(u32FIFOSts & (UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_RXOVIF_Msk)) { 
        UART1->FIFOSTS = (UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_RXOVIF_Msk); 
    }
    
    if(u32IntSts & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
        while((UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) {
            uint8_t c = UART_READ(UART1);
            uint16_t next_head = (g_u1_rx_head + 1) % UART1_RX_BUF_SIZE;
            
            // 就算在主選單沒有人去收資料 (Buffer 滿了)，這裡也會把硬體 FIFO 讀空丟棄，不會卡當
            if (next_head != g_u1_rx_tail) { 
                g_u1_rx_buf[g_u1_rx_head] = c; 
                g_u1_rx_head = next_head; 
            }
        }
    }
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

void UART1_TR515_Test(void) {
    int power_state = 0;
    OLED_Clear();
    Safe_Print_OLED(0, "UART1 TR515 Test");
    Safe_Print_OLED(16, "Red(PA8): TX ABCD123");
    Safe_Print_OLED(32, "Wait RX (abc456)...");
    Safe_Print_OLED(48, "Yellow(PF5): Exit");
    OLED_Update();
    
    // 清空緩衝區
    __disable_irq(); g_u1_rx_head = 0; g_u1_rx_tail = 0; __enable_irq();

    while(1) {
        // 1. 定期呼叫 Parser 處理接收資料
        Process_UART1_TR515_Parser();

        // 2. 監聽紅色按鍵 (PA.8) 送出資料
        if ((PA->PIN & BIT8) == 0) { 
            Delay_ms(50);
            if ((PA->PIN & BIT8) == 0) {
                JigForceBeep(50);
                
                // 送出 TR515 格式：<STX>ABCD123<CR>
                UART_WRITE(UART1, TR515_STX);
                UART1_Send_String("ABCD123");
                UART_WRITE(UART1, TR515_CR);

                OLED_Clear();
                Safe_Print_OLED(0, "--- UART1 TX ---");
                Safe_Print_OLED(16, "Sent: ABCD123");
                OLED_Update();
                
                while ((PA->PIN & BIT8) == 0); // 等待按鍵放開
                Delay_ms(1000); // 顯示一秒後恢復畫面
                
                OLED_Clear();
                Safe_Print_OLED(0, "UART1 TR515 Test");
                Safe_Print_OLED(16, "Ready...");
                OLED_Update();
            }
        }

        // 3. 監聽黃色按鍵 (PF.5) 離開
        if(Check_Exit_Button()) break;
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

void OLED_Force_Reset(void) { 
    PD->DOUT |= BIT15; Delay_ms(50); 
    PD->DOUT &= ~BIT15; Delay_ms(200); 
    PD->DOUT |= BIT15; Delay_ms(200); 
}

// =======================================================
// [音訊分流系統]
// =======================================================
void JigForceBeep(uint32_t ms) { 
    uint32_t half_period_us = 185; uint32_t total_cycles = (ms * 1000) / (half_period_us * 2);
    for (uint32_t i = 0; i < total_cycles; i++) { PB15 = 1; Delay_us(half_period_us); PB15 = 0; Delay_us(half_period_us); } PB15 = 0; Delay_ms(50); 
}

void JigBeep(uint32_t ms) {
    if (g_u8BuzzerEnabled == 1) { 
        JigForceBeep(ms);
    }
}

void Trigger_RedLight_Alarm(void) { 
    PC->DOUT |= BIT7; 
    OLED_Clear(); Safe_Print_OLED(16, "COMM ERROR ALARM"); OLED_Update();
    while(1) { JigForceBeep(500); Delay_ms(500); }
}

void Show_RTC_Time_Loop(uint32_t exit_btn_bit, const char* hint_text) {
    RTC_TimeTypeDef current_time; 
    while(1) {
        OLED_Clear(); 
        RV3028_GetTime(&current_time);
        Safe_Print_OLED(0,  "    --- RTC CURRENT TIME ---");
        Safe_Print_OLED(16, "           %04d/%02d/%02d", current_time.year, current_time.month, current_time.date);
        Safe_Print_OLED(32, "            %02d:%02d:%02d", current_time.hours, current_time.minutes, current_time.seconds);
        Safe_Print_OLED(48, hint_text);
        OLED_Update(); 
        
        // 使用 PF.5 黃色按鈕離開
        if((PF->PIN & exit_btn_bit) == 0) { 
            Delay_ms(50); 
            if((PF->PIN & exit_btn_bit) == 0) { 
                JigBeep(100); 
                while((PF->PIN & exit_btn_bit) == 0) { Delay_ms(10); } 
                break; 
            } 
        }
        Delay_ms(100); 
    }
}

// =======================================================
// [測試介面 1] UART2 監控
// =======================================================
void UART_Monitor_Test(uint32_t u32BaudRate) {
    UART_DisableInt(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    Interface_init(); PB4 = 1;               
    UART_Open(UART2, u32BaudRate); UART2->FIFO = (UART2->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;
    UART_EnableInt(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    
    char title_buf[32]; snprintf(title_buf, sizeof(title_buf), "UART2 Mntr %u", u32BaudRate);
    Show_Test_Start_Screen(title_buf); 

    int rx_count = 0; char rx_buf[128] = {0}; uint32_t loop_tick = 1000; int power_state = 0;

    while(1) {
        if (Check_Power_Toggle(&power_state)) {
            if (power_state) { 
                PC->DOUT |= BIT7; JigBeep(100); 
                UART2->FIFO |= UART_FIFO_RXRST_Msk; UART2->FIFOSTS = (UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk);
                __disable_irq(); g_u2_rx_head = 0; g_u2_rx_tail = 0; __enable_irq();
            } else { 
                PC->DOUT &= ~BIT7; JigBeep(500); 
            }
            Reset_Current_Filter(); loop_tick = 1000;
        }

        Process_Background_Sampling(power_state, loop_tick);
        if (Check_Reset_Button()) loop_tick = 1000;

        if (power_state && (UART2->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) {
            int rx_len = 0; int rx_to = 0; memset(rx_buf, 0, sizeof(rx_buf));
            while(rx_len < 127) { 
                if((UART2->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) { 
                    char c = UART_READ(UART2); if(c >= 0x20 && c <= 0x7E) rx_buf[rx_len++] = c; else rx_buf[rx_len++] = '.'; rx_to = 0; 
                } else { Delay_ms(1); rx_to++; if(rx_to > 100) break; }
            }
            if (rx_len >= 4) { rx_buf[rx_len] = '\0'; rx_count++; JigBeep(50); loop_tick = 1000; }
        }
        
        if (Check_Exit_Button()) break;
        
        if (loop_tick >= 1000) { 
            Update_Dashboard_Display(power_state, rx_count, rx_buf);
            loop_tick = 0; 
        }
        Delay_ms(1); loop_tick++;
    }
    PC->DOUT &= ~BIT7; OLED_Clear(); Safe_Print_OLED(0, "Monitor End"); OLED_Update(); Delay_ms(1000);
}

// =======================================================
// [測試介面 2] Wiegand 監控 
// =======================================================
void Wiegand_Monitor_Test(void) {
    Interface_init(); PB6 = 1; PA11 = 1; 
    GPIO_SetMode(PA, BIT10, GPIO_MODE_QUASI); GPIO_SetMode(PB, BIT5, GPIO_MODE_QUASI); GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5);
    Show_Test_Start_Screen("Wiegand Monitor"); 

    int rx_count = 0; uint32_t loop_tick = 1000; int power_state = 0; 
    uint64_t last_wg_data = 0; char data_str[32] = {0};

    while(1) {
        if (Check_Power_Toggle(&power_state)) {
            if (power_state) { 
                PC->DOUT |= BIT7; JigBeep(100); 
                QUEUE_CLEAR(au64WG1); GPIO_CLR_INT_FLAG(PA, BIT10); GPIO_CLR_INT_FLAG(PB, BIT5);
                GPIO_EnableInt(PA, 10, GPIO_INT_FALLING); GPIO_EnableInt(PB, 5, GPIO_INT_FALLING);  NVIC_EnableIRQ(GPIO_PAPBPGPH_IRQn); 
            } else { 
                GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5); PC->DOUT &= ~BIT7; JigBeep(500); 
            }
            Reset_Current_Filter(); loop_tick = 1000;
        }
        
        Process_Background_Sampling(power_state, loop_tick);
        if (Check_Reset_Button()) loop_tick = 1000;

        vCheckingTimeOut();
        if (!QUEUE_EMPTY(au64WG1)) { 
            last_wg_data = QUEUE_PULL(au64WG1); 
            if (g_u8WiegandNum > 0) { rx_count++; JigBeep(50); loop_tick = 1000; }
        }
        
        if (Check_Exit_Button()) break;
        
        if (loop_tick >= 1000) {
            if (rx_count > 0) snprintf(data_str, sizeof(data_str), "W%02d:%llX", g_u8WiegandNum, last_wg_data);
            else strcpy(data_str, "WAITING...");
            Update_Dashboard_Display(power_state, rx_count, data_str);
            loop_tick = 0; 
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

// =======================================================
// [測試介面 3] TK2 監控
// =======================================================
void TK2_Monitor_Test(void) {
    Interface_init(); PB7 = 1; PA11 = 1; 
    GPIO_SetMode(PA, BIT10, GPIO_MODE_QUASI); GPIO_SetMode(PB, BIT5, GPIO_MODE_QUASI); GPIO_SetMode(PB, BIT8, GPIO_MODE_QUASI); 
    GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5);  GPIO_DisableInt(PB, 8);
    Show_Test_Start_Screen("TK2 Monitor");
    
    int rx_count = 0; uint32_t loop_tick = 1000; int power_state = 0;
    uint8_t last_tk2_cnt = 0; uint32_t tk2_idle_tick = 0; char tk2_str[64] = "WAITING...";

    while(1) {
        if (Check_Power_Toggle(&power_state)) {
            if (power_state) { 
                PC->DOUT |= BIT7; JigBeep(100); 
                TK2Cnt = 0; last_tk2_cnt = 0; tk2_idle_tick = 0; g_u8TK2Step = 0;
                memset((void *)g_u8TK2Bit, 0, sizeof(g_u8TK2Bit)); GPIO_CLR_INT_FLAG(PB, BIT5); 
                GPIO_EnableInt(PB, 5, GPIO_INT_FALLING);  NVIC_EnableIRQ(GPIO_PAPBPGPH_IRQn); 
            } else { 
                GPIO_DisableInt(PB, 5); PC->DOUT &= ~BIT7; JigBeep(500); 
            }
            Reset_Current_Filter(); loop_tick = 1000;
        }
        
        Process_Background_Sampling(power_state, loop_tick);
        if (Check_Reset_Button()) loop_tick = 1000;

        vCheckingTimeOut();
        if (TK2Cnt > 0) {
            if (TK2Cnt != last_tk2_cnt) { last_tk2_cnt = TK2Cnt; tk2_idle_tick = 0; } 
            else {
                tk2_idle_tick++;
                if (tk2_idle_tick > 50) { 
                    rx_count++; Decode_TK2_Raw(tk2_str); 
                    JigBeep(50); loop_tick = 1000; 
                    TK2Cnt = 0; last_tk2_cnt = 0; g_u8TK2Step = 0; memset((void *)g_u8TK2Bit, 0, sizeof(g_u8TK2Bit));
                }
            }
        }
        
        if (Check_Exit_Button()) break;
        
        if (loop_tick >= 1000) {
            Update_Dashboard_Display(power_state, rx_count, tk2_str);
            loop_tick = 0; 
        }
        Delay_ms(1); loop_tick++;
    }
    PC->DOUT &= ~BIT7; GPIO_DisableInt(PB, 5); OLED_Clear(); Safe_Print_OLED(0, "Monitor End"); OLED_Update(); Delay_ms(1000);
}

// =======================================================
// Main 主程式
// =======================================================
int main(void) {
    SYS_Init();
    Setup_GPIO_Modes();
    
    // 等待硬體穩定，防冷開機 OLED 黑屏
    Delay_ms(500); 
    
    // [v5.0.4] 啟動 USBD 功能，完成電腦端枚舉 (Enumeration)
    USBD_Open(&gsInfo, HID_ClassRequest, NULL);
    HID_Init();
    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();
    
    OLED_Force_Reset();
    vOLED_INIT();
    vINA237_Init();
    set237Calibration_1A();
    RV3028_Init();
    
    OLED_Clear();
    Safe_Print_OLED(0, "System Ready"); 
    Safe_Print_OLED(16, "JIG-8FT-P1 OK"); 
    OLED_Update();
    
    JigBeep(500); Delay_ms(100); JigBeep(500); 
    Delay_ms(1000);
    
// ---主選單定義，修改為 7 個選項 ---
    const char *menu_items[] = { "RS232 Monitor", "Wiegand", "TK2", "What's the time?", "Buzzer Settings", "USBHID TEST", "UART1 TR515" };
    const int NUM_ITEMS = sizeof(menu_items) / sizeof(menu_items[0]); // 自動計算為 7
    int current_idx = 1; 

    const char *baud_items[] = { "115200, N, 8, 1", "9600, N, 8, 1", "19200, N, 8, 1", "38400, N, 8, 1" };
    const uint32_t baud_values[] = { 115200, 9600, 19200, 38400 };
    const int NUM_BAUDS = sizeof(baud_items) / sizeof(baud_items[0]);

    while(1) {
        UI_Draw_Menu_State("Select Function", menu_items, NUM_ITEMS, current_idx);
        int selected = 0;

        while(1) {
            // 監聽 PF.3 (SW_SET): 藍色按鈕 / "-"符號 -> 往下捲動選單
            if((PF->PIN & BIT3) == 0) { 
                Delay_ms(50);
                if((PF->PIN & BIT3) == 0) {
                    JigBeep(50); UI_Menu_Scroll_Anim_Smooth("Select Function", menu_items, NUM_ITEMS, current_idx, 1);
                    current_idx = (current_idx + 1) % NUM_ITEMS;
                    while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break; 
                }
            }
            
            // 監聽 PF.4 (SW_Change): 綠色按鈕 / "+"符號 -> 往上捲動選單
            if((PF->PIN & BIT4) == 0) { 
                Delay_ms(50);
                if((PF->PIN & BIT4) == 0) {
                    JigBeep(50); UI_Menu_Scroll_Anim_Smooth("Select Function", menu_items, NUM_ITEMS, current_idx, -1);
                    current_idx = (current_idx - 1 + NUM_ITEMS) % NUM_ITEMS;
                    while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break; 
                }
            }
            
            // 監聽 PF.5 (SW): 黃色按鈕 / 圓形符號 -> 確認進入選項
            if((PF->PIN & BIT5) == 0) { 
                Delay_ms(50);
                if((PF->PIN & BIT5) == 0) {
                    JigBeep(200); while((PF->PIN & BIT5) == 0) { Delay_ms(10); } selected = 1; break; 
                }
            }
        }

        if (selected == 1) {
            if (current_idx == 0) {
                int baud_idx = 1; int baud_selected = 0;
                while(1) {
                    UI_Draw_Menu_State("Select Baud Rate", baud_items, NUM_BAUDS, baud_idx);
                    while(1) {
                        if((PF->PIN & BIT3) == 0) { Delay_ms(50); if((PF->PIN & BIT3) == 0) { JigBeep(50); UI_Menu_Scroll_Anim_Smooth("Select Baud Rate", baud_items, NUM_BAUDS, baud_idx, 1); baud_idx = (baud_idx + 1) % NUM_BAUDS; while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break; } }
                        if((PF->PIN & BIT4) == 0) { Delay_ms(50); if((PF->PIN & BIT4) == 0) { JigBeep(50); UI_Menu_Scroll_Anim_Smooth("Select Baud Rate", baud_items, NUM_BAUDS, baud_idx, -1); baud_idx = (baud_idx - 1 + NUM_BAUDS) % NUM_BAUDS; while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break; } }
                        if(Check_Exit_Button()) { baud_selected = 1; break; }
                    }
                    if (baud_selected == 1) break; 
                }
                UART_Monitor_Test(baud_values[baud_idx]);
            } 
            else if (current_idx == 1) { Wiegand_Monitor_Test(); } 
            else if (current_idx == 2) { TK2_Monitor_Test(); } 
            else if (current_idx == 3) { Show_RTC_Time_Loop(BIT5, "    Yellow(Exit) > Back"); }
            else if (current_idx == 4) { 
                int exit_buzzer = 0;
                while(1) {
                    OLED_Clear(); Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "Buzzer Settings");
                    if (g_u8BuzzerEnabled) Safe_Print_OLED_Smooth(16, 16, 63, 0x0F, "  Current: ON ");
                    else Safe_Print_OLED_Smooth(16, 16, 63, 0x04, "  Current: OFF");
                    
                    // 優化 OLED 提示對應顏色與符號
                    Safe_Print_OLED_Smooth(32, 16, 63, 0x08, " Blue(-) -> Turn ON"); 
                    Safe_Print_OLED_Smooth(48, 16, 63, 0x08, " Green(+) -> Turn OFF");
                    OLED_Update();
                    
                    while(1) {
                        // PF.3 (SW_SET): 藍色按鈕 / "-"符號 -> 打開蜂鳴器
                        if((PF->PIN & BIT3) == 0) { Delay_ms(50); if((PF->PIN & BIT3) == 0) { g_u8BuzzerEnabled = 1; JigForceBeep(100); while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break; } }
                        
                        // PF.4 (SW_Change): 綠色按鈕 / "+"符號 -> 關閉蜂鳴器
                        if((PF->PIN & BIT4) == 0) { Delay_ms(50); if((PF->PIN & BIT4) == 0) { g_u8BuzzerEnabled = 0; while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break; } }
                        
                        // PF.5 (SW): 黃色按鈕 / 圓形符號 -> 離開
                        if(Check_Exit_Button()) { exit_buzzer = 1; break; }
                    }
                    if (exit_buzzer == 1) break;
                }
            }
            else if (current_idx == 5) {
                // [v5.0.4] 新增的 USBHID 獨立測試區塊
                int exit_usb_test = 0;
                while(1) {
                    OLED_Clear();
                    Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "USBHID TEST");
                    Safe_Print_OLED_Smooth(16, 16, 63, 0x0A, " Red Btn(Power) -> Send");
                    Safe_Print_OLED_Smooth(32, 16, 63, 0x0A, " Yellow(Exit)   -> Back");
                    OLED_Update();
                    
                    while(1) {
                        // 監聽 PA.8 紅色按鍵發送字串
                        if((PA->PIN & BIT8) == 0) { 
                            Delay_ms(50); 
                            if((PA->PIN & BIT8) == 0) { 
                                JigForceBeep(100); 
                                OLED_Clear();
                                Safe_Print_OLED_Smooth(16, 16, 63, 0x0F, " Sending via USB...");
                                OLED_Update();
                                
                                // 發送自定義測試字串
                                Output_String_To_USB_HID("BALLY-chou_test_0429");
                                
                                OLED_Clear();
                                Safe_Print_OLED_Smooth(16, 16, 63, 0x0F, " Send Success!");
                                OLED_Update();
                                Delay_ms(1000);
                                
                                while((PA->PIN & BIT8) == 0) { Delay_ms(10); } 
                                break; // 跳出重新刷選單畫面
                            } 
                        }
                        
                        // 監聽 PF.5 黃色按鍵離開
                        if(Check_Exit_Button()) { exit_usb_test = 1; break; }
                    }
                    if (exit_usb_test == 1) break;
                }
            }
						else if (current_idx == 6) {
                // 執行新增的 UART1 獨立測試函式
                UART1_TR515_Test();
            }
        }
    }
}