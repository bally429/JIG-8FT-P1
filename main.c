/*
 * ===========================================================================================
 * Project: JIG-8FT-P1 _WIFIBLE (M031 端主程式)
 * MCU: Nuvoton M032SE3AE
 * OLED: 3.2inch 256x64 mono white OLED Module (SSD1322)
 * RTC: RV-3028-C7
 * PowerMonitor: INA237 I2C Interface
 * * [版本履歷]
 * V5.3.0 (2026/06/30): 引入 TMR1 1ms 背景任務、全域 UART1 SC 無感轉發、多工鬧鐘與碼表。
 * V5.3.1 (2026/06/30): 補回遺漏的 UI 繪圖與儀表板更新函式，修復編譯錯誤。
 * V5.3.2 (2026/06/30): [重大架構升級] USB HID 全面改為「非同步佇列 (Ring Buffer) 背景發送」，
 * 徹底解決在特定介面 (如 TK2, Wiegand) 中掃描條碼導致卡死或無輸出的問題。
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
// [系統全域設定與變數]
// =======================================================
volatile uint8_t g_u8BuzzerEnabled = 0; 
volatile uint8_t g_u8EP2Ready = 0;
extern volatile uint8_t g_u8Led_Status[8]; 
volatile uint8_t g_u8UsbHidAppendCR = 1; 
volatile uint8_t g_u8UsbHidSmartCaps = 1; 

// TMR1 硬體系統計時 (1ms) 與背景狀態控制
volatile uint32_t g_u32SystemMs = 0;
volatile uint32_t g_u32StopwatchMs = 0;
volatile uint8_t g_u8StopwatchRunning = 0;
volatile uint8_t g_force_alarm_menu = 0; 

// =======================================================
// [V5.3.2] USB HID 專用非同步發送緩衝區 (Ring Buffer)
// =======================================================
#define HID_TX_BUF_SIZE 256
volatile uint8_t g_hid_tx_buf[HID_TX_BUF_SIZE];
volatile uint16_t g_hid_head = 0;
volatile uint16_t g_hid_tail = 0;

// =======================================================
// [UART Ring Buffer 設定]
// =======================================================
#define UART1_RX_BUF_SIZE 256
volatile uint8_t g_u1_rx_buf[UART1_RX_BUF_SIZE];
volatile uint16_t g_u1_rx_head = 0;
volatile uint16_t g_u1_rx_tail = 0;

#define RX0_BUF_SIZE 128
volatile uint8_t g_u0_rx_buf[RX0_BUF_SIZE];
volatile uint16_t g_u0_rx_head = 0;
volatile uint16_t g_u0_rx_tail = 0;

#define UART2_RX_BUF_SIZE 256
volatile uint8_t g_u2_rx_buf[UART2_RX_BUF_SIZE];
volatile uint16_t g_u2_rx_head = 0;
volatile uint16_t g_u2_rx_tail = 0;

#define JIG_8CP_STX 0x02
#define JIG_8CP_CR  0x0D

// =======================================================
// [鬧鐘結構設定]
// =======================================================
typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t enabled;
} AlarmDef;
AlarmDef g_alarms[6] = {0};
volatile uint8_t g_alarm_triggered = 0;

// =======================================================
// [外部函數與共用模組宣告]
// =======================================================
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

// =======================================================
// [所有函數原型宣告 - Prototypes] 
// =======================================================
void Process_UART1_JIG_8CP_Parser(void);
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us); 
void JigForceBeep(uint32_t ms); 
void JigBeep(uint32_t ms);      
void Safe_Print_OLED_Smooth(int y, int min_y, int max_y, uint8_t brightness, const char *fmt, ...);
void Safe_Print_OLED(int y, const char *fmt, ...);
void Time_Set_Menu_Loop(void);
void Global_Background_Tasks(void);
void Show_Test_Start_Screen(const char* title);
void Update_Dashboard_Display(int power_state, int rx_count, const char* specific_data_str);
void UI_Draw_Menu_State(const char* title, const char** items, int num_items, int curr_idx);
void UI_Menu_Scroll_Anim_Smooth(const char* title, const char** items, int num_items, int old_idx, int dir);
int Check_Exit_Button(void);

// [V5.3.2 新增宣告]
void USBHID_Enqueue_Data(const char* str);
void USBHID_Enqueue_String(const char* str);
void USBHID_Process_Queue(void);
void Internal_Send_Char_HID(char c);

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
    g_u8CurrentFilterIdx = 0; g_u8FilterFilled = 0;
    g_fCurrentAvg = 0.0f; g_fMaxCurrent = 0.0f; g_fMinCurrent = 9999.0f;
}

// =======================================================
// [核心按鍵與共用 UI 模組]
// =======================================================
int Check_Exit_Button(void) {
    if((PF->PIN & BIT5) == 0) { Delay_ms(50); if((PF->PIN & BIT5) == 0) { JigBeep(200); while((PF->PIN & BIT5)==0){} return 1; } } return 0;
}
int Check_Reset_Button(void) {
    if((PF->PIN & BIT3) == 0) { Delay_ms(50); if((PF->PIN & BIT3) == 0) { JigBeep(50); g_fMaxCurrent = 0.0f; g_fMinCurrent = 9999.0f; while((PF->PIN & BIT3)==0){} return 1; } } return 0;
}
int Check_Power_Toggle(int *power_state) {
    if ((PA->PIN & BIT8) == 0) { Delay_ms(50); if ((PA->PIN & BIT8) == 0) { *power_state = !(*power_state); while ((PA->PIN & BIT8) == 0) {} Delay_ms(50); return 1; } } return 0;
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
        Global_Background_Tasks(); if (g_force_alarm_menu) break;
        if (Check_Exit_Button()) break; 
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
    snprintf(l_buf, 17, "                "); 
    if(strlen(specific_data_str) > 0 && rx_count >= 0) {
        snprintf(r_data, sizeof(r_data), "%02d/%s", rx_count, specific_data_str);
    } else {
        strncpy(r_data, specific_data_str, sizeof(r_data));
    }
    snprintf(r_buf, 17, "%s", r_data);
    Safe_Print_OLED(48, "%-16s%s", l_buf, r_buf); 
    OLED_Update(); 
}

// =======================================================
// [OLED UI 繪圖底層引擎]
// =======================================================
void Safe_Print_OLED_Smooth(int y, int min_y, int max_y, uint8_t brightness, const char *fmt, ...) {
    char temp_buf[128]; char full_line[33]; va_list argptr;
    va_start(argptr, fmt); vsnprintf(temp_buf, sizeof(temp_buf), fmt, argptr); va_end(argptr);
    int temp_len = strlen(temp_buf);
    for(int i = 0; i < 32; i++) { if(i < temp_len) { char c = temp_buf[i]; if(c < 0x20 || c > 0x7E) c = ' '; full_line[i] = c; } else { full_line[i] = ' '; } }
    full_line[32] = '\0'; OLED_PrintString(0, y, min_y, max_y, full_line, brightness);
}
void Safe_Print_OLED(int y, const char *fmt, ...) {
    char temp_buf[128]; char full_line[33]; va_list argptr;
    va_start(argptr, fmt); vsnprintf(temp_buf, sizeof(temp_buf), fmt, argptr); va_end(argptr);
    int temp_len = strlen(temp_buf);
    for(int i = 0; i < 32; i++) { if(i < temp_len) { char c = temp_buf[i]; if(c < 0x20 || c > 0x7E) c = ' '; full_line[i] = c; } else { full_line[i] = ' '; } }
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
// [全域攔截系統與鬧鐘引擎]
// =======================================================
void Handle_Alarm_Trigger(void) {
    if (!g_alarm_triggered) return;
    uint32_t start_ms = g_u32SystemMs;
    uint32_t beep_timer = g_u32SystemMs - 2000; 
    
    OLED_Clear();
    Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "================================");
    Safe_Print_OLED_Smooth(24, 0, 63, 0x0F, "      TIME'S UP! (ALARM)");
    Safe_Print_OLED_Smooth(48, 0, 63, 0x0F, "================================");
    OLED_Update();

    while(g_alarm_triggered) {
        Process_UART1_JIG_8CP_Parser(); // 維持 UART 背景接收能力
        USBHID_Process_Queue(); // 同時維持背景發送
        uint32_t elapsed = g_u32SystemMs - start_ms;
        if (elapsed > 60000) { g_alarm_triggered = 0; break; } // 1分鐘超時自動停止

        // 每兩秒發出4短聲
        if (g_u32SystemMs - beep_timer > 2000) {
            for(int i=0; i<4; i++) { JigForceBeep(60); Delay_ms(60); }
            beep_timer = g_u32SystemMs;
        }

        // 按任意鍵停止
        if ((PA->PIN & BIT8)==0 || (PF->PIN & (BIT3|BIT4|BIT5|BIT6)) != (BIT3|BIT4|BIT5|BIT6)) {
            JigBeep(50);
            g_alarm_triggered = 0;
            while((PA->PIN & BIT8)==0 || (PF->PIN & (BIT3|BIT4|BIT5|BIT6)) != (BIT3|BIT4|BIT5|BIT6)) { Delay_ms(10); }
            break;
        }
    }
    g_force_alarm_menu = 1; // 強制導航回鬧鐘選單
}

void Global_Background_Tasks(void) {
    // 1. 全域背景處理 UART1 封包
    Process_UART1_JIG_8CP_Parser();
    
    // 2. [V5.3.2] 全域處理 USB HID 佇列發送 (核心解耦技術)
    USBHID_Process_Queue();
    
    // 3. 全域背景檢查鬧鐘
    static uint8_t last_sec = 99;
    RTC_TimeTypeDef rtc;
    RV3028_GetTime(&rtc);
    if (rtc.seconds != last_sec) {
        last_sec = rtc.seconds;
        for(int i=0; i<6; i++) {
            if(g_alarms[i].enabled && g_alarms[i].hours == rtc.hours && g_alarms[i].minutes == rtc.minutes && g_alarms[i].seconds == rtc.seconds) {
                g_alarm_triggered = 1;
                break; 
            }
        }
    }
    
    // 4. 觸發鬧鐘畫面攔截
    if (g_alarm_triggered) { Handle_Alarm_Trigger(); }
}

int Get_Weekday(int year, int month, int day) {
    if (month == 1 || month == 2) { month += 12; year--; }
    int k = year % 100; int j = year / 100;
    int h = day + 13 * (month + 1) / 5 + k + k / 4 + j / 4 + 5 * j;
    return h % 7; 
}
const char* week_str[] = {"Sat", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri"};

// =======================================================
// [V5.3.2 新架構：USB HID 非同步佇列與發送引擎]
// =======================================================
// 1. 將資料丟入緩衝區 (非阻塞，瞬間完成)
void USBHID_Enqueue_Data(const char* str) {
    while(*str) {
        uint16_t next = (g_hid_head + 1) % HID_TX_BUF_SIZE;
        if (next != g_hid_tail) {
            g_hid_tx_buf[g_hid_head] = *str++;
            g_hid_head = next;
        } else break; // 緩衝區滿了則丟棄
    }
}

// 2. 封裝字串，根據設定決定是否補上 Enter (CR)
void USBHID_Enqueue_String(const char* str) {
    USBHID_Enqueue_Data(str);
    if (g_u8UsbHidAppendCR) {
        char cr_str[2] = {0x0D, 0x00}; // 0x0D 是 Enter
        USBHID_Enqueue_Data(cr_str);
    }
}

// 3. 底層觸發鍵盤按壓
int Trigger_USB_HID_Key(uint8_t mod, uint8_t key) {
    uint8_t report[8] = {0}; report[0] = mod; report[2] = key;
    uint32_t timeout = 0;
    while(g_u8EP2Ready == 0) { Delay_us(100); timeout++; if(timeout > 500) return 0; }
    g_u8EP2Ready = 0;
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), report, 8); USBD_SET_PAYLOAD_LEN(EP2, 8);
    return 1; 
}

// 4. 解析單一字元，並發送
void Internal_Send_Char_HID(char c) {
    uint8_t is_caps_on = (g_u8UsbHidSmartCaps && (g_u8Led_Status[0] & 0x02)) ? 1 : 0; 
    uint8_t mod = 0; uint8_t key = 0; 
    
    if (c >= 'a' && c <= 'z') { key = c - 'a' + 0x04; mod = is_caps_on ? 0x02 : 0x00; }
    else if (c >= 'A' && c <= 'Z') { key = c - 'A' + 0x04; mod = is_caps_on ? 0x00 : 0x02; }
    else if (c >= '1' && c <= '9') { key = c - '1' + 0x1E; } else if (c == '0') { key = 0x27; } 
    else if (c == '-') { key = 0x2D; } else if (c == '_') { mod = 0x02; key = 0x2D; } 
    else if (c == ' ') { key = 0x2C; }
    else if (c == 0x0D) { key = 0x28; } // Enter Key
    
    if (key != 0) { 
        if (!Trigger_USB_HID_Key(mod, key)) return; 
        Delay_ms(2); 
        if (!Trigger_USB_HID_Key(0, 0)) return; 
        Delay_ms(2); 
    }
}

// 5. 在 Global_Background_Tasks 中被呼叫的背景發送處理器
void USBHID_Process_Queue(void) {
    if (g_hid_head == g_hid_tail) return; // 佇列是空的
    
    // 如果 USB 通道空閒，就取出一個字元發送
    if (g_u8EP2Ready) {
        char c = g_hid_tx_buf[g_hid_tail];
        g_hid_tail = (g_hid_tail + 1) % HID_TX_BUF_SIZE;
        Internal_Send_Char_HID(c);
        
        // 當發送完最後一個字元時，發出提示音
        if (g_hid_head == g_hid_tail) {
            JigBeep(100);
        }
    }
}

// =======================================================
// [UART1 與指令解析引擎]
// =======================================================
int UART1_Read_Byte(uint8_t *data) {
    if (g_u1_rx_head == g_u1_rx_tail) return 0;
    *data = g_u1_rx_buf[g_u1_rx_tail]; g_u1_rx_tail = (g_u1_rx_tail + 1) % UART1_RX_BUF_SIZE; return 1;
}

void UART1_Send_String(const char* str) {
    while(*str) { UART_WRITE(UART1, *str++); while(UART1->FIFOSTS & UART_FIFOSTS_TXFULL_Msk); }
}

void Get_JIG_8CP_Checksum(const char* payload, char* checksum_out) {
    uint8_t sum = 0; while (*payload) { sum += (uint8_t)(*payload); payload++; } snprintf(checksum_out, 3, "%02X", sum); 
}

void JIG_8CP_Send_Packet(const char* cmd_code, const char* data) {
    char payload[64]; char checksum[3];
    snprintf(payload, sizeof(payload), "%s%s", cmd_code, data); Get_JIG_8CP_Checksum(payload, checksum);
    UART_WRITE(UART1, JIG_8CP_STX); UART1_Send_String(payload); UART1_Send_String(checksum); UART_WRITE(UART1, JIG_8CP_CR);
}

void JIG_8CP_Command_Handler(const char* cmd_code, const char* data) {
    if (strcmp(cmd_code, "SC") == 0) {
        // [V5.3.2 修改] 收到指令後不再原地發送，而是瞬間丟進背景佇列
        USBHID_Enqueue_String(data); 
    }
}

void Process_UART1_JIG_8CP_Parser(void) {
    static uint8_t rx_packet[128]; static uint8_t rx_idx = 0; static uint8_t is_stx_received = 0; uint8_t c;

    while(UART1_Read_Byte(&c)) {
        if (c == JIG_8CP_STX) { is_stx_received = 1; rx_idx = 0; } 
        else if (c == JIG_8CP_CR) {
            if (is_stx_received && rx_idx >= 4) {
                rx_packet[rx_idx] = '\0'; 
                char received_chk[3]; received_chk[0] = rx_packet[rx_idx - 2]; received_chk[1] = rx_packet[rx_idx - 1]; received_chk[2] = '\0';
                rx_packet[rx_idx - 2] = '\0'; 
                char calculated_chk[3]; Get_JIG_8CP_Checksum((char*)rx_packet, calculated_chk); 
                
                if (strcmp(received_chk, calculated_chk) == 0) {
                    char cmd_code[3]; cmd_code[0] = rx_packet[0]; cmd_code[1] = rx_packet[1]; cmd_code[2] = '\0';
                    JIG_8CP_Command_Handler(cmd_code, (char*)&rx_packet[2]); 
                }
            }
            is_stx_received = 0; 
        } else if (is_stx_received) {
            if (rx_idx < sizeof(rx_packet) - 1) rx_packet[rx_idx++] = c; else is_stx_received = 0; 
        }
    }
}

// =======================================================
// [PinConfig 與底層硬體初始化]
// =======================================================
void WIFIBLE_ReaderTest_init(void) {
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk);  SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA5MFP_I2C0_SCL | SYS_GPA_MFPL_PA4MFP_I2C0_SDA);
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA7MFP_Msk | SYS_GPA_MFPL_PA6MFP_Msk);  SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA7MFP_I2C1_SCL | SYS_GPA_MFPL_PA6MFP_I2C1_SDA);
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF1MFP_Msk | SYS_GPF_MFPL_PF0MFP_Msk);  SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF1MFP_ICE_CLK | SYS_GPF_MFPL_PF0MFP_ICE_DAT);
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA11MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA8MFP_Msk); SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA11MFP_GPIO | SYS_GPA_MFPH_PA10MFP_GPIO | SYS_GPA_MFPH_PA9MFP_GPIO | SYS_GPA_MFPH_PA8MFP_GPIO);
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA1MFP_Msk); SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA1MFP_GPIO);
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk | SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB8MFP_Msk); SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB15MFP_GPIO | SYS_GPB_MFPH_PB14MFP_GPIO | SYS_GPB_MFPH_PB8MFP_GPIO);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB7MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk); SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB7MFP_GPIO | SYS_GPB_MFPL_PB6MFP_GPIO | SYS_GPB_MFPL_PB5MFP_GPIO | SYS_GPB_MFPL_PB4MFP_GPIO);
    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC14MFP_Msk); SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC14MFP_GPIO);
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC7MFP_Msk | SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk); SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC7MFP_GPIO | SYS_GPC_MFPL_PC6MFP_GPIO | SYS_GPC_MFPL_PC1MFP_GPIO | SYS_GPC_MFPL_PC0MFP_GPIO);
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD15MFP_Msk); SYS->GPD_MFPH |= (SYS_GPD_MFPH_PD15MFP_GPIO);
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD3MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD0MFP_Msk); SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD3MFP_GPIO | SYS_GPD_MFPL_PD2MFP_GPIO | SYS_GPD_MFPL_PD1MFP_GPIO | SYS_GPD_MFPL_PD0MFP_GPIO);
    SYS->GPF_MFPH &= ~(SYS_GPF_MFPH_PF15MFP_Msk | SYS_GPF_MFPH_PF14MFP_Msk);  SYS->GPF_MFPH |= (SYS_GPF_MFPH_PF15MFP_GPIO | SYS_GPF_MFPH_PF14MFP_GPIO);
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF6MFP_Msk | SYS_GPF_MFPL_PF5MFP_Msk | SYS_GPF_MFPL_PF4MFP_Msk | SYS_GPF_MFPL_PF3MFP_Msk | SYS_GPF_MFPL_PF2MFP_Msk); SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF6MFP_GPIO | SYS_GPF_MFPL_PF5MFP_GPIO | SYS_GPF_MFPL_PF4MFP_GPIO | SYS_GPF_MFPL_PF3MFP_GPIO | SYS_GPF_MFPL_PF2MFP_GPIO);
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA0MFP_Msk); SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA3MFP_SPI0_SS | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA0MFP_SPI0_MOSI);
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB12MFP_Msk); SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB13MFP_UART0_TXD | SYS_GPB_MFPH_PB12MFP_UART0_RXD);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk); SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB3MFP_UART1_TXD | SYS_GPB_MFPL_PB2MFP_UART1_RXD);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB0MFP_Msk); SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB1MFP_UART2_TXD | SYS_GPB_MFPL_PB0MFP_UART2_RXD);
}

void Setup_GPIO_Modes(void) {
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT); PB15 = 0; 
    GPIO_SetMode(PC, BIT1, GPIO_MODE_OUTPUT); GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT); 
    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT); GPIO_SetMode(PD, BIT15, GPIO_MODE_OUTPUT); 
    GPIO_SetMode(PF, BIT15, GPIO_MODE_OUTPUT); 
    GPIO_SetMode(PB, BIT0, GPIO_MODE_QUASI); GPIO_SetMode(PA, BIT1, GPIO_MODE_QUASI); 
    GPIO_SetMode(PD, BIT2, GPIO_MODE_QUASI); GPIO_SetMode(PD, BIT1, GPIO_MODE_QUASI); GPIO_SetMode(PD, BIT0, GPIO_MODE_QUASI); 
    GPIO_SetMode(PA, BIT8, GPIO_MODE_QUASI); GPIO_SetMode(PF, BIT6, GPIO_MODE_QUASI); GPIO_SetMode(PF, BIT14, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, BIT5, GPIO_MODE_QUASI); GPIO_SetMode(PF, BIT3, GPIO_MODE_QUASI); GPIO_SetMode(PF, BIT4, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, BIT7, GPIO_MODE_OUTPUT); PC->DOUT &= ~BIT7; 
    PD->DOUT |= BIT15; PF->DOUT |= BIT15; PC->DOUT |= BIT1;  PD->DOUT |= BIT3;  PA->DOUT |= BIT8;
    PF->DOUT |= (BIT6 | BIT14 | BIT5 | BIT3 | BIT4);
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
    CLK_EnableModuleClock(USBD_MODULE);
    CLK->AHBCLK |= ((1ul << 0)|(1ul << 1)|(1ul << 2)|(1ul << 3)|(1ul << 5)); 
    CLK_EnableModuleClock(SPI0_MODULE); CLK_EnableModuleClock(I2C0_MODULE); CLK_EnableModuleClock(I2C1_MODULE); 
    CLK_EnableModuleClock(UART0_MODULE); CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_EnableModuleClock(UART1_MODULE); CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    CLK_EnableModuleClock(UART2_MODULE); CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
    CLK_EnableModuleClock(TMR0_MODULE); CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    
    CLK_EnableModuleClock(TMR1_MODULE); CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
    SystemCoreClockUpdate(); WIFIBLE_ReaderTest_init();
    
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);
    TIMER_Start(TIMER1);

    UART_Open(UART0, 115200); UART0->FIFO = (UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE; UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
    UART_Open(UART1, 115200); UART1->FIFO = (UART1->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;
    UART_EnableInt(UART1, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk); NVIC_EnableIRQ(UART13_IRQn);
    UART_Open(UART2, 9600); UART2->FIFO = (UART2->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE; UART_EnableInt(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    I2C_Open(I2C0, 100000); NVIC_EnableIRQ(UART02_IRQn); 
    SYS_LockReg();
}

// =======================================================
// [中斷與延遲模組]
// =======================================================
void TMR1_IRQHandler(void) {
    if(TIMER_GetIntFlag(TIMER1)) {
        TIMER_ClearIntFlag(TIMER1);
        g_u32SystemMs++;
        if(g_u8StopwatchRunning) g_u32StopwatchMs++;
    }
}
void UART13_IRQHandler(void) {
    uint32_t u32IntSts = UART1->INTSTS; uint32_t u32FIFOSts = UART1->FIFOSTS;
    if(u32FIFOSts & (UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_RXOVIF_Msk)) { UART1->FIFOSTS = (UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_RXOVIF_Msk); }
    if(u32IntSts & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
        while((UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) {
            uint8_t c = UART_READ(UART1); uint16_t next_head = (g_u1_rx_head + 1) % UART1_RX_BUF_SIZE;
            if (next_head != g_u1_rx_tail) { g_u1_rx_buf[g_u1_rx_head] = c; g_u1_rx_head = next_head; }
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
    if(u32u2FIFOSts & (UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_RXOVIF_Msk)) { UART2->FIFOSTS = (UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_RXOVIF_Msk); }
    if(u32u2IntSts & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
        while((UART2->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) {
            uint8_t c = UART_READ(UART2); if (c == 0x00 || c == 0xFF) continue;
            uint16_t next_head = (g_u2_rx_head + 1) % UART2_RX_BUF_SIZE;
            if (next_head != g_u2_rx_tail) { g_u2_rx_buf[g_u2_rx_head] = c; g_u2_rx_head = next_head; }
        }
    }
}
void Delay_us(uint32_t us) { CLK_SysTickDelay(us); }
void Delay_ms(uint32_t ms) { while (ms >= 100) { TIMER_Delay(TIMER0, 100 * 1000); ms -= 100; } if (ms > 0) TIMER_Delay(TIMER0, ms * 1000); }
void OLED_Force_Reset(void) { PD->DOUT |= BIT15; Delay_ms(50); PD->DOUT &= ~BIT15; Delay_ms(200); PD->DOUT |= BIT15; Delay_ms(200); }
void JigForceBeep(uint32_t ms) { uint32_t half_period_us = 185; uint32_t total_cycles = (ms * 1000) / (half_period_us * 2); for (uint32_t i = 0; i < total_cycles; i++) { PB15 = 1; Delay_us(half_period_us); PB15 = 0; Delay_us(half_period_us); } PB15 = 0; Delay_ms(50); }
void JigBeep(uint32_t ms) { if (g_u8BuzzerEnabled == 1) { JigForceBeep(ms); } }

// =======================================================
// [V5.3.0 Time Set - 碼表、鬧鐘、時間設定選單]
// =======================================================
void RTC_Time_Date_Loop(void) {
    RTC_TimeTypeDef edit_time;
    int mode = 0; uint32_t blink_timer = 0; int show_cursor = 1;
    uint8_t last_red=1, last_blue=1, last_green=1, last_yellow=1;

    while(1) {
        Global_Background_Tasks(); if (g_force_alarm_menu) break;
        blink_timer++; if(blink_timer > 10) { show_cursor = !show_cursor; blink_timer = 0; }
        OLED_Clear();

        if (mode == 0) {
            RTC_TimeTypeDef current_time;
            RV3028_GetTime(&current_time);
            int wd = Get_Weekday(current_time.year, current_time.month, current_time.date);
            Safe_Print_OLED(0,  "      --- TIME & DATE ---");
            Safe_Print_OLED(16, "       %04d/%02d/%02d (%s)", current_time.year, current_time.month, current_time.date, week_str[wd]);
            Safe_Print_OLED(32, "         %02d:%02d:%02d", current_time.hours, current_time.minutes, current_time.seconds);
            Safe_Print_OLED(48, " R:Set Time   Y:Back"); 
        } else {
            char y_s[8], m_s[8], d_s[8], hr_s[8], min_s[8], sec_s[8];
            if (mode==1 && !show_cursor) strcpy(y_s,"    "); else snprintf(y_s,8,"%04d",edit_time.year);
            if (mode==2 && !show_cursor) strcpy(m_s,"  "); else snprintf(m_s,8,"%02d",edit_time.month);
            if (mode==3 && !show_cursor) strcpy(d_s,"  "); else snprintf(d_s,8,"%02d",edit_time.date);
            if (mode==4 && !show_cursor) strcpy(hr_s,"  "); else snprintf(hr_s,8,"%02d",edit_time.hours);
            if (mode==5 && !show_cursor) strcpy(min_s,"  "); else snprintf(min_s,8,"%02d",edit_time.minutes);
            if (mode==6 && !show_cursor) strcpy(sec_s,"  "); else snprintf(sec_s,8,"%02d",edit_time.seconds);
            int wd = Get_Weekday(edit_time.year, edit_time.month, edit_time.date);
            Safe_Print_OLED(0,  "     *** SET RTC TIME ***");
            Safe_Print_OLED(16, "       %s/%s/%s (%s)", y_s, m_s, d_s, week_str[wd]);
            Safe_Print_OLED(32, "         %s:%s:%s", hr_s, min_s, sec_s);
            Safe_Print_OLED(48, " R:Next G:+ B:- Y:Cancel");
        }
        OLED_Update();
        
        uint8_t btn_exit=(PF->PIN & BIT5)?1:0, btn_next=(PA->PIN & BIT8)?1:0, btn_plus=(PF->PIN & BIT4)?1:0, btn_minus=(PF->PIN & BIT3)?1:0;
        if (btn_exit == 0 && last_yellow == 1) { JigBeep(100); if (mode == 0) break; else mode = 0; }
        if (btn_next == 0 && last_red == 1) {
            JigBeep(50);
            if (mode == 0) { mode = 1; RV3028_GetTime(&edit_time); if (edit_time.year < 2026) edit_time.year = 2026; }
            else { mode++; if (mode > 6) { RV3028_SetTime(&edit_time); mode = 0; JigBeep(200); } }
            show_cursor = 1; blink_timer = 0;
        }
        if (mode > 0 && btn_plus == 0 && last_green == 1) {
            JigBeep(30); show_cursor = 1; blink_timer = 0;
            if (mode==1) { edit_time.year++; if (edit_time.year>2099) edit_time.year=2026; }
            else if (mode==2) { edit_time.month++; if (edit_time.month>12) edit_time.month=1; }
            else if (mode==3) { edit_time.date++; if (edit_time.date>31) edit_time.date=1; }
            else if (mode==4) { edit_time.hours++; if (edit_time.hours>23) edit_time.hours=0; }
            else if (mode==5) { edit_time.minutes++; if (edit_time.minutes>59) edit_time.minutes=0; }
            else if (mode==6) { edit_time.seconds++; if (edit_time.seconds>59) edit_time.seconds=0; }
        }
        if (mode > 0 && btn_minus == 0 && last_blue == 1) {
            JigBeep(30); show_cursor = 1; blink_timer = 0;
            if (mode==1) { edit_time.year--; if (edit_time.year<2026) edit_time.year=2099; }
            else if (mode==2) { edit_time.month--; if (edit_time.month<1||edit_time.month>12) edit_time.month=12; }
            else if (mode==3) { edit_time.date--; if (edit_time.date<1||edit_time.date>31) edit_time.date=31; }
            else if (mode==4) { if (edit_time.hours==0) edit_time.hours=23; else edit_time.hours--; }
            else if (mode==5) { if (edit_time.minutes==0) edit_time.minutes=59; else edit_time.minutes--; }
            else if (mode==6) { if (edit_time.seconds==0) edit_time.seconds=59; else edit_time.seconds--; }
        }
        last_yellow = btn_exit; last_red = btn_next; last_green = btn_plus; last_blue = btn_minus;
        Delay_ms(15);
    }
}

void Stopwatch_Loop(void) {
    uint8_t last_red=1, last_blue=1, last_yellow=1;
    while(1) {
        Global_Background_Tasks(); if (g_force_alarm_menu) break;
        uint32_t ms = g_u32StopwatchMs % 1000;
        uint32_t s = (g_u32StopwatchMs / 1000) % 60;
        uint32_t m = (g_u32StopwatchMs / 60000) % 60;
        uint32_t h = (g_u32StopwatchMs / 3600000) % 100;
        OLED_Clear();
        Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "      --- STOPWATCH ---");
        Safe_Print_OLED_Smooth(24, 0, 63, 0x0F, "       %02d:%02d:%02d:%03d", h, m, s, ms);
        Safe_Print_OLED_Smooth(48, 0, 63, 0x08, " R:Start/Stop  B:Rst  Y:Back");
        OLED_Update();

        uint8_t red=(PA->PIN & BIT8)?1:0, blue=(PF->PIN & BIT3)?1:0, yellow=(PF->PIN & BIT5)?1:0;
        if (red == 0 && last_red == 1) { JigBeep(50); g_u8StopwatchRunning = !g_u8StopwatchRunning; }
        if (blue == 0 && last_blue == 1) { JigBeep(50); g_u32StopwatchMs = 0; }
        if (yellow == 0 && last_yellow == 1) { JigBeep(100); break; }
        last_red = red; last_blue = blue; last_yellow = yellow;
        Delay_ms(15); 
    }
}

void Alarm_Menu_Loop(void) {
    int current_idx = 0; int mode = 0; 
    uint8_t last_red=1, last_blue=1, last_green=1, last_yellow=1, last_black=1;
    uint32_t blink_timer = 0; int show_cursor = 1;

    while(1) {
        Global_Background_Tasks(); 
        if (g_force_alarm_menu && !g_alarm_triggered) g_force_alarm_menu = 0; 
        blink_timer++; if (blink_timer > 10) { show_cursor = !show_cursor; blink_timer = 0; }
        OLED_Clear();
        
        int p_idx = (current_idx - 1 + 6) % 6; int n_idx = (current_idx + 1) % 6;
        char buf[32];
        snprintf(buf, 32, "  %d. [%02d:%02d:%02d] (%s)", p_idx+1, g_alarms[p_idx].hours, g_alarms[p_idx].minutes, g_alarms[p_idx].seconds, g_alarms[p_idx].enabled?"ON ":"OFF");
        Safe_Print_OLED_Smooth(0, 0, 63, 0x04, buf);

        if (mode == 0) {
            snprintf(buf, 32, "> %d. [%02d:%02d:%02d] (%s)", current_idx+1, g_alarms[current_idx].hours, g_alarms[current_idx].minutes, g_alarms[current_idx].seconds, g_alarms[current_idx].enabled?"ON ":"OFF");
        } else {
            char h_s[4], m_s[4], s_s[4];
            if(mode==1 && !show_cursor) strcpy(h_s,"  "); else snprintf(h_s,4,"%02d",g_alarms[current_idx].hours);
            if(mode==2 && !show_cursor) strcpy(m_s,"  "); else snprintf(m_s,4,"%02d",g_alarms[current_idx].minutes);
            if(mode==3 && !show_cursor) strcpy(s_s,"  "); else snprintf(s_s,4,"%02d",g_alarms[current_idx].seconds);
            snprintf(buf, 32, "> %d. [%s:%s:%s] (%s)", current_idx+1, h_s, m_s, s_s, g_alarms[current_idx].enabled?"ON ":"OFF");
        }
        Safe_Print_OLED_Smooth(16, 0, 63, 0x0F, buf);
        snprintf(buf, 32, "  %d. [%02d:%02d:%02d] (%s)", n_idx+1, g_alarms[n_idx].hours, g_alarms[n_idx].minutes, g_alarms[n_idx].seconds, g_alarms[n_idx].enabled?"ON ":"OFF");
        Safe_Print_OLED_Smooth(32, 0, 63, 0x04, buf);
        Safe_Print_OLED_Smooth(48, 0, 63, 0x08, " Blk:ON/OFF R:Set Y:Back");
        OLED_Update();

        uint8_t red=(PA->PIN&BIT8)?1:0, blue=(PF->PIN&BIT3)?1:0, green=(PF->PIN&BIT4)?1:0, yellow=(PF->PIN&BIT5)?1:0, black=(PF->PIN&BIT6)?1:0;
        if (mode == 0) {
            if (red==0 && last_red==1) { JigBeep(50); mode=1; show_cursor=1; blink_timer=0; }
            if (blue==0 && last_blue==1) { JigBeep(50); current_idx = (current_idx + 1) % 6; }
            if (green==0 && last_green==1) { JigBeep(50); current_idx = (current_idx - 1 + 6) % 6; }
            if (black==0 && last_black==1) { JigBeep(50); g_alarms[current_idx].enabled = !g_alarms[current_idx].enabled; }
            if (yellow==0 && last_yellow==1) { JigBeep(100); break; }
        } else {
            if (red==0 && last_red==1) { JigBeep(50); mode++; if (mode>3) mode=0; show_cursor=1; blink_timer=0; }
            if (green==0 && last_green==1) {
                JigBeep(30); show_cursor=1; blink_timer=0;
                if (mode==1) g_alarms[current_idx].hours = (g_alarms[current_idx].hours + 1) % 24;
                if (mode==2) g_alarms[current_idx].minutes = (g_alarms[current_idx].minutes + 1) % 60;
                if (mode==3) g_alarms[current_idx].seconds = (g_alarms[current_idx].seconds + 1) % 60;
            }
            if (blue==0 && last_blue==1) {
                JigBeep(30); show_cursor=1; blink_timer=0;
                if (mode==1) g_alarms[current_idx].hours = (g_alarms[current_idx].hours==0) ? 23 : g_alarms[current_idx].hours - 1;
                if (mode==2) g_alarms[current_idx].minutes = (g_alarms[current_idx].minutes==0) ? 59 : g_alarms[current_idx].minutes - 1;
                if (mode==3) g_alarms[current_idx].seconds = (g_alarms[current_idx].seconds==0) ? 59 : g_alarms[current_idx].seconds - 1;
            }
            if (yellow==0 && last_yellow==1) { JigBeep(100); mode=0; }
        }
        last_red=red; last_blue=blue; last_green=green; last_yellow=yellow; last_black=black;
        Delay_ms(15);
    }
}

void Time_Set_Menu_Loop(void) {
    const char *items[] = { "1. Time & Date", "2. Stopwatch", "3. Alarms" };
    int idx = 0; uint8_t last_red=1, last_blue=1, last_green=1, last_yellow=1;

    while(1) {
        Global_Background_Tasks();
        if (g_force_alarm_menu) { idx = 2; Alarm_Menu_Loop(); continue; } // 自動導航到鬧鐘
        OLED_Clear();
        Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "      --- TIME SET ---");
        Safe_Print_OLED_Smooth(16, 16, 63, (idx==0)?0x0F:0x04, "%s 1. Time & Date", (idx==0)?">":" ");
        Safe_Print_OLED_Smooth(32, 16, 63, (idx==1)?0x0F:0x04, "%s 2. Stopwatch", (idx==1)?">":" ");
        Safe_Print_OLED_Smooth(48, 16, 63, (idx==2)?0x0F:0x04, "%s 3. Alarms", (idx==2)?">":" ");
        OLED_Update();

        uint8_t red=(PA->PIN&BIT8)?1:0, blue=(PF->PIN&BIT3)?1:0, green=(PF->PIN&BIT4)?1:0, yellow=(PF->PIN&BIT5)?1:0;
        if (blue == 0 && last_blue == 1) { JigBeep(50); idx = (idx+1)%3; }
        if (green == 0 && last_green == 1) { JigBeep(50); idx = (idx-1+3)%3; }
        if (red == 0 && last_red == 1) {
            JigBeep(50);
            if (idx == 0) RTC_Time_Date_Loop();
            else if (idx == 1) Stopwatch_Loop();
            else if (idx == 2) Alarm_Menu_Loop();
        }
        if (yellow == 0 && last_yellow == 1) { JigBeep(100); break; }
        last_red=red; last_blue=blue; last_green=green; last_yellow=yellow;
        Delay_ms(15);
    }
}

// =======================================================
// [各式監控 UI 介面]
// =======================================================
void UART_Monitor_Test(uint32_t u32BaudRate) {
    UART_DisableInt(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk); Interface_init(); PB4 = 1;               
    UART_Open(UART2, u32BaudRate); UART2->FIFO = (UART2->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;
    UART_EnableInt(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    char title_buf[32]; snprintf(title_buf, sizeof(title_buf), "UART2 Mntr %u", u32BaudRate);
    Show_Test_Start_Screen(title_buf); 
    int rx_count = 0; char rx_buf[128] = {0}; uint32_t loop_tick = 1000; int power_state = 0;

    while(1) {
        Global_Background_Tasks(); if (g_force_alarm_menu) break; 
        if (Check_Power_Toggle(&power_state)) {
            if (power_state) { PC->DOUT |= BIT7; JigBeep(100); UART2->FIFO |= UART_FIFO_RXRST_Msk; UART2->FIFOSTS = (UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk); __disable_irq(); g_u2_rx_head = 0; g_u2_rx_tail = 0; __enable_irq(); } else { PC->DOUT &= ~BIT7; JigBeep(500); }
            Reset_Current_Filter(); loop_tick = 1000;
        }
        Process_Background_Sampling(power_state, loop_tick); if (Check_Reset_Button()) loop_tick = 1000;
        if (power_state && (UART2->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) {
            int rx_len = 0; int rx_to = 0; memset(rx_buf, 0, sizeof(rx_buf));
            while(rx_len < 127) { if((UART2->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) { char c = UART_READ(UART2); if(c >= 0x20 && c <= 0x7E) rx_buf[rx_len++] = c; else rx_buf[rx_len++] = '.'; rx_to = 0; } else { Delay_ms(1); rx_to++; if(rx_to > 100) break; } }
            if (rx_len >= 4) { rx_buf[rx_len] = '\0'; rx_count++; JigBeep(50); loop_tick = 1000; }
        }
        if (Check_Exit_Button()) break;
        if (loop_tick >= 1000) { Update_Dashboard_Display(power_state, rx_count, rx_buf); loop_tick = 0; }
        Delay_ms(1); loop_tick++;
    }
    PC->DOUT &= ~BIT7; OLED_Clear(); Safe_Print_OLED(0, "Monitor End"); OLED_Update(); Delay_ms(1000);
}

void Wiegand_Monitor_Test(void) {
    Interface_init(); PB6 = 1; PA11 = 1; GPIO_SetMode(PA, BIT10, GPIO_MODE_QUASI); GPIO_SetMode(PB, BIT5, GPIO_MODE_QUASI); GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5);
    Show_Test_Start_Screen("Wiegand Monitor"); 
    int rx_count = 0; uint32_t loop_tick = 1000; int power_state = 0; uint64_t last_wg_data = 0; char data_str[32] = {0};

    while(1) {
        Global_Background_Tasks(); if (g_force_alarm_menu) break;
        if (Check_Power_Toggle(&power_state)) {
            if (power_state) { PC->DOUT |= BIT7; JigBeep(100); QUEUE_CLEAR(au64WG1); GPIO_CLR_INT_FLAG(PA, BIT10); GPIO_CLR_INT_FLAG(PB, BIT5); GPIO_EnableInt(PA, 10, GPIO_INT_FALLING); GPIO_EnableInt(PB, 5, GPIO_INT_FALLING);  NVIC_EnableIRQ(GPIO_PAPBPGPH_IRQn); } else { GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5); PC->DOUT &= ~BIT7; JigBeep(500); }
            Reset_Current_Filter(); loop_tick = 1000;
        }
        Process_Background_Sampling(power_state, loop_tick); if (Check_Reset_Button()) loop_tick = 1000; vCheckingTimeOut();
        if (!QUEUE_EMPTY(au64WG1)) { last_wg_data = QUEUE_PULL(au64WG1); if (g_u8WiegandNum > 0) { rx_count++; JigBeep(50); loop_tick = 1000; } }
        if (Check_Exit_Button()) break;
        if (loop_tick >= 1000) {
            if (rx_count > 0) snprintf(data_str, sizeof(data_str), "W%02d:%llX", g_u8WiegandNum, last_wg_data); else strcpy(data_str, "WAITING...");
            Update_Dashboard_Display(power_state, rx_count, data_str); loop_tick = 0; 
        }
        Delay_ms(1); loop_tick++;
    }
    PC->DOUT &= ~BIT7; GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5); OLED_Clear(); Safe_Print_OLED(0, "Monitor End"); OLED_Update(); Delay_ms(1000);
}

void Decode_TK2_Raw(char* out_str) {
    int start_idx = -1; int out_idx = 0;
    for(int i = 0; i <= TK2Cnt - 5; i++) { if(g_u8TK2Bit[i] == 0x31 && g_u8TK2Bit[i+1] == 0x31 && g_u8TK2Bit[i+2] == 0x30 && g_u8TK2Bit[i+3] == 0x31 && g_u8TK2Bit[i+4] == 0x30) { start_idx = i; break; } }
    if (start_idx != -1) { for(int i = start_idx; i <= TK2Cnt - 5; i += 5) { int b0 = g_u8TK2Bit[i]-0x30; int b1 = g_u8TK2Bit[i+1]-0x30; int b2 = g_u8TK2Bit[i+2]-0x30; int b3 = g_u8TK2Bit[i+3]-0x30; uint8_t val = b0*1 + b1*2 + b2*4 + b3*8; char c = val + 0x30; out_str[out_idx++] = c; if (c == '?') break; if (out_idx >= 30) break; } } else { strcpy(out_str, "NO_SS_ERR"); } out_str[out_idx] = '\0';
}

void TK2_Monitor_Test(void) {
    Interface_init(); PB7 = 1; PA11 = 1; GPIO_SetMode(PA, BIT10, GPIO_MODE_QUASI); GPIO_SetMode(PB, BIT5, GPIO_MODE_QUASI); GPIO_SetMode(PB, BIT8, GPIO_MODE_QUASI); GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5);  GPIO_DisableInt(PB, 8);
    Show_Test_Start_Screen("TK2 Monitor");
    int rx_count = 0; uint32_t loop_tick = 1000; int power_state = 0; uint8_t last_tk2_cnt = 0; uint32_t tk2_idle_tick = 0; char tk2_str[64] = "WAITING...";

    while(1) {
        Global_Background_Tasks(); if (g_force_alarm_menu) break;
        if (Check_Power_Toggle(&power_state)) {
            if (power_state) { PC->DOUT |= BIT7; JigBeep(100); TK2Cnt = 0; last_tk2_cnt = 0; tk2_idle_tick = 0; g_u8TK2Step = 0; memset((void *)g_u8TK2Bit, 0, sizeof(g_u8TK2Bit)); GPIO_CLR_INT_FLAG(PB, BIT5); GPIO_EnableInt(PB, 5, GPIO_INT_FALLING);  NVIC_EnableIRQ(GPIO_PAPBPGPH_IRQn); } else { GPIO_DisableInt(PB, 5); PC->DOUT &= ~BIT7; JigBeep(500); }
            Reset_Current_Filter(); loop_tick = 1000;
        }
        Process_Background_Sampling(power_state, loop_tick); if (Check_Reset_Button()) loop_tick = 1000; vCheckingTimeOut();
        if (TK2Cnt > 0) {
            if (TK2Cnt != last_tk2_cnt) { last_tk2_cnt = TK2Cnt; tk2_idle_tick = 0; } else { tk2_idle_tick++; if (tk2_idle_tick > 50) { rx_count++; Decode_TK2_Raw(tk2_str); JigBeep(50); loop_tick = 1000; TK2Cnt = 0; last_tk2_cnt = 0; g_u8TK2Step = 0; memset((void *)g_u8TK2Bit, 0, sizeof(g_u8TK2Bit)); } }
        }
        if (Check_Exit_Button()) break;
        if (loop_tick >= 1000) { Update_Dashboard_Display(power_state, rx_count, tk2_str); loop_tick = 0; }
        Delay_ms(1); loop_tick++;
    }
    PC->DOUT &= ~BIT7; GPIO_DisableInt(PB, 5); OLED_Clear(); Safe_Print_OLED(0, "Monitor End"); OLED_Update(); Delay_ms(1000);
}

void UART1_JIG_8CP_Test(void) {
    OLED_Clear(); Safe_Print_OLED(0, "UART1 JIG_8CP"); Safe_Print_OLED(16, "Red Btn -> TX"); Safe_Print_OLED(32, "Wait RX Cmd:01..."); Safe_Print_OLED(48, "Yellow(Exit)->Back"); OLED_Update();
    __disable_irq(); g_u1_rx_head = 0; g_u1_rx_tail = 0; __enable_irq();

    while(1) {
        Global_Background_Tasks(); if (g_force_alarm_menu) break; 
        if ((PA->PIN & BIT8) == 0) { 
            Delay_ms(50);
            if ((PA->PIN & BIT8) == 0) {
                JigForceBeep(50); JIG_8CP_Send_Packet("SC", "ABCD123");
                OLED_Clear(); Safe_Print_OLED(0, "--- JIG_8CP TX ---"); Safe_Print_OLED(16, "Cmd : SC (HID)"); Safe_Print_OLED(32, "Data: ABCD123"); OLED_Update();
                while ((PA->PIN & BIT8) == 0) { Delay_ms(10); } Delay_ms(1000);
            }
        }
        if(Check_Exit_Button()) break;
        Delay_ms(15);
    }
}

// =======================================================
// [Main 主程式]
// =======================================================
int main(void) {
    SYS_Init();
    Setup_GPIO_Modes();
    Delay_ms(500); 
    
    USBD_Open(&gsInfo, HID_ClassRequest, NULL);
    HID_Init(); NVIC_EnableIRQ(USBD_IRQn); USBD_Start();
    OLED_Force_Reset(); vOLED_INIT(); vINA237_Init(); set237Calibration_1A(); RV3028_Init();
    
    OLED_Clear(); Safe_Print_OLED(0, "System Ready"); Safe_Print_OLED(16, "JIG-8FT-P1 OK"); OLED_Update();
    JigBeep(500); Delay_ms(100); JigBeep(500); Delay_ms(1000);
    
    const char *menu_items[] = { "RS232 Monitor", "Wiegand", "TK2", "Time Set", "Buzzer Settings", "USBHID SET", "UART1 JIG_8CP" };
    const int NUM_ITEMS = sizeof(menu_items) / sizeof(menu_items[0]); int current_idx = 1; 

    const char *baud_items[] = { "115200, N, 8, 1", "9600, N, 8, 1", "19200, N, 8, 1", "38400, N, 8, 1" };
    const uint32_t baud_values[] = { 115200, 9600, 19200, 38400 };
    const int NUM_BAUDS = sizeof(baud_items) / sizeof(baud_items[0]);

    while(1) {
        Global_Background_Tasks(); 
        if (g_force_alarm_menu) { current_idx = 3; Time_Set_Menu_Loop(); continue; }

        UI_Draw_Menu_State("Select Function", menu_items, NUM_ITEMS, current_idx);
        int selected = 0;

        while(1) {
            Global_Background_Tasks(); 
            if (g_force_alarm_menu) { selected = 2; break; }

            if((PF->PIN & BIT3) == 0) { Delay_ms(50); if((PF->PIN & BIT3) == 0) { JigBeep(50); UI_Menu_Scroll_Anim_Smooth("Select Function", menu_items, NUM_ITEMS, current_idx, 1); current_idx = (current_idx + 1) % NUM_ITEMS; while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break; } }
            if((PF->PIN & BIT4) == 0) { Delay_ms(50); if((PF->PIN & BIT4) == 0) { JigBeep(50); UI_Menu_Scroll_Anim_Smooth("Select Function", menu_items, NUM_ITEMS, current_idx, -1); current_idx = (current_idx - 1 + NUM_ITEMS) % NUM_ITEMS; while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break; } }
            if((PF->PIN & BIT5) == 0) { Delay_ms(50); if((PF->PIN & BIT5) == 0) { JigBeep(200); while((PF->PIN & BIT5) == 0) { Delay_ms(10); } selected = 1; break; } }
        }

        if (selected == 2) continue; // 重新整理主選單(因被鬧鐘中斷)

        if (selected == 1) {
            if (current_idx == 0) {
                int baud_idx = 1; int baud_selected = 0;
                while(1) {
                    UI_Draw_Menu_State("Select Baud Rate", baud_items, NUM_BAUDS, baud_idx);
                    while(1) {
                        Global_Background_Tasks(); if (g_force_alarm_menu) { baud_selected = 1; break; }
                        if((PF->PIN & BIT3) == 0) { Delay_ms(50); if((PF->PIN & BIT3) == 0) { JigBeep(50); UI_Menu_Scroll_Anim_Smooth("Select Baud Rate", baud_items, NUM_BAUDS, baud_idx, 1); baud_idx = (baud_idx + 1) % NUM_BAUDS; while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break; } }
                        if((PF->PIN & BIT4) == 0) { Delay_ms(50); if((PF->PIN & BIT4) == 0) { JigBeep(50); UI_Menu_Scroll_Anim_Smooth("Select Baud Rate", baud_items, NUM_BAUDS, baud_idx, -1); baud_idx = (baud_idx - 1 + NUM_BAUDS) % NUM_BAUDS; while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break; } }
                        if(Check_Exit_Button()) { baud_selected = 1; break; }
                    }
                    if (baud_selected == 1) break; 
                }
                if(!g_force_alarm_menu) UART_Monitor_Test(baud_values[baud_idx]);
            } 
            else if (current_idx == 1) { Wiegand_Monitor_Test(); } 
            else if (current_idx == 2) { TK2_Monitor_Test(); } 
            else if (current_idx == 3) { Time_Set_Menu_Loop(); }
            else if (current_idx == 4) { 
                int exit_buzzer = 0;
                while(1) {
                    Global_Background_Tasks(); if (g_force_alarm_menu) break;
                    OLED_Clear(); Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "Buzzer Settings");
                    if (g_u8BuzzerEnabled) Safe_Print_OLED_Smooth(16, 16, 63, 0x0F, "  Current: ON "); else Safe_Print_OLED_Smooth(16, 16, 63, 0x04, "  Current: OFF");
                    Safe_Print_OLED_Smooth(32, 16, 63, 0x08, " Blue(-) -> Turn ON"); Safe_Print_OLED_Smooth(48, 16, 63, 0x08, " Green(+) -> Turn OFF"); OLED_Update();
                    
                    while(1) {
                        Global_Background_Tasks(); if (g_force_alarm_menu) { exit_buzzer = 1; break; }
                        if((PF->PIN & BIT3) == 0) { Delay_ms(50); if((PF->PIN & BIT3) == 0) { g_u8BuzzerEnabled = 1; JigForceBeep(100); while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break; } }
                        if((PF->PIN & BIT4) == 0) { Delay_ms(50); if((PF->PIN & BIT4) == 0) { g_u8BuzzerEnabled = 0; while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break; } }
                        if(Check_Exit_Button()) { exit_buzzer = 1; break; }
                    }
                    if (exit_buzzer == 1) break;
                }
            }
            else if (current_idx == 5) {
                int exit_usb_test = 0;
                while(1) {
                    Global_Background_Tasks(); if (g_force_alarm_menu) break;
                    OLED_Clear(); Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "USBHID SET");
                    Safe_Print_OLED_Smooth(16, 16, 63, g_u8UsbHidAppendCR ? 0x0F : 0x04, " Add CR  : %s", g_u8UsbHidAppendCR ? "ON " : "OFF");
                    Safe_Print_OLED_Smooth(32, 16, 63, g_u8UsbHidSmartCaps ? 0x0F : 0x04, " SyncCaps: %s", g_u8UsbHidSmartCaps ? "ON " : "OFF");
                    Safe_Print_OLED_Smooth(48, 16, 63, 0x04, " G:CR B:Caps R:Test"); OLED_Update();
                    
                    while(1) {
                        Global_Background_Tasks(); if (g_force_alarm_menu) { exit_usb_test = 1; break; }
                        if((PF->PIN & BIT4) == 0) { Delay_ms(50); if((PF->PIN & BIT4) == 0) { g_u8UsbHidAppendCR = !g_u8UsbHidAppendCR; JigBeep(50); while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break; } }
                        if((PF->PIN & BIT3) == 0) { Delay_ms(50); if((PF->PIN & BIT3) == 0) { g_u8UsbHidSmartCaps = !g_u8UsbHidSmartCaps; JigBeep(50); while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break; } }
                        if((PA->PIN & BIT8) == 0) { Delay_ms(50); if((PA->PIN & BIT8) == 0) { JigForceBeep(50); OLED_Clear(); Safe_Print_OLED_Smooth(16, 16, 63, 0x0F, " Sending via USB..."); OLED_Update(); USBHID_Enqueue_String("BALLY-chou_test_0429"); OLED_Clear(); Safe_Print_OLED_Smooth(16, 16, 63, 0x0F, " Send Success!"); OLED_Update(); Delay_ms(1000); while((PA->PIN & BIT8) == 0) { Delay_ms(10); } break; } }
                        if(Check_Exit_Button()) { exit_usb_test = 1; break; }
                    }
                    if (exit_usb_test == 1) break;
                }
            }
            else if (current_idx == 6) { UART1_JIG_8CP_Test(); }
        }
    }
}