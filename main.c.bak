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
 * V5.3.3 (2026/06/30): [重大架構升級] USB HID 全面改為「非同步佇列 (Ring Buffer) 背景發送」。
 * V5.3.6 (2026/07/02): [架構解耦] 引入 VRAM 虛擬顯存引擎，將 vsnprintf 運算與 OLED 硬體徹底分離。
 * V5.3.8 (2026/07/07): [通訊與儲存] 修正 WIFI CMD 遺漏 及 新增 FIRMWARE_VERSION 統一管理。
 * V5.3.9 (2026/07/08): [電源狀態全域同步] 解決 WEB 發送 PWON 時，OLED 畫面與電流平均值無法同步更新問題。
 * V5.3.10 (2026/07/08): [UI Bug 修復] 修復 Alarm_Menu_Loop 中誤用硬體 OLED_Clear 導致 VRAM 溢滿與畫面黑屏問題。
 * V5.3.11 (2026/07/08): [通訊升級與安全機制] 
 * 1. 實作 GC/CF 指令交握，動態接收 ESP32 SD卡內的 Config 電壓/電流上下限。
 * 2. 實作 ST/STOK 指令，接收 NTP 時間並自動寫入 RV-3028 RTC。
 * 3. 升級 Process_Background_Sampling 具備「全域智慧電源保護」，超限時自動切斷硬體並同步回報 WEB。
 * V5.3.12 (2026/07/09): [系統穩定性定版] 全面統整實體按鈕與 WEB 虛擬按鈕的旗標交握，完美實現 0 秒差同步與全域安全斷電機制。
 * V5.3.13 (2026/07/13): [計時與同步優化] 
 * 1. 修正 Power_Monitor_Loop 中 PD 指令發送與 UI 更新計時誤差，改用硬體 g_u32SystemMs 精確控制。
 * 2. 同步 CUR 電流顯示與 g_fMaxCurrent 更新頻率，提升瞬時值捕捉精度。
 * 3. 分離鬧鐘儲存按鈕，黑色按鈕 (PF6) 負責儲存，黃色按鈕 (PF5) 僅負責退出。
 * 4. 啟用 UART0 (PB13/PB12) 作為 Debug 輸出，新增 DEBUG_PRINTF 宏。
 * V5.3.14 (2026/07/14): [除錯與穩定性優化]
 * 1. 在 Power_Monitor_Loop 中加入循環時間監控，當單次循環執行時間超過預期時輸出警告。
 * 2. 在 PD 指令發送處加入更精細的時間戳和數據內容 Debug 訊息。
 * V5.3.15 (2026/07/15): [熔斷警示完美修復]
 * 1. 修復因 M031 硬體切斷過快導致 ESP32 漏接異常數據的問題。在 Trigger_Power_Protection 中主動「補發」最後一筆異常 PD 封包，確保 WEB 警示視窗 100% 彈出。
 * V5.3.16 (2026/07/15): [除錯開關與電源狀態修正]
 * 1. 新增全域變數 g_u8DebugEnable，用於控制 DEBUG_PRINTF 的輸出。
 * 2. 修正 UART_Monitor_Test 中的電源控制邏輯，確保其正確使用全域 g_power_state。
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
// [系統版本控制]
// =======================================================
#define FIRMWARE_VERSION "V5.3.16"

// =======================================================
// [系統全域設定與變數]
// =======================================================
volatile uint8_t g_u8BuzzerEnabled = 0; 
volatile uint8_t g_u8EP2Ready = 0;
extern volatile uint8_t g_u8Led_Status[8]; 
volatile uint8_t g_u8UsbHidAppendCR = 1; 
volatile uint8_t g_u8UsbHidSmartCaps = 1; 

// --- [V5.3.16 新增] Debug 開關 ---
volatile uint8_t g_u8DebugEnable = 1; // 1 = 開啟 Debug 輸出, 0 = 關閉 Debug 輸出

// TMR1 硬體系統計時 (1ms) 與背景狀態控制
volatile uint32_t g_u32SystemMs = 0;
volatile uint32_t g_u32StopwatchMs = 0;
volatile uint8_t g_u8StopwatchRunning = 0;
volatile uint8_t g_force_alarm_menu = 0; 

// =======================================================
// [WIFI 與通訊、電源狀態變數]
// =======================================================
char g_szWifiIP[20] = "";
uint8_t g_u8WifiConnected = 0;

volatile int g_power_state = 0;          // 記錄當前物理電源真實狀態
volatile int g_web_power_toggle_req = 0; // 網頁端發來的電源控制請求 (0:無, 1:要求ON, 2:要求OFF)

// =======================================================
// [新增：電源保護機制設定 (從 SD 卡 CF 指令讀取)]
// =======================================================
float g_cfg_scale_max = 500.0f;
float g_cfg_current_max = 400.0f;
float g_cfg_voltage_min = 0.0f;
float g_cfg_voltage_max = 12.0f;
uint8_t g_cfg_overload_sec = 3;
uint8_t g_config_received = 0;

uint32_t g_overload_start_tick = 0; // 紀錄電流超載起始時間
uint8_t  g_is_overloading = 0;      // 標記是否正在超載計時中

float Process_Background_Sampling(volatile int *power_state, uint32_t current_time_ms);
// [V5.3.15 觸發電源保護副程式 - 加入最終數據補發機制]
// [V5.3.16] 接受 volatile int * 以消除編譯警告
void Trigger_Power_Protection(volatile int *power_state, const char* reason, float final_c, float final_v);

// =======================================================
// [V5.3.3] USB HID 專用非同步發送緩衝區 (Ring Buffer)
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
void JIG_8CP_Send_Packet(const char* cmd_code, const char* data);

// [V5.3.6 架構升級 API]
void UI_Clear(void);
void UI_Update(void);
void Safe_Print_OLED_Smooth(int y, int min_y, int max_y, uint8_t brightness, const char *fmt, ...);
void Safe_Print_OLED(int y, const char *fmt, ...);

void Time_Set_Menu_Loop(void);
void Global_Background_Tasks(void);
void Show_Test_Start_Screen(const char* title);
void Update_Dashboard_Display(int power_state, int rx_count, const char* specific_data_str);
void UI_Draw_Menu_State(const char* title, const char** items, int num_items, int curr_idx);
void UI_Menu_Scroll_Anim_Smooth(const char* title, const char** items, int num_items, int old_idx, int dir);
int Check_Exit_Button(void);

// [V5.3.3 新增宣告]
void USBHID_Enqueue_Data(const char* str);
void USBHID_Enqueue_String(const char* str);
void USBHID_Process_Queue(void);
void Internal_Send_Char_HID(char c);

// [V5.3.13 新增宣告]
void DEBUG_PRINTF(const char* fmt, ...);

// =======================================================
// [高頻電流滑動平均濾波器與峰值追蹤]
// =======================================================
#define CURRENT_FILTER_SIZE 50 
float g_fCurrentBuffer[CURRENT_FILTER_SIZE] = {0};
uint8_t g_u8CurrentFilterIdx = 0;
uint8_t g_u8FilterFilled = 1;
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
    // 1. 偵測實體 Red 按鈕
    if ((PA->PIN & BIT8) == 0) {
        Delay_ms(50);
        if ((PA->PIN & BIT8) == 0) {
            *power_state = !(*power_state);
            g_power_state = *power_state; // 同步全域
            while ((PA->PIN & BIT8) == 0) {}
            Delay_ms(50);
            
            // 操作員按了實體按鈕，主動推播狀態給 ESP32 以 0 秒差同步網頁
            JIG_8CP_Send_Packet("PW", *power_state ? "ON" : "OFF");
            return 1;
        }
    }
    
    // 2. 偵測網頁發送的軟體控制請求 (虛擬按鈕)
    if (g_web_power_toggle_req == 1 && *power_state == 0) {
        *power_state = 1;
        g_power_state = 1;
        g_web_power_toggle_req = 0;
        return 1;
    }
    if (g_web_power_toggle_req == 2 && *power_state == 1) {
        *power_state = 0;
        g_power_state = 0;
        g_web_power_toggle_req = 0;
        return 1; 
    }
    
    g_web_power_toggle_req = 0; // 消化過期的冗餘旗標
    return 0;
}

// [V5.3.15 觸發電源保護副程式 - 加入最終數據補發機制]
// [V5.3.16] 接受 volatile int * 以消除編譯警告
void Trigger_Power_Protection(volatile int *power_state, const char* reason, float final_c, float final_v) {
    // ?? 0. 主動補發「最後一筆異常數據」給 ESP32，這是觸發網頁熔斷警示視窗的關鍵！
    char pd_buf[32];
    snprintf(pd_buf, sizeof(pd_buf), "%.1f,%.2f", final_c, final_v);
    JIG_8CP_Send_Packet("PD", pd_buf);
    Delay_ms(50); // 給予 ESP32 處理異常 PD 封包的緩衝時間

    // 1. 強制切斷實體電源
    PC->DOUT &= ~BIT7; 
    *power_state = 0;
    g_power_state = 0; // 同步全域變數
    g_is_overloading = 0;
    
    // 2. 通知 ESP32 網頁端，強制關閉 WEB 上的開關狀態
    JIG_8CP_Send_Packet("PW", "OFF");
    
    // 3. UI 滿版警告提示 (蓋掉背景)
    UI_Clear();
    Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "================================");
    Safe_Print_OLED_Smooth(16, 0, 63, 0x0F, "   POWER PROTECT TRIGGERED!");
    Safe_Print_OLED_Smooth(32, 0, 63, 0x0F, "   REASON: %s", reason);
    Safe_Print_OLED_Smooth(48, 0, 63, 0x0F, "================================");
    UI_Update();
    
    // 4. 強烈警報聲，並給予視覺延遲
    for(int i=0; i<5; i++) { JigForceBeep(100); Delay_ms(100); }
    Delay_ms(2000); 
}


// [V5.3.11] 將 power_state 改為指標，以便在背景採樣時直接強制斷電
// [V5.3.13] 修改為使用時間戳，並返回當前採樣值
// [V5.3.15] 將異常數值傳入 Trigger_Power_Protection 確保完整保護
// [V5.3.16] 接受 volatile int * 以消除編譯警告
float Process_Background_Sampling(volatile int *power_state, uint32_t current_time_ms) {
    static uint32_t s_last_sample_ms = 0;
    if (*power_state && (current_time_ms - s_last_sample_ms >= 20)) { // 每 20ms 採樣一次
        s_last_sample_ms = current_time_ms;
        float sample_c = getCurrent_mA();
        if (sample_c == 0.0f) { set237Calibration_1A(); sample_c = getCurrent_mA(); }
        Push_Current_Sample(sample_c);

        if (sample_c > g_fMaxCurrent) g_fMaxCurrent = sample_c;
        if (sample_c < g_fMinCurrent) g_fMinCurrent = sample_c;

        // --- 電源保護機制 (需要已收到 CF 設定值才啟動) ---
        if (g_config_received) {
            float sample_v = getBusVoltage_V();
            
            // 1. 電壓異常 (瞬間保護 - 立刻觸發)
            if (sample_v > g_cfg_voltage_max) {
                Trigger_Power_Protection(power_state, "OVER VOLTAGE", sample_c, sample_v);
                return sample_c; 
            }
            if (sample_v < g_cfg_voltage_min) {
                Trigger_Power_Protection(power_state, "UNDER VOLTAGE", sample_c, sample_v);
                return sample_c;
            }
            
            // 2. 電流超載 (延遲保護 - 容忍瞬間突波)
            if (sample_c > g_cfg_current_max) {
                if (!g_is_overloading) {
                    g_is_overloading = 1;
                    g_overload_start_tick = current_time_ms; // 開始記錄超載時間
                } else {
                    if ((current_time_ms - g_overload_start_tick) >= (g_cfg_overload_sec * 1000)) {
                        Trigger_Power_Protection(power_state, "OVER CURRENT", sample_c, sample_v);
                        return sample_c;
                    }
                }
            } else {
                g_is_overloading = 0; // 電流降回正常，清除計時
            }
        }
        return sample_c; // 成功採樣並處理後返回值
    } else if (!(*power_state)) {
        g_is_overloading = 0; // 電源關閉時自動清除超載狀態
    }
    return -1.0f; // 未採樣或不在通電狀態
}

void Show_Test_Start_Screen(const char* title) {
    UI_Clear();
    Safe_Print_OLED(0, "%s", title); 
    Safe_Print_OLED(16, "Red Btn (Power)"); 
    Safe_Print_OLED(32, "Blue Btn (-) Reset"); 
    UI_Update(); 

    uint32_t wait_tick = 0;
    while(wait_tick < 5000) {
        Global_Background_Tasks(); if (g_force_alarm_menu) break;
        if (Check_Exit_Button()) break; 
        Delay_ms(10); wait_tick += 10;
    }
    UI_Clear();
    UI_Update(); 
    Reset_Current_Filter(); 
}

void Update_Dashboard_Display(int power_state, int rx_count, const char* specific_data_str) {
    float voltage = getBusVoltage_V(); 
    float inst_current = getCurrent_mA();
    if (inst_current == 0.0f) { 
        set237Calibration_1A(); 
        inst_current = getCurrent_mA(); 
    }
        
    UI_Clear();
    
    // ==============================================================
    // [修正] 直接利用 Safe_Print_OLED_Smooth 的格式化功能
    // 徹底廢除 static char row[33] 緩衝區，根除記憶體越界與重入風險
    // ==============================================================
    
    // 1. 排版第一行：電流平均值與最大值
    Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "AVG:%-6.1fmA   Max:%-6.1fmA", g_fCurrentAvg, g_fMaxCurrent);
    
    // 2. 排版第二行：實時電流與最小值
    Safe_Print_OLED_Smooth(16, 0, 63, 0x0F, "CUR:%-6.1fmA   Min:%-6.0fmA", inst_current, (g_fMinCurrent==9999.0f)?0:g_fMinCurrent);
    
    // 3. 排版第三行：電壓與電源狀態
    Safe_Print_OLED_Smooth(32, 0, 63, 0x0F, "%-5.2fV         [Power:%s]", voltage, power_state ? "ON " : "OFF");
    
    // 4. 排版第四行：動態外部資料 (防禦格式化漏洞的關鍵)
    if (strlen(specific_data_str) > 0 && rx_count >= 0) {
        // 直接格式化，不需再宣告 r_data 緩衝區
        Safe_Print_OLED_Smooth(48, 0, 63, 0x04, "%02d/%.28s", rx_count, specific_data_str); 
    } else {
        // 【最重要防禦】：必須強制使用 "%s" 包裝變數，防止 specific_data_str 含有 % 符號觸發漏洞
        Safe_Print_OLED_Smooth(48, 0, 63, 0x04, "%.32s", specific_data_str);
    }
    
    UI_Update(); 
}

// =======================================================
// [V5.3.6 架構升級：OLED 虛擬顯存 (VRAM) 與解耦渲染引擎]
// =======================================================
#define MAX_VRAM_LINES 8

typedef struct {
    int y;
    int min_y;
    int max_y;
    uint8_t brightness;
    char text[128]; // 改128 正常顯示 TimeSrt
    uint8_t active;
} VRAM_Line;

VRAM_Line g_vram[MAX_VRAM_LINES];

void UI_Clear(void) {
    for(int i = 0; i < MAX_VRAM_LINES; i++) {
        g_vram[i].active = 0;
    }
}

void UI_Update(void) {
    // --- [V5.3.16] Debug: Conditional Print ---
    if (g_u8DebugEnable) {
        DEBUG_PRINTF("[UI] Start Update at %u\r\n", g_u32SystemMs);
    }
    // --- END [V5.3.16] Debug ---
    NVIC_DisableIRQ(USBD_IRQn);
    OLED_Clear();
    for(int i = 0; i < MAX_VRAM_LINES; i++) {
        if(g_vram[i].active) {
            OLED_PrintString(0, g_vram[i].y, g_vram[i].min_y, g_vram[i].max_y, g_vram[i].text, g_vram[i].brightness);
        }
    }
    OLED_Update();
    NVIC_EnableIRQ(USBD_IRQn);
    // --- [V5.3.16] Debug: Conditional Print ---
    if (g_u8DebugEnable) {
        DEBUG_PRINTF("[UI] End Update at %u\r\n", g_u32SystemMs);
    }
    // --- END [V5.3.16] Debug ---
}

void Safe_Print_OLED_Smooth(int y, int min_y, int max_y, uint8_t brightness, const char *fmt, ...) {
    int idx = -1;
    for(int i = 0; i < MAX_VRAM_LINES; i++) {
        if(!g_vram[i].active) { idx = i; break; }
    }
    if(idx == -1) return; 

    char temp_buf[128];
    va_list argptr;
    va_start(argptr, fmt);
    
    NVIC_DisableIRQ(USBD_IRQn);
    vsnprintf(temp_buf, sizeof(temp_buf), fmt, argptr);
    NVIC_EnableIRQ(USBD_IRQn);
    
    va_end(argptr);

    int temp_len = strlen(temp_buf);
    for(int i = 0; i < 32; i++) {
        if(i < temp_len) {
            char c = temp_buf[i];
            g_vram[idx].text[i] = (c >= 0x20 && c <= 0x7E) ? c : ' ';
        } else {
            g_vram[idx].text[i] = ' ';
        }
    }
    g_vram[idx].text[32] = '\0';
    g_vram[idx].y = y;
    g_vram[idx].min_y = min_y;
    g_vram[idx].max_y = max_y;
    g_vram[idx].brightness = brightness;
    g_vram[idx].active = 1;
}

void Safe_Print_OLED(int y, const char *fmt, ...) {
    char temp_buf[128]; 
    va_list argptr;
    va_start(argptr, fmt);
    NVIC_DisableIRQ(USBD_IRQn);
    vsnprintf(temp_buf, sizeof(temp_buf), fmt, argptr);
    NVIC_EnableIRQ(USBD_IRQn);
    va_end(argptr);
    Safe_Print_OLED_Smooth(y, 0, 63, 0x0F, "%s", temp_buf);
}

void UI_Draw_Menu_State(const char* title, const char** items, int num_items, int curr_idx) {
    int prev_idx = (curr_idx - 1 + num_items) % num_items;
    int next_idx = (curr_idx + 1) % num_items;
    UI_Clear();
    Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, title);                 
    Safe_Print_OLED_Smooth(16, 16, 63, 0x04, "  %s", items[prev_idx]); 
    Safe_Print_OLED_Smooth(32, 16, 63, 0x0F, "> %s", items[curr_idx]); 
    Safe_Print_OLED_Smooth(48, 16, 63, 0x04, "  %s", items[next_idx]); 
    UI_Update(); 
}

void UI_Menu_Scroll_Anim_Smooth(const char* title, const char** items, int num_items, int old_idx, int dir) {
    int p_idx = (old_idx - 1 + num_items) % num_items;
    int n_idx = (old_idx + 1) % num_items;
    int nn_idx = (old_idx + 2) % num_items; 
    int pp_idx = (old_idx - 2 + num_items * 2) % num_items; 

    for (int offset = 0; offset <= 16; offset += 4) {
        UI_Clear(); Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, title); 
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
        UI_Update(); 
    }
}

// =======================================================
// [全域攔截系統與鬧鐘引擎]
// =======================================================
void Handle_Alarm_Trigger(void) {
    if (!g_alarm_triggered) return;
    uint32_t start_ms = g_u32SystemMs;
    uint32_t beep_timer = g_u32SystemMs - 2000; 
    
    UI_Clear();
    Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "================================");
    Safe_Print_OLED_Smooth(24, 0, 63, 0x0F, "      TIME'S UP! (ALARM)");
    Safe_Print_OLED_Smooth(48, 0, 63, 0x0F, "================================");
    UI_Update();

    while(g_alarm_triggered) {
        Process_UART1_JIG_8CP_Parser(); 
        USBHID_Process_Queue(); 
        uint32_t elapsed = g_u32SystemMs - start_ms;
        if (elapsed > 60000) { g_alarm_triggered = 0; break; } 

        if (g_u32SystemMs - beep_timer > 2000) {
            for(int i=0; i<4; i++) { JigForceBeep(60); Delay_ms(60); }
            beep_timer = g_u32SystemMs;
        }

        if ((PA->PIN & BIT8)==0 || (PF->PIN & (BIT3|BIT4|BIT5|BIT6)) != (BIT3|BIT4|BIT5|BIT6)) {
            JigBeep(50);
            g_alarm_triggered = 0;
            while((PA->PIN & BIT8)==0 || (PF->PIN & (BIT3|BIT4|BIT5|BIT6)) != (BIT3|BIT4|BIT5|BIT6)) { Delay_ms(10); }
            break;
        }
    }
    g_force_alarm_menu = 1; 
}

void Global_Background_Tasks(void) {
    Process_UART1_JIG_8CP_Parser();
    USBHID_Process_Queue();
    
    static uint32_t s_last_rtc_read = 0;
    if (g_u32SystemMs - s_last_rtc_read >= 500) {
        s_last_rtc_read = g_u32SystemMs; 
        
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
    }
    
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
// [V5.3.3 新架構：USB HID 非同步佇列與發送引擎]
// =======================================================
void USBHID_Enqueue_Data(const char* str) {
    while(*str) {
        uint16_t next = (g_hid_head + 1) % HID_TX_BUF_SIZE;
        if (next != g_hid_tail) {
            g_hid_tx_buf[g_hid_head] = *str++;
            g_hid_head = next;
        } else break; 
    }
}

void USBHID_Enqueue_String(const char* str) {
    while(*str) {
        uint16_t next = (g_hid_head + 1) % HID_TX_BUF_SIZE;
        if (next != g_hid_tail) { g_hid_tx_buf[g_hid_head] = *str++; g_hid_head = next; } 
        else break; 
    }
    if (g_u8UsbHidAppendCR) {
        uint16_t next = (g_hid_head + 1) % HID_TX_BUF_SIZE;
        if (next != g_hid_tail) { g_hid_tx_buf[g_hid_head] = 0x0D; g_hid_head = next; }
    }
}

int Trigger_USB_HID_Key(uint8_t mod, uint8_t key) {
    uint8_t report[8] = {0}; report[0] = mod; report[2] = key;
    uint32_t timeout = 0;
    while(g_u8EP2Ready == 0) { Delay_us(100); timeout++; if(timeout > 500) return 0; }
    g_u8EP2Ready = 0;
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), report, 8); USBD_SET_PAYLOAD_LEN(EP2, 8);
    return 1; 
}

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

void USBHID_Process_Queue(void) {
    if (g_hid_head == g_hid_tail) return; 
    if (g_u8EP2Ready) {
        char c = g_hid_tx_buf[g_hid_tail];
        g_hid_tail = (g_hid_tail + 1) % HID_TX_BUF_SIZE;
        Internal_Send_Char_HID(c);
        if (g_hid_head == g_hid_tail) {
            JigBeep(100);
        }
    }
}

// =======================================================
// [V5.3.13 Debug Output via UART0]
// =======================================================
void DEBUG_PRINTF(const char* fmt, ...) {
    if (!g_u8DebugEnable) return; // [V5.3.16] Check global enable flag

    char debug_buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(debug_buf, sizeof(debug_buf), fmt, args);
    va_end(args);
    if(len > 0 && len < sizeof(debug_buf)) {
        UART_Write(UART0, (uint8_t*)debug_buf, len);
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
    // V5.3.13 Debug Print
    DEBUG_PRINTF("[UART1_TX] Time:%u, Cmd:%s, Data:%s\r\n", g_u32SystemMs, cmd_code, data);
}

void JIG_8CP_Command_Handler(const char* cmd_code, const char* data) {
    // V5.3.13 Debug Print
    DEBUG_PRINTF("[UART1_RX] Time:%u, Cmd:%s, Data:%s\r\n", g_u32SystemMs, cmd_code, data);

    if (strcmp(cmd_code, "SC") == 0) {
        USBHID_Enqueue_String(data); 
    }
    else if (strcmp(cmd_code, "V") == 0) {
        JIG_8CP_Send_Packet("V", FIRMWARE_VERSION);
    }
    else if (strcmp(cmd_code, "WI") == 0) {
        strncpy(g_szWifiIP, data, sizeof(g_szWifiIP) - 1);
        g_szWifiIP[sizeof(g_szWifiIP) - 1] = '\0';
        g_u8WifiConnected = 1;
        // V5.3.13 Debug Print
        DEBUG_PRINTF("[COMM] Wifi Connected. IP: %s\r\n", g_szWifiIP);
        JIG_8CP_Send_Packet("GC", "?");
    }
    else if (strcmp(cmd_code, "CF") == 0) {
        float scale, c_max, v_min, v_max;
        int ov_sec;
        if (sscanf(data, "%f,%f,%f,%f,%d", &scale, &c_max, &v_min, &v_max, &ov_sec) == 5) {
            g_cfg_scale_max = scale;
            g_cfg_current_max = c_max;
            g_cfg_voltage_min = v_min;
            g_cfg_voltage_max = v_max;
            g_cfg_overload_sec = (uint8_t)ov_sec;
            g_config_received = 1; // 標記已成功接收
            // --- [V5.3.16] Debug Print ---
            if (g_u8DebugEnable) {
                DEBUG_PRINTF("Received CF: Current Limit = %.2f\r\n", g_cfg_current_max);
            }
            // --- END [V5.3.16] Debug Print ---
        }
    }
    else if (strcmp(cmd_code, "ST") == 0) {
        // 接收 ESP32 網路對時: YYYY,MM,DD,hh,mm,ss
        int y, m, d, h, min, s;
        if (sscanf(data, "%d,%d,%d,%d,%d,%d", &y, &m, &d, &h, &min, &s) == 6) {
            RTC_TimeTypeDef rtc_time;
            rtc_time.year = y;
            rtc_time.month = m;
            rtc_time.date = d;
            rtc_time.hours = h;
            rtc_time.minutes = min;
            rtc_time.seconds = s;
            RV3028_SetTime(&rtc_time);
            
            // 寫入完成回報 ESP32
            JIG_8CP_Send_Packet("ST", "OK");
        }
    }
    // --- 新增：處理 ESP32 主動發送或響應 AL? 時發送的 AL 指令 ---
    else if (strcmp(cmd_code, "AL") == 0) {
        int idx = 0, h = 0, m = 0, s = 0, en = 0;
        if (sscanf(data, "%d,%d,%d,%d,%d", &idx, &h, &m, &s, &en) == 5) {
            if (idx >= 0 && idx < 6) {
                g_alarms[idx].hours = h;
                g_alarms[idx].minutes = m;
                g_alarms[idx].seconds = s;
                g_alarms[idx].enabled = en;
            }
        }
    }
    // --- 新增：處理 ESP32 用於儲存設定的 AS 指令 (確保協議完整性) ---
    else if (strcmp(cmd_code, "AS") == 0) {
        int idx = 0, h = 0, m = 0, s = 0, en = 0;
        if (sscanf(data, "%d,%d,%d,%d,%d", &idx, &h, &m, &s, &en) == 5) {
            if (idx >= 0 && idx < 6) {
                g_alarms[idx].hours = h;
                g_alarms[idx].minutes = m;
                g_alarms[idx].seconds = s;
                g_alarms[idx].enabled = en;
            }
        }
    }
    else if (strcmp(cmd_code, "AS_OK") == 0) { 
        if (strcmp(data, "OK") == 0) {
             UI_Clear();
             Safe_Print_OLED_Smooth(24, 0, 63, 0x0F, "   SD Card Save OK!");
             UI_Update();
             Delay_ms(1000);
        }
    }
    else if (strcmp(cmd_code, "PW") == 0) {
        if (strcmp(data, "?") == 0) {
            JIG_8CP_Send_Packet("PW", g_power_state ? "ON" : "OFF");
            // --- [V5.3.16] Debug Print ---
            if (g_u8DebugEnable) {
                DEBUG_PRINTF("[COMM] Received PW?. Replying with current state: %s\r\n", g_power_state ? "ON" : "OFF");
            }
            // --- END [V5.3.16] Debug Print ---
        }
        else if (strstr(data, "ON") != NULL) {
            // --- [V5.3.16] Debug Print ---
            if (g_u8DebugEnable) {
                DEBUG_PRINTF("[COMM] Received PW ON request from WEB.\r\n");
            }
            // --- END [V5.3.16] Debug Print ---
            g_web_power_toggle_req = 1;
        } 
        else if (strstr(data, "OFF") != NULL) {
            // --- [V5.3.16] Debug Print ---
            if (g_u8DebugEnable) {
                DEBUG_PRINTF("[COMM] Received PW OFF request from WEB.\r\n");
            }
            // --- END [V5.3.16] Debug Print ---
            g_web_power_toggle_req = 2;
        }
    }
}

void Process_UART1_JIG_8CP_Parser(void) {
    static uint8_t rx_packet[128]; static uint8_t rx_idx = 0; static uint8_t is_stx_received = 0; uint8_t c;

    while(UART1_Read_Byte(&c)) {
        if (c == JIG_8CP_STX) { is_stx_received = 1; rx_idx = 0; } 
        else if (c == JIG_8CP_CR) {
            if (is_stx_received && rx_idx >= 3) {
                rx_packet[rx_idx] = '\0'; 
                char received_chk[3]; received_chk[0] = rx_packet[rx_idx - 2]; received_chk[1] = rx_packet[rx_idx - 1]; received_chk[2] = '\0';
                rx_packet[rx_idx - 2] = '\0'; 
                char calculated_chk[3]; Get_JIG_8CP_Checksum((char*)rx_packet, calculated_chk); 
                
                if (strcmp(received_chk, calculated_chk) == 0) {
                    char cmd_code[3] = {0}; 
                    char* data_str = "";
                    int payload_len = rx_idx - 2;
                    
                    if (payload_len >= 2) {
                        cmd_code[0] = rx_packet[0]; cmd_code[1] = rx_packet[1];
                        data_str = (char*)&rx_packet[2];
                    } else {
                        cmd_code[0] = rx_packet[0];
                        data_str = (char*)&rx_packet[1];
                    }
                    JIG_8CP_Command_Handler(cmd_code, data_str); 
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
    // V5.3.13 Enable UART0 Clock
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE); CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    CLK_EnableModuleClock(UART2_MODULE); CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
    CLK_EnableModuleClock(TMR0_MODULE); CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    
    CLK_EnableModuleClock(TMR1_MODULE); CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
    SystemCoreClockUpdate(); WIFIBLE_ReaderTest_init();
    
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);
    TIMER_Start(TIMER1);

    // V5.3.13 Initialize UART0 for Debug Output
    UART_Open(UART0, 115200);
    // --- [V5.3.16] Conditional Debug Print ---
    if (g_u8DebugEnable) {
        UART_Write(UART0, (uint8_t*)"Debug UART0 Init OK\r\n", 21);
    }
    // --- END [V5.3.16] Conditional Debug Print ---

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
    if(u32IntSts & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) { // <--- 修正此行
        while((UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) {
            uint8_t c = UART_READ(UART1); uint16_t next_head = (g_u1_rx_head + 1) % UART1_RX_BUF_SIZE;
            if (next_head != g_u1_rx_tail) { g_u1_rx_buf[g_u1_rx_head] = c; g_u1_rx_head = next_head; }
        }
    }
}

void UART02_IRQHandler(void) {
    uint32_t u32u2IntSts = UART2->INTSTS; 
    uint32_t u32u2FIFOSts = UART2->FIFOSTS;
    
    if(u32u2FIFOSts & (UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_RXOVIF_Msk)) { 
        UART2->FIFOSTS = (UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_RXOVIF_Msk); 
    }
    
    if(u32u2IntSts & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
        while((UART2->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) {
            uint8_t c = UART_READ(UART2); 
            if (c == 0x00 || c == 0xFF) continue;
            uint16_t next_head = (g_u2_rx_head + 1) % UART2_RX_BUF_SIZE;
            if (next_head != g_u2_rx_tail) { 
                g_u2_rx_buf[g_u2_rx_head] = c; 
                g_u2_rx_head = next_head; 
            }
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
        
        UI_Clear();
        if (mode == 0) {
            RTC_TimeTypeDef current_time;
            RV3028_GetTime(&current_time);
            int wd = Get_Weekday(current_time.year, current_time.month, current_time.date);
            
            Safe_Print_OLED(0,  "     --- TIME & DATE ---");
            Safe_Print_OLED(16, "       %04d/%02d/%02d (%s)", current_time.year, current_time.month, current_time.date, week_str[wd]);
            Safe_Print_OLED(32, "         %02d:%02d:%02d", current_time.hours, current_time.minutes, current_time.seconds);
            Safe_Print_OLED(48, " W:Set Time   Y:Back"); 
        } else {
            char y_s[8], m_s[8], d_s[8], hr_s[8], min_s[8], sec_s[8];
            
            if (mode==1 && !show_cursor) strcpy(y_s,"    "); else snprintf(y_s,8,"%04d",edit_time.year);
            if (mode==2 && !show_cursor) strcpy(m_s,"  ");   else snprintf(m_s,8,"%02d",edit_time.month);
            if (mode==3 && !show_cursor) strcpy(d_s,"  ");   else snprintf(d_s,8,"%02d",edit_time.date);
            if (mode==4 && !show_cursor) strcpy(hr_s,"  ");  else snprintf(hr_s,8,"%02d",edit_time.hours);
            if (mode==5 && !show_cursor) strcpy(min_s,"  "); else snprintf(min_s,8,"%02d",edit_time.minutes);
            if (mode==6 && !show_cursor) strcpy(sec_s,"  "); else snprintf(sec_s,8,"%02d",edit_time.seconds);
            int wd = Get_Weekday(edit_time.year, edit_time.month, edit_time.date);
            
            Safe_Print_OLED(0,  "     *** SET RTC TIME ***");
            Safe_Print_OLED(16, "       %s/%s/%s (%s)", y_s, m_s, d_s, week_str[wd]);
            Safe_Print_OLED(32, "         %s:%s:%s", hr_s, min_s, sec_s);
            Safe_Print_OLED(48, " R:Next G:+ B:- Y:Cancel");
        }
        UI_Update();

        
        uint8_t btn_exit=(PF->PIN & BIT5)?1:0, btn_next=(PF->PIN & BIT14)?1:0, btn_plus=(PF->PIN & BIT4)?1:0, btn_minus=(PF->PIN & BIT3)?1:0;
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
        UI_Clear();
        Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "      --- STOPWATCH ---");
        Safe_Print_OLED_Smooth(24, 0, 63, 0x0F, "       %02d:%02d:%02d:%03d", h, m, s, ms);
        Safe_Print_OLED_Smooth(48, 0, 63, 0x08, " W:Start/Stop  W:Rst  Y:Back");
        UI_Update();

        uint8_t red=(PF->PIN & BIT14)?1:0, blue=(PF->PIN & BIT3)?1:0, yellow=(PF->PIN & BIT5)?1:0;
        if (red == 0 && last_red == 1) { JigBeep(50); g_u8StopwatchRunning = !g_u8StopwatchRunning; }
        if (blue == 0 && last_blue == 1) { JigBeep(50); g_u32StopwatchMs = 0; }
        if (yellow == 0 && last_yellow == 1) { JigBeep(100); break; }
        last_red = red; last_blue = blue; last_yellow = yellow;
        Delay_ms(15); 
    }
}

// =======================================================
// [鬧鐘設定選單] - V5.3.6 (ESP32 SD卡儲存版) - 修改版 (V5.3.13)
// =======================================================
void Alarm_Menu_Loop(void) {
    int current_idx = 0; int mode = 0; 
    uint8_t last_red=1, last_blue=1, last_green=1, last_yellow=1, last_black=1;
    uint32_t blink_timer = 0; int show_cursor = 1;

    while(1) {
        Global_Background_Tasks(); 
        if (g_force_alarm_menu && !g_alarm_triggered) g_force_alarm_menu = 0; 
        blink_timer++; if (blink_timer > 10) { show_cursor = !show_cursor; blink_timer = 0; }
        
        UI_Clear();
        
        int p_idx = (current_idx - 1 + 6) % 6; int n_idx = (current_idx + 1) % 6;
        char buf[64];
        snprintf(buf, sizeof(buf), "  %d. [%02d:%02d:%02d] (%s)", p_idx+1, g_alarms[p_idx].hours, g_alarms[p_idx].minutes, g_alarms[p_idx].seconds, g_alarms[p_idx].enabled?"ON ":"OFF");
        Safe_Print_OLED_Smooth(0, 0, 63, 0x04, buf);

        if (mode == 0) {
            snprintf(buf, sizeof(buf), "> %d. [%02d:%02d:%02d] (%s)", current_idx+1, g_alarms[current_idx].hours, g_alarms[current_idx].minutes, g_alarms[current_idx].seconds, g_alarms[current_idx].enabled?"ON ":"OFF");
        } else {
            char h_s[8], m_s[8], s_s[8];
            if(mode==1 && !show_cursor) strcpy(h_s,"  "); else snprintf(h_s,8,"%02d",g_alarms[current_idx].hours);
            if(mode==2 && !show_cursor) strcpy(m_s,"  "); else snprintf(m_s,8,"%02d",g_alarms[current_idx].minutes);
            if(mode==3 && !show_cursor) strcpy(s_s,"  "); else snprintf(s_s,8,"%02d",g_alarms[current_idx].seconds);
            snprintf(buf, sizeof(buf), "> %d. [%s:%s:%s] (%s)", current_idx+1, h_s, m_s, s_s, g_alarms[current_idx].enabled?"ON ":"OFF");
        }
        Safe_Print_OLED_Smooth(16, 0, 63, 0x0F, buf);
        snprintf(buf, sizeof(buf), "  %d. [%02d:%02d:%02d] (%s)", n_idx+1, g_alarms[n_idx].hours, g_alarms[n_idx].minutes, g_alarms[n_idx].seconds, g_alarms[n_idx].enabled?"ON ":"OFF");
        Safe_Print_OLED_Smooth(32, 0, 63, 0x04, buf);
        // V5.3.13: Updated instruction text
        Safe_Print_OLED_Smooth(48, 0, 63, 0x08, " Blk:Save R:Set Y:Back");
        
        UI_Update();

        uint8_t red=(PF->PIN&BIT14)?1:0, blue=(PF->PIN&BIT3)?1:0, green=(PF->PIN&BIT4)?1:0, yellow=(PF->PIN&BIT5)?1:0, black=(PF->PIN&BIT6)?1:0;
        if (mode == 0) {
            if (red==0 && last_red==1) { JigBeep(50); mode=1; show_cursor=1; blink_timer=0; }
            if (blue==0 && last_blue==1) { JigBeep(50); current_idx = (current_idx + 1) % 6; }
            if (green==0 && last_green==1) { JigBeep(50); current_idx = (current_idx - 1 + 6) % 6; }
            if (black==0 && last_black==1) { // V5.3.13: Black button saves and exits
                JigBeep(100); // Sound for save button press

                // Save to ESP32 and SD Card
                UI_Clear();
                Safe_Print_OLED_Smooth(24, 0, 63, 0x0F, "   Saving to SD Card...");
                UI_Update();

                for(int i = 0; i < 6; i++) {
                    char tx_buf[32];
                    snprintf(tx_buf, sizeof(tx_buf), "%d,%02d,%02d,%02d,%d", 
                             i, g_alarms[i].hours, g_alarms[i].minutes, g_alarms[i].seconds, g_alarms[i].enabled);
                     
                    JIG_8CP_Send_Packet("AS", tx_buf); 
                    Delay_ms(80); 
                }
                // Wait for ESP32 response (optional, can add a timeout mechanism)
                // char expected_response[8];
                // snprintf(expected_response, sizeof(expected_response), "AS_OK");
                // ... Wait logic ...

                // Save complete, return to main menu
                break; // Exit the while loop
            }
            if (yellow==0 && last_yellow==1) { JigBeep(100); break; } // V5.3.13: Yellow button only exits without saving
        } else { // mode > 0, editing time/enabled
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
            if (yellow==0 && last_yellow==1) { JigBeep(100); mode=0; } // V5.3.13: Yellow button cancels editing and returns to list mode
        }
        last_red=red; last_blue=blue; last_green=green; last_yellow=yellow; last_black=black;
        Delay_ms(15);
    }
    
    // V5.3.13: Removed saving logic from here, moved to black button handler above.
}

void Time_Set_Menu_Loop(void) {
    const char *items[] = { "1. Time & Date", "2. Stopwatch", "3. Alarms" };
    int idx = 0; uint8_t last_red=1, last_blue=1, last_green=1, last_yellow=1;

    while(1) {
        Global_Background_Tasks();
        if (g_force_alarm_menu) { idx = 2; Alarm_Menu_Loop(); continue; } // 自動導航到鬧鐘
        UI_Clear();
        Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "   --- TIME SET ---      R:Next");
        Safe_Print_OLED_Smooth(16, 16, 63, (idx==0)?0x0F:0x04, "%s 1. Time & Date", (idx==0)?">":" ");
        Safe_Print_OLED_Smooth(32, 16, 63, (idx==1)?0x0F:0x04, "%s 2. Stopwatch", (idx==1)?">":" ");
        Safe_Print_OLED_Smooth(48, 16, 63, (idx==2)?0x0F:0x04, "%s 3. Alarms" , (idx==2)?">":" "); 
        UI_Update();

        uint8_t red=(PA->PIN& BIT8)?1:0, blue=(PF->PIN&BIT3)?1:0, green=(PF->PIN&BIT4)?1:0, yellow=(PF->PIN&BIT5)?1:0;
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

void Power_Monitor_Loop(void) {
    UI_Clear(); 
    Safe_Print_OLED(0, "Power Monitor"); 
    Safe_Print_OLED(16, "Red Btn (Power)"); 
    Safe_Print_OLED(32, "Wait for Module..."); 
    UI_Update(); 
    Delay_ms(1000); 

    int power_state = 0;
    uint8_t last_white = 1;

    // Use hardware timestamps for precise timing control
    uint32_t last_ui_update_ms = g_u32SystemMs; // Record last UI update time
    uint32_t last_pd_send_ms = g_u32SystemMs;   // Record last PD send time
    // --- 調整更新頻率 ---
    const uint32_t UI_UPDATE_INTERVAL_MS = 500; // UI update interval (ms) - 降低 OLED 更新頻率
    const uint32_t PD_SEND_INTERVAL_MS = 20;   // PD send interval (ms) - 保持或提高 PD 發送頻率
    // --- END 調整 ---

    Reset_Current_Filter();

    while(1) {
			   // --- [V5.3.14] Debug: Measure loop execution time ---
        uint32_t loop_start_time = g_u32SystemMs;
        // --- END [V5.3.14] Debug ---
        Global_Background_Tasks(); if (g_force_alarm_menu) break; 
        
        power_state = g_power_state;

        if (Check_Power_Toggle(&power_state)) {
            if (power_state) { 
                PC->DOUT |= BIT7; 
                // V5.3.13 Debug Print
                DEBUG_PRINTF("[POWER] Physical Toggle ON at Time: %u\r\n", g_u32SystemMs);
                JigBeep(100); 
            } 
            else { 
                PC->DOUT &= ~BIT7; 
                // V5.3.13 Debug Print
                DEBUG_PRINTF("[POWER] Physical Toggle OFF at Time: %u\r\n", g_u32SystemMs);
                JigBeep(500); 
            }
            Reset_Current_Filter(); 
            last_ui_update_ms = g_u32SystemMs; 
            last_pd_send_ms = g_u32SystemMs; // Reset PD send timer on power change
        }
				// Background sampling using new function that returns sample value
				// This function runs every 20ms regardless of UI updates
				float sampled_current = Process_Background_Sampling(&g_power_state, g_u32SystemMs);

        if (Check_Reset_Button()) { 
            last_ui_update_ms = g_u32SystemMs; 
        }

        uint8_t white = (PF->PIN & BIT14) ? 1 : 0;
        if (white == 0 && last_white == 1) {
            JigBeep(50);
            g_u8WifiConnected = 0; 
            strcpy(g_szWifiIP, "WAITING...");
            JIG_8CP_Send_Packet("WI", "?"); 
            last_ui_update_ms = g_u32SystemMs; // Force UI refresh after IP request
            last_pd_send_ms = g_u32SystemMs;   // Also reset PD timer if needed
            while((PF->PIN & BIT14) == 0) { Delay_ms(10); } 
        }
        last_white = white;

        if (Check_Exit_Button()) break;

        // ---------------------------------------------------
        // [Task A] Screen Refresh (Execute every UI_UPDATE_INTERVAL_MS)
        // ---------------------------------------------------
        if (g_u32SystemMs - last_ui_update_ms >= UI_UPDATE_INTERVAL_MS) {
            float voltage = getBusVoltage_V(); 
            // Use the most recent instantaneous current value available for display
            // (This might be slightly different from what's sent in PD if sampling is faster than UI update)
            float inst_current = getCurrent_mA(); // 或者使用最近的 sampled_current
            if (inst_current == 0.0f) { set237Calibration_1A(); inst_current = getCurrent_mA(); }

            UI_Clear();
            char buf1[33], buf2[33], buf3[33], buf4[33];
            
            snprintf(buf1, 33, "AVG:%-6.1fmA        Max:%-6.1fmA", g_fCurrentAvg, g_fMaxCurrent);
            snprintf(buf2, 33, "CUR:%-6.1fmA        Min:%-6.0fmA", inst_current, (g_fMinCurrent==9999.0f)?0:g_fMinCurrent);
            snprintf(buf3, 33, "%-5.2fV              [Power:%s]", voltage, power_state?"ON ":"OFF");

            if (g_u8WifiConnected) {
                snprintf(buf4, 33, "IP:%-14s B:Rst R:Pwr", g_szWifiIP); 
            } else if (strcmp(g_szWifiIP, "WAITING...") == 0) {
                snprintf(buf4, 33, "WIFI: WAITING... B:Rst R:Pwr");
            } else {
                snprintf(buf4, 33, "W:WIFI IP B:Rst R:Pwr Y:Exit");
            }

            Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, buf1);
            Safe_Print_OLED_Smooth(16, 0, 63, 0x0F, buf2);
            Safe_Print_OLED_Smooth(32, 0, 63, 0x0F, buf3);
            Safe_Print_OLED_Smooth(48, 0, 63, 0x04, buf4); 
            UI_Update(); 
            
            last_ui_update_ms = g_u32SystemMs; 
        }

        // ---------------------------------------------------
        // [Task B] Data Transfer ESP32 (Execute every PD_SEND_INTERVAL_MS)
        // ---------------------------------------------------
        if (g_u8WifiConnected && power_state) {
            if (g_u32SystemMs - last_pd_send_ms >= PD_SEND_INTERVAL_MS) { 
                char data_str[32];
                // Send the instantaneous value *at the moment of sending*, not necessarily the UI display value
                float inst_current = getCurrent_mA(); 
                float voltage = getBusVoltage_V();
                snprintf(data_str, sizeof(data_str), "%.1f,%.2f", inst_current, voltage);
                
                // --- [V5.3.14] Debug: Print PD send time and data ---
                DEBUG_PRINTF("[M032_PD_TX] Time: %u, Data: %s\r\n", g_u32SystemMs, data_str);
                // --- END [V5.3.14] Debug ---
                
                JIG_8CP_Send_Packet("PD", data_str);
                last_pd_send_ms = g_u32SystemMs; 
            }
        } else {
            last_pd_send_ms = g_u32SystemMs; // Ensure immediate send on reconnection/turn-on
        }

        Delay_ms(1); 
				        // V5.3.13: Removed old tick incrementers
        // ui_tick++;
        // tx_tick++;

        // --- [V5.3.14] Debug: Calculate and print loop duration ---
        uint32_t loop_end_time = g_u32SystemMs;
        uint32_t loop_duration = loop_end_time - loop_start_time;
        if (loop_duration > 25) { // If loop takes more than 25ms (expected: ~21ms based on PD freq), warn
             DEBUG_PRINTF("[LOOP] Loop took %u ms! (Expected ~%u ms)\r\n", loop_duration, PD_SEND_INTERVAL_MS);
        }
        // --- END [V5.3.14] Debug
    }
    
    PC->DOUT &= ~BIT7; g_power_state = 0; UI_Clear(); Safe_Print_OLED(0, "Monitor End"); UI_Update(); Delay_ms(1000);
}

// =======================================================
// [各式監控 UI 介面] - 修復 UART2 緩衝區競爭與幻影觸發 Bug
// =======================================================
void UART_Monitor_Test(uint32_t u32BaudRate) {
    UART_DisableInt(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk); 
    Interface_init(); 
    PB4 = 1;                
    UART_Open(UART2, u32BaudRate); 
    UART2->FIFO = (UART2->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;
    UART_EnableInt(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    
    char title_buf[32]; 
    snprintf(title_buf, sizeof(title_buf), "UART2 Mntr %u", u32BaudRate);
    Show_Test_Start_Screen(title_buf); 
    
    int rx_count = 0; 
    static char rx_buf[128];
    memset(rx_buf, 0, sizeof(rx_buf));
    
    uint32_t loop_tick = 1000; 
    int power_state = 0; 

    // 進入前先清空 UART2 的軟體與硬體緩衝區
    __disable_irq(); 
    g_u2_rx_head = 0; 
    g_u2_rx_tail = 0; 
    UART2->FIFO |= UART_FIFO_RXRST_Msk;
    __enable_irq();

    while(1) {
        Global_Background_Tasks(); 
        if (g_force_alarm_menu) break; 

        // 同步全域電源狀態
        power_state = g_power_state; 

        if (Check_Power_Toggle(&power_state)) {
            if (power_state) { 
                PC->DOUT |= BIT7; 
                JigBeep(100); 
                __disable_irq(); 
                g_u2_rx_head = 0; 
                g_u2_rx_tail = 0; 
                __enable_irq(); 
            } else { 
                PC->DOUT &= ~BIT7; 
                JigBeep(500); 
            }
            Reset_Current_Filter(); 
            loop_tick = 1000;
        }

        // 背景採樣與重置鍵偵測
        Process_Background_Sampling(&g_power_state, loop_tick);
        if (Check_Reset_Button()) loop_tick = 1000;

        // ---------------------------------------------------
        // [修正] 改由軟體 Ring Buffer 讀取，徹底解決硬體 ISR 競爭
        // ---------------------------------------------------
        if (g_power_state) { // 必須依賴全域真實狀態
            int rx_len = 0;
            int has_data = 0;
            
            // 將緩衝區內累積的字元一次性讀出 (最多 127 個)
            while(g_u2_rx_head != g_u2_rx_tail && rx_len < 127) {
                char c = g_u2_rx_buf[g_u2_rx_tail];
                g_u2_rx_tail = (g_u2_rx_tail + 1) % UART2_RX_BUF_SIZE;
                
                if(c >= 0x20 && c <= 0x7E) rx_buf[rx_len++] = c;
                else rx_buf[rx_len++] = '.';
                has_data = 1;
            }
            
            if (has_data) { 
                rx_buf[rx_len] = '\0'; 
                rx_count++; 
                JigBeep(50); 
                loop_tick = 1000; // 強制立刻刷新 OLED
            }
        } else {
            // 若未通電，持續消化並丟棄背景進來的雜訊，防止 Buffer 溢滿
            __disable_irq(); 
            g_u2_rx_head = 0; 
            g_u2_rx_tail = 0; 
            __enable_irq();
        }

        if (Check_Exit_Button()) break;
        
        if (loop_tick >= 1000) { 
            Update_Dashboard_Display(g_power_state, rx_count, rx_buf); 
            loop_tick = 0; 
        }
        
        Delay_ms(1); 
        loop_tick++;
    }
    
    // 離開介面時強制關閉硬體電源與重置全域狀態
    PC->DOUT &= ~BIT7; 
    g_power_state = 0; 
    UI_Clear(); 
    Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "Monitor End"); 
    UI_Update(); 
    Delay_ms(1000);
}
void Wiegand_Monitor_Test(void) {
    Interface_init(); PB6 = 1; PA11 = 1; GPIO_SetMode(PA, BIT10, GPIO_MODE_QUASI); GPIO_SetMode(PB, BIT5, GPIO_MODE_QUASI); GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5);
    Show_Test_Start_Screen("Wiegand Monitor"); 
    int rx_count = 0; uint32_t loop_tick = 1000; int power_state = 0; uint64_t last_wg_data = 0; 
    
    static char data_str[32];
    memset(data_str, 0, sizeof(data_str));

    while(1) {
        Global_Background_Tasks(); if (g_force_alarm_menu) break;
        
        power_state = g_power_state; // [V5.3.11] Ensure global power protection works
        
        if (Check_Power_Toggle(&power_state)) {
            if (power_state) { PC->DOUT |= BIT7; JigBeep(100); QUEUE_CLEAR(au64WG1); GPIO_CLR_INT_FLAG(PA, BIT10); GPIO_CLR_INT_FLAG(PB, BIT5); GPIO_EnableInt(PA, 10, GPIO_INT_FALLING); GPIO_EnableInt(PB, 5, GPIO_INT_FALLING);  NVIC_EnableIRQ(GPIO_PAPBPGPH_IRQn); } else { GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5); PC->DOUT &= ~BIT7; JigBeep(500); }
            Reset_Current_Filter(); loop_tick = 1000;
        }
        Process_Background_Sampling(&g_power_state, loop_tick); // [V5.3.16] Pass global state
        if (Check_Reset_Button()) loop_tick = 1000; vCheckingTimeOut();
        if (!QUEUE_EMPTY(au64WG1)) { last_wg_data = QUEUE_PULL(au64WG1); if (g_u8WiegandNum > 0) { rx_count++; JigBeep(50); loop_tick = 1000; } }
        if (Check_Exit_Button()) break;
        if (loop_tick >= 1000) {
            if (rx_count > 0) snprintf(data_str, sizeof(data_str), "W%02d:%llX", g_u8WiegandNum, last_wg_data); else strcpy(data_str, "WAITING...");
            Update_Dashboard_Display(g_power_state, rx_count, data_str); // [V5.3.16] Pass global state
            loop_tick = 0; 
        }
        Delay_ms(1); loop_tick++;
    }
    PC->DOUT &= ~BIT7; g_power_state = 0; GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5); UI_Clear(); Safe_Print_OLED(0, "Monitor End"); UI_Update(); Delay_ms(1000);
}

void Decode_TK2_Raw(char* out_str) {
    int start_idx = -1; int out_idx = 0;
    for(int i = 0; i <= TK2Cnt - 5; i++) { if(g_u8TK2Bit[i] == 0x31 && g_u8TK2Bit[i+1] == 0x31 && g_u8TK2Bit[i+2] == 0x30 && g_u8TK2Bit[i+3] == 0x31 && g_u8TK2Bit[i+4] == 0x30) { start_idx = i; break; } }
    if (start_idx != -1) { for(int i = start_idx; i <= TK2Cnt - 5; i += 5) { int b0 = g_u8TK2Bit[i]-0x30; int b1 = g_u8TK2Bit[i+1]-0x30; int b2 = g_u8TK2Bit[i+2]-0x30; int b3 = g_u8TK2Bit[i+3]-0x30; uint8_t val = b0*1 + b1*2 + b2*4 + b3*8; char c = val + 0x30; out_str[out_idx++] = c; if (c == '?') break; if (out_idx >= 30) break; } } else { strcpy(out_str, "NO_SS_ERR"); } out_str[out_idx] = '\0';
}

void TK2_Monitor_Test(void) {
    Interface_init(); PB7 = 1; PA11 = 1; GPIO_SetMode(PA, BIT10, GPIO_MODE_QUASI); GPIO_SetMode(PB, BIT5, GPIO_MODE_QUASI); GPIO_SetMode(PB, BIT8, GPIO_MODE_QUASI); GPIO_DisableInt(PA, 10); GPIO_DisableInt(PB, 5);  GPIO_DisableInt(PB, 8);
    Show_Test_Start_Screen("TK2 Monitor");
    int rx_count = 0; uint32_t loop_tick = 1000; int power_state = 0; uint8_t last_tk2_cnt = 0; uint32_t tk2_idle_tick = 0; 
    
    static char tk2_str[64];
    strcpy(tk2_str, "WAITING...");

    while(1) {
        Global_Background_Tasks(); if (g_force_alarm_menu) break;
        
        power_state = g_power_state; // [V5.3.11] Ensure global power protection works
        
        if (Check_Power_Toggle(&power_state)) {
            if (power_state) { PC->DOUT |= BIT7; JigBeep(100); TK2Cnt = 0; last_tk2_cnt = 0; tk2_idle_tick = 0; g_u8TK2Step = 0; memset((void *)g_u8TK2Bit, 0, sizeof(g_u8TK2Bit)); GPIO_CLR_INT_FLAG(PB, BIT5); GPIO_EnableInt(PB, 5, GPIO_INT_FALLING);  NVIC_EnableIRQ(GPIO_PAPBPGPH_IRQn); } else { GPIO_DisableInt(PB, 5); PC->DOUT &= ~BIT7; JigBeep(500); }
            Reset_Current_Filter(); loop_tick = 1000;
        }
        Process_Background_Sampling(&g_power_state, loop_tick); // [V5.3.16] Pass global state
        if (Check_Reset_Button()) loop_tick = 1000; vCheckingTimeOut();
        if (TK2Cnt > 0) {
            if (TK2Cnt != last_tk2_cnt) { last_tk2_cnt = TK2Cnt; tk2_idle_tick = 0; } else { tk2_idle_tick++; if (tk2_idle_tick > 50) { rx_count++; Decode_TK2_Raw(tk2_str); JigBeep(50); loop_tick = 1000; TK2Cnt = 0; last_tk2_cnt = 0; g_u8TK2Step = 0; memset((void *)g_u8TK2Bit, 0, sizeof(g_u8TK2Bit)); } }
        }
        if (Check_Exit_Button()) break;
        if (loop_tick >= 1000) { Update_Dashboard_Display(g_power_state, rx_count, tk2_str); loop_tick = 0; } // [V5.3.16] Pass global state
        Delay_ms(1); loop_tick++;
    }
    PC->DOUT &= ~BIT7; g_power_state = 0; GPIO_DisableInt(PB, 5); UI_Clear(); Safe_Print_OLED(0, "Monitor End"); UI_Update(); Delay_ms(1000);
}

void UART1_JIG_8CP_Test(void) {
    UI_Clear(); Safe_Print_OLED(0, "UART1 JIG_8CP"); Safe_Print_OLED(16, "Red Btn -> TX"); Safe_Print_OLED(32, "Wait RX Cmd:01..."); Safe_Print_OLED(48, "Yellow(Exit)->Back"); UI_Update();
    __disable_irq(); g_u1_rx_head = 0; g_u1_rx_tail = 0; __enable_irq();

    while(1) {
        Global_Background_Tasks(); if (g_force_alarm_menu) break; 
        if ((PA->PIN & BIT8) == 0) { 
            Delay_ms(50);
            if ((PA->PIN & BIT8) == 0) {
                JigForceBeep(50); JIG_8CP_Send_Packet("SC", "ABCD123");
                UI_Clear(); Safe_Print_OLED(0, "--- JIG_8CP TX ---"); Safe_Print_OLED(16, "Cmd : SC (HID)"); Safe_Print_OLED(32, "Data: ABCD123"); UI_Update();
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
    
    UI_Clear(); Safe_Print_OLED(0, "System Ready"); Safe_Print_OLED(16, "JIG-8FT-P1 OK"); Safe_Print_OLED(32, "--- BALLY ---"); UI_Update();

    JigBeep(500); Delay_ms(100); JigBeep(500); Delay_ms(1000);
    
    // V5.3.13: Add status display, wait for AL? response
    UI_Clear();
    Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "Init Comm...");
    Safe_Print_OLED_Smooth(16, 0, 63, 0x0F, "Sync Alarms...");
    UI_Update();

    JIG_8CP_Send_Packet("AL", "?");
    uint32_t sync_start_time = g_u32SystemMs;
    while (g_u32SystemMs - sync_start_time < 2000) { // Wait up to 2 seconds
        Global_Background_Tasks(); // This handles UART reception
        // Check if at least one alarm is set (e.g., first not 00:00:00)
        if (g_alarms[0].hours != 0 || g_alarms[0].minutes != 0 || g_alarms[0].seconds != 0 || g_alarms[0].enabled != 0) {
            // If first alarm has value, assume sync is complete
            break;
        }
        Delay_ms(10);
    }

    JIG_8CP_Send_Packet("GC", "?"); // [新增] 開機時向ESP32索取Config
    
    // 1. 在陣列最前面加入 "Power Monitor"
    const char *menu_items[] = { "Power Monitor", "RS232 Monitor", "Wiegand", "TK2", "Time Set", "Buzzer Settings", "USBHID SET", "UART1 JIG_8CP" };
    const int NUM_ITEMS = sizeof(menu_items) / sizeof(menu_items[0]); 
    int current_idx = 0; // 預設停在第一項 (Power Monitor)

    const char *baud_items[] = { "115200, N, 8, 1", "9600, N, 8, 1", "19200, N, 8, 1", "38400, N, 8, 1" };
    const uint32_t baud_values[] = { 115200, 9600, 19200, 38400 };
    const int NUM_BAUDS = sizeof(baud_items) / sizeof(baud_items[0]);

    while(1) {
        Global_Background_Tasks(); 
        if (g_force_alarm_menu) { current_idx = 4; Time_Set_Menu_Loop(); continue; } // Time Set 的 index 變成了 4
                
        char menu_title[64];
        snprintf(menu_title, sizeof(menu_title), "Select Function (%s)", FIRMWARE_VERSION);
        UI_Draw_Menu_State(menu_title, menu_items, NUM_ITEMS, current_idx);
        int selected = 0;

        while(1) {
            Global_Background_Tasks(); 
            if (g_force_alarm_menu) { selected = 2; break; }

            if((PF->PIN & BIT3) == 0) { Delay_ms(50); if((PF->PIN & BIT3) == 0) { JigBeep(50); UI_Menu_Scroll_Anim_Smooth(menu_title, menu_items, NUM_ITEMS, current_idx, 1); current_idx = (current_idx + 1) % NUM_ITEMS; while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break; } }
            if((PF->PIN & BIT4) == 0) { Delay_ms(50); if((PF->PIN & BIT4) == 0) { JigBeep(50); UI_Menu_Scroll_Anim_Smooth(menu_title, menu_items, NUM_ITEMS, current_idx, -1); current_idx = (current_idx - 1 + NUM_ITEMS) % NUM_ITEMS; while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break; } }
            if((PF->PIN & BIT5) == 0) { Delay_ms(50); if((PF->PIN & BIT5) == 0) { JigBeep(200); while((PF->PIN & BIT5) == 0) { Delay_ms(10); } selected = 1; break; } }
        }

        if (selected == 2) continue; // 重新整理主選單(因被鬧鐘中斷)

        if (selected == 1) {
            if (current_idx == 0) {
                Power_Monitor_Loop(); 
            }
            else if (current_idx == 1) {
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
            else if (current_idx == 2) { Wiegand_Monitor_Test(); } 
            else if (current_idx == 3) { TK2_Monitor_Test(); } 
            else if (current_idx == 4) { Time_Set_Menu_Loop(); }
            else if (current_idx == 5) { 
                int exit_buzzer = 0;
                while(1) {
                    Global_Background_Tasks(); if (g_force_alarm_menu) break;
                    UI_Clear(); Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "Buzzer Settings");
                    if (g_u8BuzzerEnabled) Safe_Print_OLED_Smooth(16, 16, 63, 0x0F, "  Current: ON "); else Safe_Print_OLED_Smooth(16, 16, 63, 0x04, "  Current: OFF");
                    Safe_Print_OLED_Smooth(32, 16, 63, 0x08, " Blue(-) -> Turn ON"); Safe_Print_OLED_Smooth(48, 16, 63, 0x08, " Green(+) -> Turn OFF"); UI_Update();
                    
                    while(1) {
                        Global_Background_Tasks(); if (g_force_alarm_menu) { exit_buzzer = 1; break; }
                        if((PF->PIN & BIT3) == 0) { Delay_ms(50); if((PF->PIN & BIT3) == 0) { g_u8BuzzerEnabled = 1; JigForceBeep(100); while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break; } }
                        if((PF->PIN & BIT4) == 0) { Delay_ms(50); if((PF->PIN & BIT4) == 0) { g_u8BuzzerEnabled = 0; while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break; } }
                        if(Check_Exit_Button()) { exit_buzzer = 1; break; }
                    }
                    if (exit_buzzer == 1) break;
                }
            }
            else if (current_idx == 6) {
                int exit_usb_test = 0;
                while(1) {
                    Global_Background_Tasks(); if (g_force_alarm_menu) break;
                    UI_Clear(); Safe_Print_OLED_Smooth(0, 0, 63, 0x0F, "USBHID SET");
                    Safe_Print_OLED_Smooth(16, 16, 63, g_u8UsbHidAppendCR ? 0x0F : 0x04, " Add CR  : %s", g_u8UsbHidAppendCR ? "ON " : "OFF");
                    Safe_Print_OLED_Smooth(32, 16, 63, g_u8UsbHidSmartCaps ? 0x0F : 0x04, " SyncCaps: %s", g_u8UsbHidSmartCaps ? "ON " : "OFF");
                    Safe_Print_OLED_Smooth(48, 16, 63, 0x04, " G:CR B:Caps R:Test"); UI_Update();
                    
                    while(1) {
                        Global_Background_Tasks(); if (g_force_alarm_menu) { exit_usb_test = 1; break; }
                        if((PF->PIN & BIT4) == 0) { Delay_ms(50); if((PF->PIN & BIT4) == 0) { g_u8UsbHidAppendCR = !g_u8UsbHidAppendCR; JigBeep(50); while((PF->PIN & BIT4) == 0) { Delay_ms(10); } break; } }
                        if((PF->PIN & BIT3) == 0) { Delay_ms(50); if((PF->PIN & BIT3) == 0) { g_u8UsbHidSmartCaps = !g_u8UsbHidSmartCaps; JigBeep(50); while((PF->PIN & BIT3) == 0) { Delay_ms(10); } break; } }
                        if((PA->PIN & BIT8) == 0) { Delay_ms(50); if((PA->PIN & BIT8) == 0) { JigForceBeep(50); UI_Clear(); Safe_Print_OLED_Smooth(16, 16, 63, 0x0F, " Sending via USB..."); UI_Update(); USBHID_Enqueue_String("BALLY-chou_test_0429"); UI_Clear(); Safe_Print_OLED_Smooth(16, 16, 63, 0x0F, " Send Success!"); UI_Update(); Delay_ms(1000); while((PA->PIN & BIT8) == 0) { Delay_ms(10); } break; } }
                        if(Check_Exit_Button()) { exit_usb_test = 1; break; }
                    }
                    if (exit_usb_test == 1) break;
                }
            }
            else if (current_idx == 7) { UART1_JIG_8CP_Test(); }
        }
    }
}