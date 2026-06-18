#ifndef __RV3028_H__
#define __RV3028_H__

#include "NuMicro.h"

// RV-3028-C7 I2C 7-bit Slave Address
#define RV3028_ADDR     0x52

// 定義時間結構體
typedef struct {
    uint16_t year;    // 完整年份，例如 2026
    uint8_t  month;   // 1 ~ 12
    uint8_t  date;    // 1 ~ 31
    uint8_t  weekday; // 0: 日, 1: 一, 2: 二 ... 6: 六
    uint8_t  hours;   // 0 ~ 23 (24小時制)
    uint8_t  minutes; // 0 ~ 59
    uint8_t  seconds; // 0 ~ 59
} RTC_TimeTypeDef;

// 函數宣告
void RV3028_Init(void);
uint8_t RV3028_SetTime(RTC_TimeTypeDef *time);
uint8_t RV3028_GetTime(RTC_TimeTypeDef *time);
int IsLeapYear(uint16_t year); // 軟體端閏年判斷輔助函數

#endif
