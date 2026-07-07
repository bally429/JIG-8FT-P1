#include "RV3028.h"

// 內部輔助函數：十進位與 BCD 碼互轉
static uint8_t DEC2BCD(uint8_t dec) { return ((dec / 10) << 4) + (dec % 10); }
static uint8_t BCD2DEC(uint8_t bcd) { return ((bcd >> 4) * 10) + (bcd & 0x0F); }

// 底層 I2C 寫入暫存器
static void RV3028_WriteReg(uint8_t reg, uint8_t data) {
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);
    I2C_SET_DATA(I2C0, (RV3028_ADDR << 1) | 0x00); // Write
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI); I2C_WAIT_READY(I2C0);
    I2C_SET_DATA(I2C0, reg);
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI); I2C_WAIT_READY(I2C0);
    I2C_SET_DATA(I2C0, data);
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI); I2C_WAIT_READY(I2C0);
    I2C_STOP(I2C0);
}

// 底層 I2C 讀取暫存器
static uint8_t RV3028_ReadReg(uint8_t reg) {
    uint8_t data;
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);
    I2C_SET_DATA(I2C0, (RV3028_ADDR << 1) | 0x00); // Write
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI); I2C_WAIT_READY(I2C0);
    I2C_SET_DATA(I2C0, reg);
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI); I2C_WAIT_READY(I2C0);
    
    I2C_START(I2C0); // Repeated Start
    I2C_WAIT_READY(I2C0);
    I2C_SET_DATA(I2C0, (RV3028_ADDR << 1) | 0x01); // Read
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI); I2C_WAIT_READY(I2C0);
    
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI); // Read with NACK
    I2C_WAIT_READY(I2C0);
    data = I2C_GET_DATA(I2C0);
    I2C_STOP(I2C0);
    return data;
}

// 軟體判斷閏年 (雖然 RV-3028 硬體會自動算，但軟體UI可能需要)
int IsLeapYear(uint16_t year) {
    if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) return 1;
    return 0;
}

// 初始化 RV3028
void RV3028_Init(void) {
// 假設您有 RV3028_ReadReg 與 RV3028_WriteReg 函式
uint8_t backup_reg = RV3028_ReadReg(0x37);

// 清除 Bit 3 和 Bit 2
backup_reg &= ~(0x0C); 

// 設定 Bit 3:2 為 11 (Level Switching Mode - 自動切換備用電池)
backup_reg |= (0x0C);  

// 如果您使用的是「超級電容 (Supercap)」或「可充電電池」，還需要打開 TCE (涓流充電 Bit 5)
// backup_reg |= (1 << 5); // 取消此行註解以開啟充電

RV3028_WriteReg(0x37, backup_reg);
}

// 設定時間
uint8_t RV3028_SetTime(RTC_TimeTypeDef *time) {
    // RV-3028 只存年份的後兩碼 (例如 2026 -> 26)
    uint8_t short_year = time->year % 100;

    RV3028_WriteReg(0x00, DEC2BCD(time->seconds));
    RV3028_WriteReg(0x01, DEC2BCD(time->minutes));
    RV3028_WriteReg(0x02, DEC2BCD(time->hours));
    RV3028_WriteReg(0x03, DEC2BCD(time->weekday));
    RV3028_WriteReg(0x04, DEC2BCD(time->date));
    RV3028_WriteReg(0x05, DEC2BCD(time->month));
    RV3028_WriteReg(0x06, DEC2BCD(short_year));
    return 1;
}

// 讀取時間
uint8_t RV3028_GetTime(RTC_TimeTypeDef *time) {
    time->seconds = BCD2DEC(RV3028_ReadReg(0x00) & 0x7F);
    time->minutes = BCD2DEC(RV3028_ReadReg(0x01) & 0x7F);
    time->hours   = BCD2DEC(RV3028_ReadReg(0x02) & 0x3F);
    time->weekday = BCD2DEC(RV3028_ReadReg(0x03) & 0x07);
    time->date    = BCD2DEC(RV3028_ReadReg(0x04) & 0x3F);
    time->month   = BCD2DEC(RV3028_ReadReg(0x05) & 0x1F);
    time->year    = BCD2DEC(RV3028_ReadReg(0x06)) + 2000; // 轉回西元年
    return 1;
}