/*
* Author: Ken Chen, 2014/2/20
* 
* File Name: OLED.h
*
* Version: 0.0.2 
* 
* Description: OLED(SSD1309) Driver for Nuvoton Chip(Model Name - Nuc120).
*
* Copyright(c) 2014 Gigatms Technology Corp. All rights reserved.    
*/

// #include "DrvGPIO.h"
// #include "DrvSPI.h"
#include "NuMicro.h"
#ifndef __OLED_H__
#define __OLED_H__



#define OLED_RESET_PIN 		PD, BIT15
#define OLED_CMD_DATA_PIN 	PF, BIT15
// #define OLED_DISPLAY_PIN 	PC, BIT0

#define OLED_COMMAND_MODE 	PF15=0
#define OLED_DATAMODE		PF15=1

// #define OLED_DISPLAY_OFF 	PC0=0
// #define OLED_DISPLAY_ON  	PC0=1

#define OLED_RESET_LOW		PD15=0
#define OLED_RESET_HIGH		PD15=1


// OLED init , call this in initialize.
void vOLED_INIT(void);

// u8Line will be 1 or 2. 
void vPrintText(uint8_t u8Line,char *str,...);

void vPrintLogNumber(char *str,...);

int show_icon(int level);

void vPrintNumber(char *str,...);

int Clear_All(void);

int Clear_DDR(void);

int show_korea(int pos, int index);

int Fill_Screen_All(uint8_t u8GrayScale);

// 這些宣告必須放在 .h 檔，別的 .c 檔才看得到
void vPrintAsciiRaw(int rawStr, char *str, ...);
//void vPrintAsciiRaw2(int rawStr, int column16, char *str, char *str2, ...);
void vPrintAsciiRaw2(int rawStr, int column16, char *strLeft, char *strRight);//G AI修正


/* --- 請加入到 OLED.h 裡面 --- */

/* 1. 告訴 main.c 這裡有個繪圖函式 */
void Draw_Mario(int x, int y, const unsigned char *bitmap);

/* 2. 告訴 main.c 這裡有瑪莉歐的圖片資料 (使用 extern) */
extern const unsigned char Mario_Run1[];
extern const unsigned char Mario_Run2[];
extern const unsigned char Mario_Jump[];
/* 如果素材放 main.c，這步跳過；如果放 OLED.c，請在 OLED.h 加入： */
extern const unsigned char Block_Q[];
extern const unsigned char Block_Empty[];
extern const unsigned char Item_Coin[];
extern const unsigned char Item_Mushroom[];
extern const unsigned char  Mario_Clear[];
/* --------------------------- */
#endif
