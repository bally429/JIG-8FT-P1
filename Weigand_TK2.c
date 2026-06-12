#include <stdio.h>
#include <string.h>    // <--- 補上這一行
#include "NuMicro.h"

#include "queue.h"

QUEUE_U64_CREATE(au64WG1,128);

uint8_t g_WG_Mode = 0;

typedef struct
{
	uint8_t u1MSB_First:1;	// 1, MSB First
	uint8_t u1Parity:1;		// 1, Enable checking parity
	uint8_t u1Odd:1;		// Odd parity keeps if u1Parity is enabled
	uint8_t u1Even:1;	// Even parity keeps if u1Parity is enabled
	uint8_t u1Odd_35:1;	// Odd parity keeps if u1Parity is enabled
	uint8_t u1Error:1;	// Check parity error or incomplete packet
	uint8_t u1Done:1;	// Packet done

	// Check when u1Parity is enabled
	// 1111: Accept all Weigand, auto check with timeout

	uint8_t u1Reserve:1;

	uint8_t u8Reserve;
	uint8_t u8BitCnt;
	uint32_t u32TimerTick;
	uint32_t u32WG_TimerTick;
	
	uint64_t u64Data;

}WG_PRPTY;

typedef struct
{
	uint8_t u1Parity:1;		// 1, Enable checking parity
	uint8_t u1Odd:1;		// Odd parity keeps if u1Parity is enabled
	uint8_t u1Even:1;	// Even parity keeps if u1Parity is enabled
	uint8_t u1Error:1;	// Check parity error or incomplete packet
	uint8_t u1Done:1;	// Packet done
	uint8_t u1Reserve:3;
	
	uint8_t u8ByteCnt;
	uint8_t u8BitCnt;
	uint8_t u8DataCount;
	
	uint32_t u32TimerTick;
	uint32_t u32TK2_TimerTick;
	
	uint8_t au8Data[32];

}TK2_PRPTY;


static WG_PRPTY wg_prop;
static TK2_PRPTY TK2_prop;

static uint8_t m_ua8Data[9] = {0};

uint8_t u8TK2_GetUID(uint8_t *str_out){
	uint8_t i = 0;;
	if(TK2_prop.u1Done)
	{
		TK2_prop.u1Done = 0;
		
		for(i = 0; i < sizeof(TK2_prop.au8Data);i++){
			if(TK2_prop.au8Data[i] == 0x0f) break;
			
			str_out[i] = TK2_prop.au8Data[i];
		
		}
	
		TK2_prop.u8BitCnt = 0;
			
		memset((void *)TK2_prop.au8Data, 0, sizeof(TK2_prop.au8Data));
	}
	
	return i;
}



void vWeigandSiganl()
{

	wg_prop.u8BitCnt++;
	
	wg_prop.u32WG_TimerTick = wg_prop.u32TimerTick;
}


// Start 10 bit 0
// SS
// Data 14 digit
// ES
// LCR
// 10 TRAILING ZERO
#define TK2_START 	0
#define TK2_SS 		1
#define TK2_DATA 	2 
#define TK2_ES 		3
#define TK2_LCR		4

uint8_t g_u8TK2Step = 0;
uint8_t g_u8TK2Bit[128] = {0},TK2Cnt;

void GPABGH_IRQHandler(void)
{
	// Data 1 / Clock
	if(GPIO_GET_INT_FLAG(PB, BIT5))
    {
	
		m_ua8Data[wg_prop.u8BitCnt/8] |= (0x01 << (wg_prop.u8BitCnt%8));
		
		vWeigandSiganl();
		
        /* Clear PB.5 interrupt flag */
        GPIO_CLR_INT_FLAG(PB, BIT5);
		
		{
			static uint8_t u8SS = 0;
			uint8_t u8LCR = 0;
			uint8_t u8Bit;
			
			if(PA10 == 0) u8Bit = 1;
			else u8Bit = 0;
			
			if(u8Bit == 1) g_u8TK2Bit[TK2Cnt++] = 0x31;
			else g_u8TK2Bit[TK2Cnt++] = 0x30;
	
			
			TK2_prop.u32TK2_TimerTick = TK2_prop.u32TimerTick;
			
			switch(g_u8TK2Step)
			{
				case TK2_START:
					
					if(u8Bit == 1){
						g_u8TK2Step = TK2_SS;
						TK2_prop.u8BitCnt = 0;
						u8SS = 0;
					}
					else break;
				
				case TK2_SS:
					if(TK2_prop.u8BitCnt <= 3)
					{
						if(u8Bit == 1) 
							u8SS |= (1 << TK2_prop.u8BitCnt);
						else 
							u8SS &= (~(1 << TK2_prop.u8BitCnt));
					}
					

					TK2_prop.u8BitCnt ++;
				
					if(TK2_prop.u8BitCnt == 5){
						 
						g_u8TK2Step = TK2_DATA;
						TK2_prop.u8BitCnt = 0;
						TK2_prop.u8ByteCnt = 0;
					}
				
					break;
				case TK2_DATA:
					// LSB FIRST
				
					if(TK2_prop.u8BitCnt <= 3){
						if(u8Bit == 1) 
							TK2_prop.au8Data[TK2_prop.u8ByteCnt] |= (1 << TK2_prop.u8BitCnt);
						else 
							TK2_prop.au8Data[TK2_prop.u8ByteCnt] &= (~(1 << TK2_prop.u8BitCnt));
					}
						//TK2_prop.au8Data[TK2_prop.u8ByteCnt] |=  ((!PA8) << TK2_prop.u8BitCnt);
				
					TK2_prop.u8BitCnt ++;
				
					if(TK2_prop.u8BitCnt == 5){
						
						//g_u8TK2Step = TK2_DATA;
						TK2_prop.u8BitCnt = 0;
						
						if(TK2_prop.au8Data[TK2_prop.u8ByteCnt] == 0x0f){
							g_u8TK2Step = TK2_LCR;
						}
						else TK2_prop.au8Data[TK2_prop.u8ByteCnt] += 0x30;
						
						TK2_prop.u8ByteCnt++;

					}
				
					break;
				case TK2_ES:
					
					break;
				case TK2_LCR:
					u8LCR |= ((!PA8) << TK2_prop.u8BitCnt);
				
					TK2_prop.u8BitCnt ++;
				
					if(TK2_prop.u8BitCnt == 5){
						 
						// g_u8TK2Step = TK2_DATA;
						TK2_prop.u8BitCnt = 0;
						TK2_prop.u1Done = 1;
						g_u8TK2Step = TK2_START;
					}	
				
				
					break;
				default:
					break;
			}
		}
    }
	// Data 0 / Data
	else if(GPIO_GET_INT_FLAG(PA, BIT10))
    {
		m_ua8Data[wg_prop.u8BitCnt/8] &= (~(0x01 << (wg_prop.u8BitCnt%8)));
		
		vWeigandSiganl();
		
        /* Clear PA.8 interrupt flag */
        GPIO_CLR_INT_FLAG(PA, BIT10);
    }
	else 
	{
		/* Un-expected interrupt. Just clear all PB interrupts */
        PB->INTSRC = PB->INTSRC;
		
		/* Un-expected interrupt. Just clear all PA interrupts */
        PA->INTSRC = PA->INTSRC;
	}
}





void GPCDEF_IRQHandler(void)
{
	if(GPIO_GET_INT_FLAG(PF, BIT14))
    {
	
		
		
        /* Clear PF.14 interrupt flag */
        GPIO_CLR_INT_FLAG(PF, BIT14);
    }
	
}

//**************************************************************************************************************





//#define WGP(x) wg_prop.#x
//#define WGPS(x,y) wg_prop.#x = y

/*******************************************************************************
 * Variables
 ******************************************************************************/






/*******************************************************************************
 * Code
 ******************************************************************************/

void vWeigand_N(uint8_t u8N){
	int i;

	wg_prop.u1Even  = 0;
	wg_prop.u1Odd  = 0;

	for(i = 0; i < u8N/2; i++)
	{
		if(i < 8) 			wg_prop.u1Even 		+= (m_ua8Data[0] >> i) & 0x01;
		else if(i < 16)		wg_prop.u1Even 		+= (m_ua8Data[1] >> (i - 8)) & 0x01;
		else if(i < 24)		wg_prop.u1Even 		+= (m_ua8Data[2] >> (i - 16)) & 0x01;
		else if(i < 32)		wg_prop.u1Even 		+= (m_ua8Data[3] >> (i - 24)) & 0x01;
		else if(i < 40)		wg_prop.u1Even 		+= (m_ua8Data[4] >> (i - 32)) & 0x01;
	}

	for(i = u8N/2; i < u8N; i++)
	{
		if(i < 16)		wg_prop.u1Odd 		+= (m_ua8Data[1] >> (i - 8)) & 0x01;
		else if(i < 24)		wg_prop.u1Odd 		+= (m_ua8Data[2] >> (i - 16)) & 0x01;
		else if(i < 32)		wg_prop.u1Odd 		+= (m_ua8Data[3] >> (i - 24)) & 0x01;
		else if(i < 40)		wg_prop.u1Odd 		+= (m_ua8Data[4] >> (i - 32)) & 0x01;
		else if(i < 48)		wg_prop.u1Odd 		+= (m_ua8Data[5] >> (i - 40)) & 0x01;
		else if(i < 56)		wg_prop.u1Odd 		+= (m_ua8Data[6] >> (i - 48)) & 0x01;
		else if(i < 64)		wg_prop.u1Odd 		+= (m_ua8Data[7] >> (i - 56)) & 0x01;
		else 				wg_prop.u1Odd 		+= (m_ua8Data[8] >> (i - 64)) & 0x01;
	}


	wg_prop.u1Done = 1;
	wg_prop.u64Data = 0;

	for(i = 1; i < u8N - 1; i ++)
	{
		wg_prop.u64Data |= ((m_ua8Data[i/8] >> i%8) & 0x01);

		if(i != u8N - 2)
			wg_prop.u64Data <<= 1;
	}

	if(wg_prop.u1Even == 0 && wg_prop.u1Odd == 1) wg_prop.u1Error = 0;
	else wg_prop.u1Error = 1;

	wg_prop.u8BitCnt = 0;

	if(!wg_prop.u1Error)
	{
		QUEUE_PUT(au64WG1, wg_prop.u64Data);
	}
}

void vWeigand37()
{
	int i;

	wg_prop.u1Even  = 0;
	wg_prop.u1Odd  = 0;

	for(i = 0; i < 19; i++)
	{
		if(i < 8) 			wg_prop.u1Even 		+= (m_ua8Data[0] >> i) & 0x01;
		else if(i < 16)		wg_prop.u1Even 		+= (m_ua8Data[1] >> (i - 8)) & 0x01;
		else 				wg_prop.u1Even 		+= (m_ua8Data[2] >> (i - 16)) & 0x01;
	}

	for(i = 18; i < 37; i++)
	{
		if(i < 24) wg_prop.u1Odd 		+= (m_ua8Data[2] >> (i - 16)) & 0x01;
		else if(i < 32) wg_prop.u1Odd 	+= (m_ua8Data[3] >> (i - 24)) & 0x01;
		else wg_prop.u1Odd 				+= (m_ua8Data[4] >> (i - 32)) & 0x01;
	}

	wg_prop.u1Done = 1;
	wg_prop.u64Data = 0;

	for(i = 1; i < 36; i ++)
	{
		wg_prop.u64Data |= ((m_ua8Data[i/8] >> i%8) & 0x01);

		if(i != 35)
			wg_prop.u64Data <<= 1;
	}

	if(wg_prop.u1Even == 0 && wg_prop.u1Odd == 1) wg_prop.u1Error = 0;
	else wg_prop.u1Error = 1;

	wg_prop.u8BitCnt = 0;

	if(!wg_prop.u1Error)
	{
		QUEUE_PUT(au64WG1, wg_prop.u64Data);
	}
}

void vWeigand35()
{
	int i, Ignore = 4;

	wg_prop.u1Even  = 0;
	wg_prop.u1Odd  = 0;
	wg_prop.u1Odd_35 = 0;

	for(i = 1; i < 35; i++)
	{
		if(i == Ignore){

			Ignore += 3;
			continue;
		}

		if(i < 8) 			wg_prop.u1Even 		+= (m_ua8Data[0] >> i) & 0x01;
		else if(i < 16)		wg_prop.u1Even 		+= (m_ua8Data[1] >> (i - 8)) & 0x01;
		else if(i < 24)		wg_prop.u1Even 		+= (m_ua8Data[2] >> (i - 16)) & 0x01;
		else if(i < 32)		wg_prop.u1Even 		+= (m_ua8Data[3] >> (i - 24)) & 0x01;
		else 				wg_prop.u1Even 		+= (m_ua8Data[4] >> (i - 32)) & 0x01;
	}

	Ignore = 3;

	for(i = 1; i < 35; i++)
	{
		if(i == Ignore){

			Ignore += 3;
			continue;
		}

		if(i < 8) 			wg_prop.u1Odd 		+= (m_ua8Data[0] >> i) & 0x01;
		else if(i < 16)		wg_prop.u1Odd 		+= (m_ua8Data[1] >> (i - 8)) & 0x01;
		else if(i < 24)		wg_prop.u1Odd 		+= (m_ua8Data[2] >> (i - 16)) & 0x01;
		else if(i < 32)		wg_prop.u1Odd 		+= (m_ua8Data[3] >> (i - 24)) & 0x01;
		else 				wg_prop.u1Odd 		+= (m_ua8Data[4] >> (i - 32)) & 0x01;
	}


	for(i = 0; i < 35; i++)
	{
		if(i == Ignore){

			Ignore += 3;
			continue;
		}

		if(i < 8) 			wg_prop.u1Odd_35 		+= (m_ua8Data[0] >> i) & 0x01;
		else if(i < 16)		wg_prop.u1Odd_35 		+= (m_ua8Data[1] >> (i - 8)) & 0x01;
		else if(i < 24)		wg_prop.u1Odd_35 		+= (m_ua8Data[2] >> (i - 16)) & 0x01;
		else if(i < 32)		wg_prop.u1Odd_35 		+= (m_ua8Data[3] >> (i - 24)) & 0x01;
		else 				wg_prop.u1Odd_35 		+= (m_ua8Data[4] >> (i - 32)) & 0x01;
	}

	wg_prop.u1Done = 1;

	wg_prop.u64Data = 0;

	for(i = 2; i < 34; i ++)
	{
		wg_prop.u64Data |= ((m_ua8Data[i/8] >> i%8) & 0x01);

		if(i != 33)
			wg_prop.u64Data <<= 1;
	}

	if(wg_prop.u1Even == 0 && wg_prop.u1Odd == 1  && wg_prop.u1Odd_35 == 1) wg_prop.u1Error = 0;
	else wg_prop.u1Error = 1;

	wg_prop.u8BitCnt = 0;

	if(!wg_prop.u1Error)
	{
		if(QUEUE_FULL(au64WG1))
			QUEUE_PUT(au64WG1, wg_prop.u64Data);
	}
}
uint8_t g_u8WeigandNum = 0;
void vGetWeigandData(uint8_t u8Bits)
{
	g_u8WeigandNum = u8Bits;
	
	if(u8Bits == 35){

		vWeigand35();
	}
	else if(u8Bits == 37){
		vWeigand37();
	}
	else{
		vWeigand_N(u8Bits);
	}

}

// Call by system tick per 1 ms second
void vCheckingTimeOut()
{
	wg_prop.u32TimerTick ++;
	TK2_prop.u32TimerTick++;
	
	if(TK2_prop.u32TimerTick - TK2_prop.u32TK2_TimerTick > 100) //2026/0506 原20改100 接收在長一些時間
	{
		if(TK2Cnt){
			memset((void *) g_u8TK2Bit, 0, sizeof(g_u8TK2Bit));
			TK2Cnt = 0;
		}
		
		if(g_u8TK2Step){
			g_u8TK2Step = TK2_START;
			TK2_prop.u8BitCnt = 0;
			
			memset((void *)TK2_prop.au8Data, 0, sizeof(TK2_prop.au8Data));
		}
		
	}
	
	if(wg_prop.u32TimerTick - wg_prop.u32WG_TimerTick > 100 && wg_prop.u8BitCnt != 0) //2026/0506 原20改100 接收在長一些時間
	{
//		if(SE100.u4WGSelect == WG_AUTO){

			vGetWeigandData(wg_prop.u8BitCnt);
//		}

		wg_prop.u8BitCnt= 0;
	}
}

void vWeigand_SensorInit()
{
	


}


