#include "DMA.h"
#include <stdio.h>
#include <string.h>
#include <stm32f407xx.h>
uint8_t COND0,COND1,COND2,COND3,COND4,COND5,COND6,COND7;
void DMA1_PClkCtrl( DMA_TypeDef *pDMAx, uint8_t ENorDA)
{
	if(ENorDA == ENABLE)
	{
			switch( (int)pDMAx)
			{
				case (int)DMA1:
					DMA1_PCLK_EN;
					break;
				
				case (int)DMA2:
					DMA2_PCLK_EN;
					break;
			}
	
	}
	else
	{
			switch( (int)pDMAx)
			{
				case (int)DMA1:
					DMA1_PCLK_DA;
					break;
				
				case (int)DMA2:
					DMA2_PCLK_DA;
					break;
			}
	}
}


void DMA1_init(DMA_Handler_t *pDMAHandle)
{ 
	//ENABLE THE CLOCK
	DMA1_PClkCtrl(pDMAHandle->pDMAx,ENABLE);
	//IDENTIFY THE SUITABLE STREAM(ch /4 ,stream 6)
	//IDENTIFY THE CHANNEL OF THE PERIPHERAL
	pDMAHandle->pSTREAM->CR|= (0x4<<25);
	//PROGRAM THE SOURCE ADDRESS
	pDMAHandle->pSTREAM->M0AR = pDMAHandle->DMA_Cofg.DMA_Message;
	//PROGRAM THE DESTINATION ADDRESS
	pDMAHandle->pSTREAM->PAR = (uint32_t) &USART2->DR;
	//NUMBER OF DATA ITEMS
	pDMAHandle->pSTREAM->NDTR= pDMAHandle->DMA_Cofg.DMA_Length;
	//DIRECTION OF DATA TRANSFER
	pDMAHandle->pSTREAM->CR |= pDMAHandle->DMA_Cofg.DMA_Direction;	
	//AUTO MEMORY INCREMENT
	pDMAHandle->pSTREAM->CR |= (1<<10);
	//PROGRAM THE SOURCE AND DESTINATION DATA WIDTH (default)
	//MODE FIFO OR DIRECT
	if(pDMAHandle->DMA_Cofg.DMA_Mode == 1)
	{
		pDMAHandle->pSTREAM->FCR |= (1<<2);
		pDMAHandle->pSTREAM->FCR |= (3<<0);
	}		
	//CIRCULAR MODE IF NEEDED	(def)
	//SINGLE OR BURST TRANSFER(def)
	//STREAM PRIORITY(def)
	//Interrupt config
	if(pDMAHandle->DMA_Cofg.DMA_Interrupt_config ==ENABLE)
	{
			pDMAHandle->pSTREAM->CR |= (1<<3);
			pDMAHandle->pSTREAM->CR |= (1<<4);
			pDMAHandle->pSTREAM->CR |= (1<<2);
			pDMAHandle->pSTREAM->FCR |= (1<<7);
			pDMAHandle->pSTREAM->CR |= (1<<1);
	}
	
	if (pDMAHandle->pSTREAM == DMA1_Stream6)
	{
		COND6= SET;
	}
	
#if COND0
	#define is_HT0()			DMA1->LISR & (1 << 4)
	#define is_FT0()			DMA1->LISR & (1 << 5)	
	#define is_TE0()			DMA1->LISR & (1 << 3)
	#define is_FE0()			DMA1->LISR & (1 << 0)
	#define is_DME0()		DMA1->LISR & (1 << 2)
#elif COND1
	#define is_HT1()			DMA1->LISR & (1 << 10)
	#define is_FT1()			DMA1->LISR & (1 << 11)	
	#define is_TE1()			DMA1->LISR & (1 << 9)
	#define is_FE1()			DMA1->LISR & (1 << 6)
	#define is_DME1()		DMA1->LISR & (1 << 8)	
#elif COND2
	#define is_HT2()			DMA1->LISR & (1 << 20)
	#define is_FT2()			DMA1->LISR & (1 << 21)	
	#define is_TE2()			DMA1->LISR & (1 << 19)
	#define is_FE2()			DMA1->LISR & (1 << 16)
	#define is_DME2()		DMA1->LISR & (1 << 18)
#elif COND3
	#define is_HT3()			DMA1->LISR & (1 << 26)
	#define is_FT3()			DMA1->LISR & (1 << 27)	
	#define is_TE3()			DMA1->LISR & (1 << 25)
	#define is_FE3()			DMA1->LISR & (1 << 22)
	#define is_DME3()		DMA1->LISR & (1 << 24)
#elif COND4
	#define is_HT4()			DMA1->HISR & (1 << 4)
	#define is_FT4()			DMA1->HISR & (1 << 5)	
	#define is_TE4()			DMA1->HISR & (1 << 3)
	#define is_FE4()			DMA1->HISR & (1 << 0)
	#define is_DME4()		DMA1->HISR & (1 << 2)
#elif COND5
	#define is_HT5()			DMA1->HISR & (1 << 10)
	#define is_FT5()			DMA1->HISR & (1 << 11)	
	#define is_TE5()			DMA1->HISR & (1 << 9)
	#define is_FE5()			DMA1->HISR & (1 << 6)
	#define is_DME5()		DMA1->HISR & (1 << 8)	
#elif COND6
	#define is_HT6()			DMA1->HISR & (1 << 20)
	#define is_FT6()			DMA1->HISR & (1 << 21)	
	#define is_TE6()			DMA1->HISR & (1 << 19)
	#define is_FE6()			DMA1->HISR & (1 << 16)
	#define is_DME6()		DMA1->HISR & (1 << 18)
#elif COND7
	#define is_HT7()			DMA1->HISR & (1 << 26)
	#define is_FT7()			DMA1->HISR & (1 << 27)	
	#define is_TE7()			DMA1->HISR & (1 << 25)
	#define is_FE7()			DMA1->HISR & (1 << 22)
	#define is_DME7()		DMA1->HISR & (1 << 24)

#endif
		
	
	//ENABLE THE STREAM
	pDMAHandle->pSTREAM->CR |= (1<<0);
}
	
	
void DMA1_strem6_interrupt_def()
{
	#define if_HT()				DMA1
	#define if_FT()
	#define if_TE()
	#define if_FE()
	#define if_DME()
	
	
	

	
}











