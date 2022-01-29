#include "DMA.h"
#include "GPIO.h"
#include <stdio.h>
#include "UART.h"
#include <string.h>
#include <stm32f407xx.h>
void button_init(void);
void UART_init(void);
void DMA1_init(DMA_Handler_t *pDMAHandle);
void USART2_Init(void);
void USART2_GPIOInit(void);
USART_Handle_t USART2_Handle;
GPIO_Handler_t USART2pins;
GPIO_Handler_t push;
DMA_Handler_t Handle_DMA1;
int i;

char msg[1024]="UART TESTINGgggggggg........xoxo \n \r ";	

void button_init(void)
{	
		push.pGPIOx = GPIOA;
		push.GPIO_PinCofg.GPIO_PinNum = GPIO_PIN_0;
		push.GPIO_PinCofg.GPIO_PinMode = GPIO_MODE_IT_FT;
		push.GPIO_PinCofg.GPIO_PinSpeed = GPIO_SPEED_HIG;
		push.GPIO_PinCofg.GPIO_PinPUPDctrl = GPIO_NPUPD;
		GPIO_Init(&push);
		GPIO_IRQInterruptConfg(IRQ_NO_EXTI0, ENABLE);
		while(1);
}


void USART2_GPIOInit(void)
{

	USART2pins.pGPIOx= GPIOA;
	
	USART2pins.GPIO_PinCofg.GPIO_PinMode= GPIO_MODE_OUT;
	USART2pins.GPIO_PinCofg.GPIO_PinSpeed= GPIO_SPEED_HIG;
	USART2pins.GPIO_PinCofg.GPIO_PinOPtype= GPIO_OUT_PP;
	USART2pins.GPIO_PinCofg.GPIO_PinMode= GPIO_MODE_ALT;	
	USART2pins.GPIO_PinCofg.GPIO_PinPUPDctrl= GPIO_NPUPD;
	USART2pins.GPIO_PinCofg.GPIO_PinAltFunMode=7;
	
	//USART Tx
	USART2pins.GPIO_PinCofg.GPIO_PinNum= GPIO_PIN_2;
	GPIO_Init(&USART2pins);
	
	USART2pins.GPIO_PinCofg.GPIO_PinMode= GPIO_MODE_IN;
	USART2pins.GPIO_PinCofg.GPIO_PinSpeed= GPIO_SPEED_HIG;
	USART2pins.GPIO_PinCofg.GPIO_PinOPtype= GPIO_OUT_PP;
	USART2pins.GPIO_PinCofg.GPIO_PinMode= GPIO_MODE_ALT;	
	USART2pins.GPIO_PinCofg.GPIO_PinPUPDctrl= GPIO_NPUPD;
	USART2pins.GPIO_PinCofg.GPIO_PinAltFunMode=7;
	
	//USART Rx
	USART2pins.GPIO_PinCofg.GPIO_PinNum= GPIO_PIN_3;
	GPIO_Init(&USART2pins);
	
	GPIO_PClkCtrl(GPIOA,ENABLE);

}



void USART2_Init(void)
{
	
	//USART_Handle_t USART2_Handle;
	USART2_Handle.pUSART= USART2;
	USART2_Handle.USART_Config.USART_wordlength= USART_WORDLEN_8;
	USART2_Handle.USART_Config.USART_stop_bits= USART_STOP_BI1_1;
	USART2_Handle.USART_Config.USART_baud_rate= USART_STD_BAUD_115200;
	USART2_Handle.USART_Config.USART_mode= USART_MODE_TX_ONLY;
	USART2_Handle.USART_Config.USART_stop_bits= USART_STOP_BI1_1;
	USART2_Handle.USART_Config.USART_wordlength= USART_WORDLEN_8;
	USART_Init(&USART2_Handle);	
}



int main (void)
{
	int b= (int) strlen(msg);
	button_init();
	USART2_GPIOInit();
	USART2_Init();
	
	Handle_DMA1.pDMAx = DMA1;
	Handle_DMA1.pSTREAM = DMA1_Stream6;
	Handle_DMA1.DMA_Cofg.DMA_Direction = DMA_Direction_M2P;
	uint32_t len = sizeof(msg); 
	Handle_DMA1.DMA_Cofg.DMA_Length = len;
	Handle_DMA1.DMA_Cofg.DMA_Message = (uint32_t) msg;
	Handle_DMA1.DMA_Cofg.DMA_Interrupt_config = ENABLE;
	GPIO_IRQInterruptConfg(17,ENABLE);
	
	
	DMA1_init(&Handle_DMA1);
	
	
	
						while(1)
							{
									
											while(!GPIO_ReadPin(GPIOA,GPIO_PIN_0))
												{
															for(i=0;i<b;i++)
																	{
																			USART_SendData(USART2,(uint8_t*)(msg+i));
																	}

													
												}
							}
	
	
	
	
	
	
	while(1);
	
}

void EXTI0_IRQHandler(void)
{
	EXTI_TypeDef *pEXTI;
	pEXTI = EXTI;
	
	if( EXTI->PR & (1<<13))
	{
		EXTI->PR |= (1<<13);		//clears the pending bit
	}
	

}

void DMA1_Stream6_IRQHandler(void)
{


if(is_HT6())
{	DMA1->HISR |= (1 << 20);
}
	is_FT6()			DMA1->HISR & (1 << 21)	
	is_TE6()			DMA1->HISR & (1 << 19)
	is_FE6()			DMA1->HISR & (1 << 16)
	is_DME6()			DMA1->HISR & (1 << 18)

	
}













