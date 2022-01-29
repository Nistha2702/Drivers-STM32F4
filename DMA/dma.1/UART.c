#include <stdint.h>
#include <stm32f407xx.h>
#include <stdint.h>
#include "GPIO.h"
#include "UART.h"

uint16_t AHB_Prescaler[9] = {2,4,8,16,32,64,128,256,512};
uint8_t  APBx_Prescaler[4] = {2,4,8,16};


 void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnorDi)
 {
	 if(EnorDi==ENABLE)
	 {
		 switch((int)pUSARTx)
		 {
			 			
					 case (int)USART1:
							USART1_PCLK_EN;
										break;
						
						case (int)USART2:
							USART2_PCLK_EN;	
												break;
						
						case (int)USART3:
							USART3_PCLK_EN;
													break;
						
						case (int)UART4:
							UART4_PCLK_EN;
													break;
							
						case (int)UART5:
							UART5_PCLK_EN;
													break;
						
						case (int)USART6:
							USART6_PCLK_EN;
												break;
						

		}		 
	 }
	 else
	{
		switch((int)pUSARTx)
		{
						case (int)USART1:
						//GPIOA_PCLK_DI;
						break;
						
						case (int)USART2:
						//GPIOB_PCLK_DI;	
						break;
						
						case (int)USART3:
						//GPIOC_PCLK_DI;	
						break;
		}
  }
}
 

 void USART_Init(USART_Handle_t *pUSARTHandle)
{  
	USART_PeriClockControl(pUSARTHandle->pUSART , ENABLE);
	
	//pUSARTHandle->pUSART->CR1|= 1<<13;
	pUSARTHandle->pUSART->CR1|=pUSARTHandle->USART_Config.USART_wordlength<<12 ;
	pUSARTHandle->pUSART->CR2|=pUSARTHandle->USART_Config.USART_stop_bits<<12 ;
	//USART_SetBaudRate(pUSARTHandle->pUSART, pUSARTHandle->USART_Config.USART_baud_rate);
	//USART2->BRR = (0x1D4C);//9600
	USART2->BRR = (0x271) ; // 115200 Baud
	pUSARTHandle->pUSART->CR1|=1<<3 ;
	pUSARTHandle->pUSART->CR1|=1<<2 ;
	pUSARTHandle->pUSART->CR1|= 1<<13;
	//DMA enable
	pUSARTHandle->pUSART->CR3|=1<<7;
	
}

 uint8_t USART_SendData(USART_TypeDef *pUSART, uint8_t *pTxBuffer)
 {
	 pUSART->DR  = (*pTxBuffer  & (uint8_t)0xFF);
	 while(!(pUSART->SR & (0x1UL << 6U)));
	 return pUSART->DR;
 }
 
/*void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Length)
{
	uint16_t *pdata;

	for(uint32_t i = 0 ; i < Length; i++)
	{
			while(! (pUSARTHandle->pUSART->SR & (0x1UL << 7U) ));
			pUSARTHandle->pUSART->DR = (*pTxBuffer  & (uint8_t)0xFF);
			pTxBuffer++;
		
	}

	while( ! (pUSARTHandle->pUSART->SR & 0x06));
}
 
 uint8_t USART_ReceiveData(USART_TypeDef *pUSART, uint8_t *pRxBuffer)
 {
	 *pRxBuffer= pUSART->DR;
	while(!(pUSART->SR & 0x05));
	 return pUSART->DR;

 }
 */
 
void USART_SetBaudRate(USART_TypeDef *pUSARTx, uint32_t BaudRate)
{
		uint32_t PCLKx;
		uint32_t usartdiv;
		uint32_t M_part,F_part;		

		if(pUSARTx == USART1 || pUSARTx == USART6)
								{
									PCLKx = RCC_GetPCLK2Value();
								}
						else
								{
									PCLKx = RCC_GetPCLK2Value();
								}
	
		if(pUSARTx->CR1 & (1 << 15))
								{
									usartdiv = ((25 * PCLKx) / (2 * BaudRate));
								}
						else
								{
									usartdiv = ((25 * PCLKx) / (4 * BaudRate));
								}

	M_part = usartdiv/100;
	pUSARTx->BRR|= M_part << 4;
	F_part = (usartdiv - (M_part * 100));
	if(pUSARTx->CR1 & (1 << 15))
			{
				F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);
			}
	else
			{
				F_part = ((( F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
			}
		pUSARTx->BRR|= F_part;
}

uint32_t RCC_GetPCLK2Value(void)
{
							uint32_t  SystemClk = 0, temp, pclk2;

							uint8_t clksrc = ((RCC->CFGR >> 2) & 0x3);

							uint8_t ahbp,apb2p;

							if(clksrc == 0)
							{
								SystemClk = 16000000;
							}
							else if(clksrc == 1)
							{
								SystemClk = 8000000;
							}
							else if(clksrc == 2)
							{
								SystemClk = RCC_GetPLLOutputClock();
							}

							/* AHBP */
							temp = ((RCC->CFGR >> 4) & 0xF);

							if(temp < 8)
							{
								ahbp = 1;
							}
							else
							{
								ahbp = AHB_Prescaler[temp-8];
							}

							/* APB2 */
							temp = ((RCC->CFGR >> 13) & 0x7);

							if(temp < 4)
							{
								apb2p = 1;
							}
							else
							{
								apb2p = APBx_Prescaler[temp-4];
							}

							pclk2 = (SystemClk / ahbp) / apb2p;

							return pclk2;
}

uint32_t RCC_GetPLLOutputClock(void)
{
	/* Not used for now */
	return 0;
}


uint32_t RCC_GetPCLK1Value(void)
{
	/* Not used for now */
	return 0;
}




