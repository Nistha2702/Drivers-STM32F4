#include "UART.h"
#include <stm32f407xx.h>
#include <stdint.h>
#include <stdio.h>

void USART_PClkCtrl( USART_TypeDef *pUSARTx,uint8_t ENorDA)
{
		if(ENorDA == ENABLE)
		{
			switch( (int)pUSARTx)
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
			switch( (int)pUSARTx)
			{
				case (int)USART1:
					USART1_PCLK_DA;
				break;
				
				case (int)USART2:
					USART2_PCLK_DA;
				break;
				
				case (int)USART3:
					USART3_PCLK_DA;
				break;
				
				case (int)UART4:
					UART4_PCLK_DA;
				break;
				
				case (int)UART5:
					UART5_PCLK_DA;
				break;
				
				case (int)USART6:
					USART6_PCLK_DA;
				break;				
			}						
		}		
}


void USART_Init(USART_Handle_t *pUSARTHandle)
{			
	
				 USART_PClkCtrl( pUSARTHandle->pUSARTx,ENABLE);

					uint32_t tempreg=0;
				/******************************** Configuration of CR1******************************************/

					if ( pUSARTHandle->USART_Confg.USART_Mode == USART_MODE_ONLY_RX)
						{
							tempreg |= ( 1 << USART_CR1_RE);
						}else if (pUSARTHandle->USART_Confg.USART_Mode == USART_MODE_ONLY_TX)
						{ 
							tempreg |= ( 1 << USART_CR1_TE );

						}else if (pUSARTHandle->USART_Confg.USART_Mode == USART_MODE_TX_RX)
						{
							tempreg |= ( 1 << USART_CR1_TE );
							tempreg |= ( 1 << USART_CR1_RE);
						}


					tempreg |= pUSARTHandle->USART_Confg.USART_DataLength << USART_CR1_M ;

					if ( pUSARTHandle->USART_Confg.USART_ParityBit == USART_PARITY_EN_EVEN)
						{
							tempreg |= ( 1 << USART_CR1_PCE);
						}else if (pUSARTHandle->USART_Confg.USART_ParityBit == USART_PARITY_EN_ODD )
						{ 
								tempreg |= ( 1 << USART_CR1_PCE);
								tempreg |= ( 1 << USART_CR1_PS);

						}

					pUSARTHandle->pUSARTx->CR1 = tempreg;

				/******************************** Configuration of CR2******************************************/

					tempreg=0;

					pUSARTHandle->pUSARTx->CR2 |= pUSARTHandle->USART_Confg.USART_NoOfStopBits << USART_CR2_STOP;

					

				/******************************** Configuration of CR3******************************************/

					if ( pUSARTHandle->USART_Confg.USART_HWflowCtrl == USART_HW_FLOW_CTRL_CTS)
					{
						
						pUSARTHandle->pUSARTx->CR3 |= ( 1 << USART_CR3_CTSE);
					}else if (pUSARTHandle->USART_Confg.USART_HWflowCtrl == USART_HW_FLOW_CTRL_RTS)
					{ 
						pUSARTHandle->pUSARTx->CR3 |= ( 1 << USART_CR3_RTSE);
					}else if (pUSARTHandle->USART_Confg.USART_HWflowCtrl == USART_HW_FLOW_CTRL_CTS_RTS)
					{
						pUSARTHandle->pUSARTx->CR3 |= ( 1 << USART_CR3_RTSE);
						pUSARTHandle->pUSARTx->CR3 |= ( 1 << USART_CR3_CTSE);
						
					}


				/******************************** Configuration of BRR(Baudrate register)******************************************/

					//Implement the code to configure the baud rate
					//We will cover this in the lecture. No action required here 
}

void USART_DeInit(USART_TypeDef *pUSARTx)
{
	
	switch((int)pUSARTx)
		{
			case (int)USART1:
			USART1_RESET;
			break;
			
			case (int)USART2:
			USART2_RESET;	
			break;
			
			case (int)USART3:
			USART3_RESET;	
			break;
			
			case (int)UART4:
			UART4_RESET;	
			break;
			
			case (int)UART5:
			UART5_RESET;	
			break;
			
			case (int)USART6:
			USART6_RESET;	
			break;			
		}
}


uint8_t USART_GetFlagStatus(USART_TypeDef *pUSARTx , uint32_t FlagName)
{
		if(pUSARTx->SR & FlagName)
				{
					return FLAG_SET;
				}
		return FLAG_RESET;
}


void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;
   
	for(uint32_t i = 0 ; i < Len; i++)
	{
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));
		if(pUSARTHandle->USART_Confg.USART_DataLength == USART_WORDLEN_9BITS)
		{

			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			if(pUSARTHandle->USART_Confg.USART_ParityBit == USART_PARITY_DISABLE)
			{
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				pTxBuffer++;
			}
		}
		else
		{
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}


void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	for(uint32_t i = 0 ; i < Len; i++)
	{
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));
			if(pUSARTHandle->USART_Confg.USART_DataLength == USART_WORDLEN_9BITS)
		{
			if(pUSARTHandle->USART_Confg.USART_ParityBit == USART_PARITY_DISABLE)
			{
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			  	pRxBuffer++;
			}
		}
		else
		{
			if(pUSARTHandle->USART_Confg.USART_ParityBit == USART_PARITY_DISABLE)
			{
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF) ;
			}
			else
			{
				 *pRxBuffer = (uint8_t) (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}
			pRxBuffer++;
		}
	}
}


uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pUSARTHandle->TxBsyState;
	if(state!=USART_STATE_TX_BUSY)
		{
			pUSARTHandle->TxLen = Len;
			pUSARTHandle->pTxBuffer = pTxBuffer;
			pUSARTHandle->pUSARTx->CR1 |= (1<< USART_CR1_TXEIE);
			pUSARTHandle->pUSARTx->CR1 |= (1<< USART_CR1_TCIE);
		}
	return state;	
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
		uint8_t state = pUSARTHandle->pRxBuffer;
	if(state!=USART_STATE_RX_BUSY)
		{
			pUSARTHandle->TxLen = Len;
			pUSARTHandle->pRxBuffer = pRxBuffer;
			pUSARTHandle->pUSARTx->CR1 |= (1<< USART_CR1_RXNEIE);
		}
	return state;	
}


void USART_IRQInterruptConfg(uint8_t IRQNumber, uint8_t ENorDA)
{
	if(ENorDA == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			/* Program ISER0 register */
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			/* Program ISER1 register (32 to 63) */
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			/* Program ISER2 register (64 to 95) */
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			/* Program ICER0 register */
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			/* Program ICER1 register (32 to 63) */
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			/* Program ICER2 register (64 to 95) */
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}


void USART_IRQPriorityConfg(uint32_t IRQPriority, uint8_t IRQnumber)
{		
		uint8_t N=IRQnumber/4;
	  uint8_t U=IRQnumber%4;
  	uint8_t shift=8*U+(8-4);
	*(NVIC_IPR_BASE  +  (N*4))|=((IRQPriority)<<(shift));
}


void USART_IRQHandler(USART_Handle_t *pUSARTHandle)
{
	
}


__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{
}


	