#include <stm32f407xx.h>
#include "I2C.h"
#include "RCC.h"

uint32_t tempreg;
void I2C_PClkCtrl( I2C_TypeDef *pI2Cx, uint8_t ENorDA)
{
	if(ENorDA == ENABLE)
	{
			switch( (int)pI2Cx)
			{
				case (int)I2C1:
					I2C1_PCLK_EN;
					break;
				
				case (int)I2C2:
					I2C2_PCLK_EN;
					break;
				
				case (int)I2C3:
					I2C3_PCLK_EN;
					break;
			}
		}
	else
	{
			switch( (int)pI2Cx)
			{
				case (int)I2C1:
					I2C1_PCLK_DA;
					break;
				
				case (int)I2C2:
					I2C2_PCLK_DA;
					break;
				
				case (int)I2C3:
					I2C3_PCLK_DA;
					break;
			}
		}
	}

	
void I2C_Init(I2C_Handler_t *pI2CHandler)
{
	
				I2C_PClkCtrl( pI2CHandler->pI2Cx, ENABLE);
				pI2CHandler->pI2Cx->CR1 |= (pI2CHandler->I2C_PinCofg.I2C_ACKCtrl <<10);
				
				tempreg= ((RCC_GetPCLK1Value() /1000000U) & 0x3F);
				pI2CHandler->pI2Cx->CR2 |= tempreg;
				
				pI2CHandler->pI2Cx->OAR1 |= (pI2CHandler->I2C_PinCofg.I2C_DeviceAddress << 1);
				pI2CHandler->pI2Cx->OAR1 |= 1<<14;
				
				//CCR
				
				if(pI2CHandler->I2C_PinCofg.I2C_SClkSpeed <= I2C_SCLK_SPEED_SM)
				{
					pI2CHandler->pI2Cx->CCR |= RCC_GetPCLK1Value() / (2* pI2CHandler->I2C_PinCofg.I2C_SClkSpeed);
					
				}
				else
				{
					pI2CHandler->pI2Cx->CCR |= (1 <<15);
					pI2CHandler->pI2Cx->CCR |= (pI2CHandler->I2C_PinCofg.I2C_FMDutyCyc <<14);
					
						if(pI2CHandler->I2C_PinCofg.I2C_FMDutyCyc == I2C_FM_DUTY_2)
						{
									pI2CHandler->pI2Cx->CCR |= RCC_GetPCLK1Value() / (3* pI2CHandler->I2C_PinCofg.I2C_SClkSpeed);
						}
						else
						{
									pI2CHandler->pI2Cx->CCR |= RCC_GetPCLK1Value() / (25* pI2CHandler->I2C_PinCofg.I2C_SClkSpeed);
						}
		
				}
	
				//TRISE CONFIG
				if(pI2CHandler->I2C_PinCofg.I2C_SClkSpeed <= I2C_SCLK_SPEED_SM)
				{
					tempreg = (RCC_GetPCLK1Value() /1000000U) +1 ;		//standerd mode
				}
				else
				{
					tempreg = ((RCC_GetPCLK1Value() * 300)/1000000000U)+1;		//fast mode
				}
				pI2CHandler->pI2Cx->TRISE = (tempreg & 0x3F);
	
}	
	
	
	
void I2C_MasterSendData (I2C_Handler_t *pI2CHandler, uint8_t *pTxBuffer, uint8_t SlaveAddress,uint8_t Len)
{
	//1.START condition
	pI2CHandler->pI2Cx->CR1 |= (1 << 8U);
	//2.confirm the start generation(SB flag)
	while(!(pI2CHandler->pI2Cx->SR1 & 0x1));
	//3.send the address of slave
	pI2CHandler->pI2Cx->DR |= (SlaveAddress << 1);
	//clear 0th bit
		pI2CHandler->pI2Cx->DR &= ~1;
	//4.confirm that address phase is complete (ADDR flag)
	while(!(pI2CHandler->pI2Cx->SR1 & (0x1UL << 1U)));
	//5.clear ADDR
	uint32_t read = pI2CHandler->pI2Cx->SR1;
	read = pI2CHandler->pI2Cx->SR2;
	//6. send data bit by bit
	while(Len > 0)
	{
			while(!(pI2CHandler->pI2Cx->SR1 & (0x1UL << 7U)));
			pI2CHandler->pI2Cx->DR = *pTxBuffer;
			pTxBuffer++;
			Len--;
	}	 
	//7.wait for TXE ans BTF to set
	while(!(pI2CHandler->pI2Cx->SR1 & (0x1UL << 2U)));
	//8.STOP condition
	pI2CHandler->pI2Cx->CR1 |= (1 << 9U);
}
	
	
	
void I2C_MasterReceiveData (I2C_Handler_t *pI2CHandler, uint8_t *pRxBuffer, uint8_t SlaveAddress,uint8_t Len)
{
	//1.START condition
	pI2CHandler->pI2Cx->CR1 |= (1 << 8U);
	//2.confirm the start generation(SB flag)
	while(!(pI2CHandler->pI2Cx->SR1 & 0x1));
	//3.send the address of slave
	pI2CHandler->pI2Cx->DR |= (SlaveAddress << 1);
	pI2CHandler->pI2Cx->DR |= 1;
	//4.confirm that address phase is complete (ADDR flag)
	while(!(pI2CHandler->pI2Cx->SR1 & (0x1UL << 1U)));
	//read 1 byte
	if (Len ==1)
	{
		//disable ACK
		pI2CHandler->pI2Cx->CR1 &= ~(1 << 10);
		//clear ADDR flag
		uint32_t read = pI2CHandler->pI2Cx->SR1;
		read = pI2CHandler->pI2Cx->SR2;
		//wait for RXNE
		while(!(pI2CHandler->pI2Cx->SR1 & (0x1UL << 6U)));
		//generate STOP
		pI2CHandler->pI2Cx->CR1 |= (1 << 9U);
		//read DR
		*pRxBuffer = pI2CHandler->pI2Cx->DR;
		return;
		
	}
	if (Len >1)
	{
		//clear ADDR flag
		uint32_t read = pI2CHandler->pI2Cx->SR1;
		read = pI2CHandler->pI2Cx->SR2;
		//read all  DATA
		for(uint32_t i=Len; i>0; i--)
		{
			//wait until RXNE set
			while(!(pI2CHandler->pI2Cx->SR1 & (0x1UL << 6U)));
			if (i==2)
			{
				//clear ACK
				pI2CHandler->pI2Cx->CR1 &= ~(1 << 10);
				//STOP			
				pI2CHandler->pI2Cx->CR1 |= (1 << 9U);
			}
		//read DR
		*pRxBuffer = pI2CHandler->pI2Cx->DR;
			pRxBuffer++;
			
		}
	//re-enable ACK
		if(pI2CHandler->I2C_PinCofg.I2C_ACKCtrl== I2C_ACK_ENABLE)
		{
			pI2CHandler->pI2Cx->CR1 |= (1 << 10);
		}
	}
	
}	
	
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
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


void I2C_IRQPriorityConfig(uint8_t IRQPriority, uint8_t IRQnumber)
{
		uint8_t N=IRQnumber/4;
	  uint8_t U=IRQnumber%4;
  	uint8_t shift=8*U+(8-4);
	*(NVIC_IPR_BASE  +  (N*4))|=((IRQPriority)<<(shift));
}


	
uint8_t I2C_MasterSendData_IT (I2C_Handler_t *pI2CHandler, uint8_t *pTxBuffer, uint8_t SlaveAddress,uint8_t Len,uint8_t Start)
{
	
	uint8_t busystate = pI2CHandler->TxRxState;

	if( (busystate != I2C_STATE_BSY_TX) && (busystate != I2C_STATE_BSY_RX))
	{
		pI2CHandler->pTxBuffer = pTxBuffer;
		pI2CHandler->TxLen = Len;
		pI2CHandler->TxRxState = I2C_STATE_BSY_TX;
		pI2CHandler->DevAddr = SlaveAddress;
		pI2CHandler->Start = Start;

		//START Condition
		pI2CHandler->pI2Cx->CR1 |= (1 << 8U);

		//enable ITBUFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << 10);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << 9);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << 8);

	}

	return busystate;

}





uint8_t I2C_MasterReceiveData_IT (I2C_Handler_t *pI2CHandler, uint8_t *pRxBuffer, uint8_t SlaveAddress,uint8_t Len, uint8_t Start)
{
		uint8_t busystate = pI2CHandler->TxRxState;

	if( (busystate != I2C_STATE_BSY_TX) && (busystate != I2C_STATE_BSY_RX))
	{
		pI2CHandler->pRxBuffer = pRxBuffer;
		pI2CHandler->RxLen = Len;
		pI2CHandler->TxRxState = I2C_STATE_BSY_RX;
		pI2CHandler->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception 
		pI2CHandler->DevAddr = SlaveAddress;
		pI2CHandler->Start = Start;
		//START Condition
		pI2CHandler->pI2Cx->CR1 |= (1 << 8U);

		//enable ITBUFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << 10);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << 9);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << 8);
		
	}

	return busystate;
}

void I2C_EV_IRQHandling(I2C_Handler_t *pI2CHandler)
{
	/* Interrupt handling for both master and slave mode of a device */
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandler->pI2Cx->CR2 & (1 << 9);
	temp2 = pI2CHandler->pI2Cx->CR2 & (1 << 10);

	temp3 = pI2CHandler->pI2Cx->SR1 & (1 << 0);


	/* Handling for interrupt generated by SB event */
	if(temp1 && temp3)
	{
		if(pI2CHandler->TxRxState == I2C_STATE_BSY_TX)
		{
				pI2CHandler->pI2Cx->DR |= (pI2CHandler->DevAddr << 1);
				pI2CHandler->pI2Cx->DR &= ~(1);

		}
		else if(pI2CHandler->TxRxState == I2C_STATE_BSY_RX)
		{
				pI2CHandler->pI2Cx->DR |= (pI2CHandler->DevAddr << 1);
				pI2CHandler->pI2Cx->DR |= 1;
		}
	}

	temp3 = pI2CHandler->pI2Cx->SR1 & (1 << 1);
	/* Handling for interrupt generated by ADDR event */
	if(temp1 && temp3)
	{
			uint32_t read = pI2CHandler->pI2Cx->SR1;
			read = pI2CHandler->pI2Cx->SR2;
	}

	temp3 = pI2CHandler->pI2Cx->SR1 & (1 << 2);
	/* Handling for interrupt generated by BTF(Byte Transfer Finished) event */
	if(temp1 && temp3)
	{
		/* BTF flag is set */
		if(pI2CHandler->TxRxState == I2C_STATE_BSY_TX)
		{
			/* TXE flag is also set */
			if(pI2CHandler->pI2Cx->SR1 & (1 << 7))
			{
				if(pI2CHandler->TxLen == 0)
				{
					/* Generate STOP condition */
					if(pI2CHandler->Start == I2C_DISABLE_SR)
					{
						pI2CHandler->pI2Cx->CR1 |= (1 << 9U);
					}

					/* Reset all member elements of the handle structure */
					I2C_CloseSendData(pI2CHandle);

					/* Notify application about transmission complete */
					I2C_ApplicationEventCallback(pI2CHandler, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandler->TxRxState == I2C_STATE_BSY_RX)
		{
			;
		}
	}

	temp3 = pI2CHandler->pI2Cx->SR1 & (1 << 4);
	/* Handling for interrupt generated by STOPF event (only in slave)*/
	if(temp1 && temp3)
	{
		/* Clear STOPF flag */
		pI2CHandler->pI2Cx->CR1 |= 0x0000;

		/* STOPF generated by master */
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandler->pI2Cx->SR1 & (1 << 7);
	/* Handling for interrupt generated by TXE event */
	if(temp1 && temp2 && temp3)
	{
		/* Checking device mode */
		if(pI2CHandler->pI2Cx->SR2 & (1 << 0))
		{
			if(pI2CHandler->TxRxState == I2C_STATE_BSY_TX)
			{
				if(pI2CHandler->TxLen >0)
				{
									pI2CHandler->pI2Cx->DR = *(pI2CHandler->pTxBuffer);
									pI2CHandler-> TxLen--;
									pI2CHandler->pTxBuffer++;
				}

			}
		}
		else
		{
			/* Checking if slave is really in Tx mode */
			if(pI2CHandler->pI2Cx->SR2 & (1 << 2))
			{
				I2C_ApplicationEventCallback(pI2CHandler,I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = pI2CHandler->pI2Cx->SR1 & (1 << 6);
	/* Handling for interrupt generated by RXNE event */
	if(temp1 && temp2 && temp3)
	{
		/* Checking device mode */
		if(pI2CHandler->pI2Cx->SR2 & (1 << 0))
		{
			/* RXNE flag is set */
			if(pI2CHandler->TxRxState == I2C_STATE_BSY_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandler);
			}
		}
		else
		{
			/* Checking if slave is really in Rx mode */
			if(!(pI2CHandler->pI2Cx->SR2 & (1 << 2)))
			{
				I2C_ApplicationEventCallback(pI2CHandler,I2C_EV_DATA_RCV);
			}
		}
	}
}

void I2C_ER_IRQHandling(I2C_Handler_t *pI2CHandler)
{
	
}

void I2C_CloseReceiveData(I2C_Handler_t *pI2CHandler)
{
	/* Disabling ITBUFEN Control Bit */
	pI2CHandler->pI2Cx->CR2 &= ~( 1 << 10);

	/* Disabling ITEVFEN Control Bit */
	pI2CHandler->pI2Cx->CR2 &= ~( 1 << 9);

	pI2CHandler->TxRxState = I2C_STATE_RDY;
	pI2CHandler->pRxBuffer = 0;
	pI2CHandler->RxLen = 0;
	pI2CHandler->RxSize = 0;

	if(pI2CHandler->I2C_PinCofg.I2C_ACKCtrl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandler->pI2Cx,ENABLE);
	}

}


void I2C_CloseSendData(I2C_Handler_t *pI2CHandler)
{
	/* Disabling ITBUFEN Control Bit */
	pI2CHandler->pI2Cx->CR2 &= ~( 1 << 10);

	/* Disabling ITEVFEN Control Bit */
	pI2CHandler->pI2Cx->CR2 &= ~( 1 << 9);


	pI2CHandler->TxRxState = I2C_STATE_RDY;
	pI2CHandler->pTxBuffer = 0;
	pI2CHandler->TxLen = 0;
}




	