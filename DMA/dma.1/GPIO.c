#include "GPIO.h"
#include <stm32f407xx.h>
#include <stdio.h>

//GPIO Peripheral Clock control API
/******************************************************************************
*
*@fn										-			GPIO_PClkCtrtl
*
*
*@brief									-			It enables or disables the pripheral clock
*@param[in]							-			GPIO base address
*@param[in]							-			ENABLE or DISABLE
*@param[in]							-
*
*
*
*@return	 							-			none
*
*@Note									-			none
*
********************************************************************************/
void GPIO_PClkCtrl( GPIO_TypeDef *pGPIOx, uint8_t ENorDA)
{
	if(ENorDA == ENABLE)
	{
			switch( (int)pGPIOx)
			{
				case (int)GPIOA:
					GPIOA_PCLK_EN;
					break;
				
				case (int)GPIOB:
					GPIOB_PCLK_EN;
					break;
				
				case (int)GPIOC:
					GPIOC_PCLK_EN;
					break;
				
				case (int)GPIOD:
					GPIOD_PCLK_EN;
					break;
				
				case (int)GPIOE:
					GPIOE_PCLK_EN;
					break;
				
				case (int)GPIOF:
					GPIOF_PCLK_EN;
					break;
				
				case (int)GPIOG:
					GPIOG_PCLK_EN;
					break;
				
				case (int)GPIOH:
					GPIOH_PCLK_EN;
					break;
				
				case (int)GPIOI:
					GPIOI_PCLK_EN;
					break;
				
			}
	}
	else
	{
			switch( (int)pGPIOx)
			{
				case (int)GPIOA:
					GPIOA_PCLK_DA;
					break;
				
				case (int)GPIOB:
					GPIOB_PCLK_DA;
					break;
				
				case (int)GPIOC:
					GPIOC_PCLK_DA;
					break;
				
				case (int)GPIOD:
					GPIOD_PCLK_DA;
					break;
				
				case (int)GPIOE:
					GPIOE_PCLK_DA;
					break;
				
				case (int)GPIOF:
					GPIOF_PCLK_DA;
					break;
				
				case (int)GPIOG:
					GPIOG_PCLK_DA;
					break;
				
				case (int)GPIOH:
					GPIOH_PCLK_DA;
					break;
				
				case (int)GPIOI:
					GPIOI_PCLK_DA;
					break;
				
			}
	}
}	




//GPIO_initialization realted API
/******************************************************************************
*
*@fn										-		GPIO_Init
*
*
*@brief									-		Initializes GPIO pin
*@param[in]							-		GPIO Handler ( base address of GPIO port + Pin Configuration of GPIO port)
*@param[in]							-
*@param[in]							-
*
*
*
*@return	 							-		none
*
*@Note									-		none
*
********************************************************************************/
void GPIO_Init(GPIO_Handler_t *pGPIOHandler)
{
	uint32_t temp=0;
	//1.Configure the mode of GPIO pin
	GPIO_PClkCtrl( pGPIOHandler->pGPIOx, ENABLE);
				//choose port using SYSCNFG
	RCC->APB2ENR |= (1<<14);								//SYSCONFIG CLK ENABLE
			
	if(pGPIOHandler->GPIO_PinCofg.GPIO_PinNum<=3)
				{
					SYSCFG->EXTICR[0]|=(pGPIOHandler->GPIO_PinCofg.GPIO_IntPort<<4*pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);
				}else if(pGPIOHandler->GPIO_PinCofg.GPIO_PinNum<=7 && pGPIOHandler->GPIO_PinCofg.GPIO_PinNum>3)
				{
					SYSCFG->EXTICR[1]|=(pGPIOHandler->GPIO_PinCofg.GPIO_IntPort<<4*(pGPIOHandler->GPIO_PinCofg.GPIO_PinNum-4));
				}else if(pGPIOHandler->GPIO_PinCofg.GPIO_PinNum<=11 && pGPIOHandler->GPIO_PinCofg.GPIO_PinNum>7)
				{
					SYSCFG->EXTICR[2]|=(pGPIOHandler->GPIO_PinCofg.GPIO_IntPort<<4*(pGPIOHandler->GPIO_PinCofg.GPIO_PinNum-8));
				}else if(pGPIOHandler->GPIO_PinCofg.GPIO_PinNum<=15 && pGPIOHandler->GPIO_PinCofg.GPIO_PinNum>11)
				{
					SYSCFG->EXTICR[3]|=(pGPIOHandler->GPIO_PinCofg.GPIO_IntPort<<4*(pGPIOHandler->GPIO_PinCofg.GPIO_PinNum-12));
				}
			
				//Enable EXTI using IMR
					EXTI->IMR  |= (1 << pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);
				
			if (pGPIOHandler->GPIO_PinCofg.GPIO_PinMode <= GPIO_MODE_ANA)
		{
			temp = (pGPIOHandler->GPIO_PinCofg.GPIO_PinMode << (2 * pGPIOHandler->GPIO_PinCofg.GPIO_PinNum));
			pGPIOHandler->pGPIOx->MODER &= ~(0x3 << pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);	
			pGPIOHandler->pGPIOx->MODER |= temp;
			
		}else
		{
			switch((int)pGPIOHandler->GPIO_PinCofg.GPIO_PinMode)
			{
				case 4:
					EXTI->RTSR &=~(1 << pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);
					EXTI->FTSR |= (1 << pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);
					break;
				
				case 5:
					EXTI->FTSR &=~(1 << pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);
					EXTI->RTSR |= (1 << pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);
					break;
				
				case 6:
					EXTI->RTSR |= (1 << pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);
					EXTI->FTSR |= (1 << pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);
					break;			
			}
				
				
				
		}
	
	temp = 0;
		
		
	//2.Configure the speed of GPIO 
			temp = (pGPIOHandler->GPIO_PinCofg.GPIO_PinSpeed << (2 * pGPIOHandler->GPIO_PinCofg.GPIO_PinNum));
		pGPIOHandler->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);		
		pGPIOHandler->pGPIOx->OSPEEDR |= temp;
	temp = 0;
		
		
		//3.Configure the PUPD registers
				temp = (pGPIOHandler->GPIO_PinCofg.GPIO_PinPUPDctrl << (2 * pGPIOHandler->GPIO_PinCofg.GPIO_PinNum));
		pGPIOHandler->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);	
		pGPIOHandler->pGPIOx->PUPDR |= temp;
	temp = 0;
		
	//4.Configure the output type
				temp = (pGPIOHandler->GPIO_PinCofg.GPIO_PinOPtype <<  pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);
		pGPIOHandler->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandler->GPIO_PinCofg.GPIO_PinNum);	
		pGPIOHandler->pGPIOx->OTYPER |= temp;
	temp = 0;
			
	
	//5.configure the alternate personality
	if (pGPIOHandler->GPIO_PinCofg.GPIO_PinMode == GPIO_MODE_ALT)
	{
		uint32_t temp1, temp2;
		temp1= pGPIOHandler->GPIO_PinCofg.GPIO_PinNum /8;
		temp2= pGPIOHandler->GPIO_PinCofg.GPIO_PinNum % 8;
		pGPIOHandler->pGPIOx->AFR[temp1] &= ~(0xF << ( 4* temp2));	
		pGPIOHandler->pGPIOx->AFR[temp1] |= pGPIOHandler->GPIO_PinCofg.GPIO_PinAltFunMode << ( 4* temp2);
		
		
	}
	
	
}



/******************************************************************************
*
*@fn										-		GPIO_DeInt
*
*
*@brief									-		Deinitializes the GPIO port i.e. sends the value of the 
														register back to its RESET value
*@param[in]							-		Base address of GPIO port
*@param[in]							-
*@param[in]							-
*
*
*
*@return	 							-		none
*
*@Note									-		none
*
********************************************************************************/
void GPIO_DeInit(GPIO_TypeDef *pGPIOx)			//deinitializing register means sending the values of the register back to its RESET value
{
	switch((int)pGPIOx)
			{
				case (int)GPIOA:
					GPIOA_RESET;
					break;
				
				case (int)GPIOB:
					GPIOB_RESET;
					break;
				
				case (int)GPIOC:
					GPIOC_RESET;
					break;
				
				case (int)GPIOD:
					GPIOD_RESET;
					break;
				
				case (int)GPIOE:
					GPIOE_RESET;
					break;
				
				case (int)GPIOF:
					GPIOF_RESET;
					break;
				
				case (int)GPIOG:
					GPIOG_RESET;
					break;
				
				case (int)GPIOH:
					GPIOH_RESET;
					break;
				
				case (int)GPIOI:
					GPIOI_RESET;
					break;
			}
	
}

//Data Read-Write to/from GPIO

/******************************************************************************
*
*@fn										-		GPIO_RINpin
*
*
*@brief									-		Reads input of the input pin
*@param[in]							-		Base address of GPIO Port
*@param[in]							-		Pin number of the GPIO from which data is to be read
*@param[in]							-
*
*
*
*@return	 							-		high or low value read 
*
*@Note									-		none
*
********************************************************************************/
uint8_t GPIO_ReadPin(GPIO_TypeDef *pGPIOx, uint8_t GPIO_PinNum )
{
	uint8_t value;
	value = (uint8_t) ( (pGPIOx->IDR >> GPIO_PinNum )& 0x00000001);
	return value;
}


/******************************************************************************
*
*@fn										-		GPIO_RINport
*
*
*@brief									-		Reads value of all 16 pins of a GPIO input configured Port
*@param[in]							-		Base Address
*@param[in]							-
*@param[in]							-
*
*
*
*@return	 							-		Values from all 16 pins (HIGH / LOW)
*
*@Note									-		none
*
********************************************************************************/
uint16_t GPIO_ReadPort(GPIO_TypeDef *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) ( pGPIOx->IDR );
	return value;

}

/******************************************************************************
*
*@fn										-		GPIO_WrOPpin
*
*
*@brief									-		Writes output (high or low) to an output configured pin of a port
*@param[in]							-		Base address of the port
*@param[in]							-		Pin number to which the output is to be given
*@param[in]							-		high or low whatever the output is to be given
*
*
*
*@return	 							-		none
*
*@Note									-		none
*
********************************************************************************/
void GPIO_WritePin(GPIO_TypeDef *pGPIOx, uint8_t GPIO_PinNum, uint8_t value)
{
	if(value==GPIO_PIN_SET)
	{pGPIOx->ODR |= ( 1 << GPIO_PinNum );
	}
	else
	{pGPIOx->ODR &= ~( 1 << GPIO_PinNum );
	}


}

/******************************************************************************
*
*@fn										-		GPIO_WrOPport
*
*
*@brief									-		Writes output (high or low) to  all 16 output configured pins of
														a port
*@param[in]							-		base address of the port
*@param[in]							-		high /low whichever output is to be given
*@param[in]							-		
*
*
*
*@return	 							-		none
*
*@Note									-		none
*
********************************************************************************/
void GPIO_WritePort(GPIO_TypeDef *pGPIOx, uint16_t value)
{

pGPIOx->ODR |= value;

}

/******************************************************************************
*
*@fn										-		GPIO_ToggleOPpin
*
*
*@brief									-		Toggles output of a selected pin
*@param[in]							-		port address
*@param[in]							-		Pin number of the port 
*@param[in]							-
*
*
*
*@return	 							-		none
*
*@Note									-		none
*
********************************************************************************/
void GPIO_TogglePin(GPIO_TypeDef *pGPIOx, uint8_t GPIO_PinNum)
{

pGPIOx->ODR ^= (1 << GPIO_PinNum);

}

//IRQ configuration and Handling

/******************************************************************************
*
*@fn										-		GPIO_IRQConfg
*
*
*@brief									-		Configures interrupt on a pin
*@param[in]							-		IRQ number 
*@param[in]							-		IRQ priority
*@param[in]							-		enable or disable
*
*
*
*@return	 							-		none
*
*@Note									-		none
*
********************************************************************************/
void GPIO_IRQInterruptConfg(uint8_t IRQnumber, uint8_t ENorDA)
{	
		if(ENorDA==ENABLE)
		{
			if( IRQnumber<=31)
				*NVIC_ISER0 |= (1 << IRQnumber );
			else if( IRQnumber<=63 && IRQnumber>31)
				*NVIC_ISER1 |= (1 << (IRQnumber-32));
			else if( IRQnumber<=95 && IRQnumber>63)
				*NVIC_ISER2 |= (1 << (IRQnumber-64));	
		}
		
		else
		{
			if( IRQnumber<=31)
				*NVIC_ICER0 |= (1 << IRQnumber );
			else if( IRQnumber<=63 && IRQnumber>31)
				*NVIC_ICER1 |= (1 << (IRQnumber-32));
			else if( IRQnumber<=95 && IRQnumber>63)
				*NVIC_ICER2 |= (1 << (IRQnumber-64));	
		}
}


void GPIO_IRQPriorityConfg(uint8_t IRQPriority, uint8_t IRQnumber)
{
		uint8_t N=IRQnumber/4;
	  uint8_t U=IRQnumber%4;
  	uint8_t shift=8*U+(8-4);
	*(NVIC_IPR_BASE + (N*4)) |= ((IRQPriority)<<(shift));
	
}



/******************************************************************************
*
*@fn										-		GPIO_IRQHandler
*
*
*@brief									-		when an interrupt is set then handler executes
*@param[in]							-		pin number 
*@param[in]							-
*@param[in]							-
*
*
*
*@return	 							-		none
*
*@Note									-		none
*
********************************************************************************/
void GPIO_IRQHandler(uint8_t PinNum)
{
	if(EXTI->PR & (1<<PinNum))
	 {EXTI->PR|=(1<<PinNum);}
	
}	
	
	
	