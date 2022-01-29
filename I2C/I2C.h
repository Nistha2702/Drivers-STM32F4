#include <stm32f407xx.h>
#ifndef _I2C_H
#define _I2C_H
				
				
				
				
/*
*CLOCK ENABLE-DISABLE MACROS FOR I2C
*/
				
				
				#define I2C1_PCLK_EN 				( RCC->APB1ENR |= (1<<21) )
				#define I2C2_PCLK_EN 				( RCC->APB1ENR |= (1<<22) )
				#define I2C3_PCLK_EN 				( RCC->APB1ENR |= (1<<23) )
				
				#define I2C1_PCLK_DA 				( RCC->APB1ENR &= ~(1<<21) )
				#define I2C2_PCLK_DA 				( RCC->APB1ENR &= ~(1<<22) )
				#define I2C3_PCLK_DA 				( RCC->APB1ENR &= ~(1<<23) )
				
/*
*	GENERAL MACROS FOR I2C
*/
					
				
				
				#define ENABLE 							1
				#define DISABLE 						0
				#define SET 								ENABLE
				#define RESET 							DISABLE
				#define GPIO_PIN_SET  			SET
				#define GPIO_PIN_RESET  		RESET
				
typedef struct
{
			uint32_t							I2C_SClkSpeed;										//@SClkSpeed
			uint8_t								I2C_DeviceAddress;
			uint8_t								I2C_ACKCtrl;											//@ACKCtrl
			uint16_t							I2C_FMDutyCyc;										//@FMDutyCyc
}I2C_PinConfg_t;

typedef struct
{	
			I2C_TypeDef 		  		*pI2Cx;
			I2C_PinConfg_t				I2C_PinCofg;
			uint8_t								*pTxBuffer;
			uint8_t								*pRxBuffer;
			uint32_t							TxLen;
			uint32_t							RxLen;
			uint8_t								TxRxState;
			uint8_t								DevAddr;
			uint32_t							RxSize;
			uint8_t								Start;
	
}I2C_Handler_t;

/*
*  SClkSpeed
*/
#define I2C_SCLK_SPEED_SM		100000
#define I2C_SCLK_SPEED_FM4K	400000

/*
* ACKCtrl
*/
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
*FMDutyCyc
*/
#define I2C_FM_DUTY_2				0
#define I2C_FM_DUTY_16_9		1
 

/*
*TxRxState
*/
#define I2C_STATE_RDY				0
#define I2C_STATE_BSY_TX		1
#define I2C_STATE_BSY_RX		2


/*
*IRQ_Number
*/
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73


//NVIC base address definitions

#define NVIC_ISER0					(volatile uint32_t*)0xE000E100U
#define NVIC_ISER1					(volatile uint32_t*)0xE000E104U		  
#define NVIC_ISER2					(volatile uint32_t*)0xE000E108U
#define NVIC_ISER3					(volatile uint32_t*)0xE000E112U
	
#define NVIC_ICER0					(volatile uint32_t*)0xE000E180U
#define NVIC_ICER1					(volatile uint32_t*)0xE000E184U		  
#define NVIC_ICER2					(volatile uint32_t*)0xE000E188U
#define NVIC_ICER3					(volatile uint32_t*)0xE000E192U
	
#define NVIC_IPR_BASE	  		(volatile uint32_t*)0xE000E400U



/*****************************************************************************************************
													APIS SUPPORTED BY THIS DRIVER
*****************************************************************************************************/
//I2C Peripheral Clock control API
void I2C_PClkCtrl( I2C_TypeDef *pI2Cx, uint8_t ENorDA);
//GPIO_initialization realted API
void I2C_Init(I2C_Handler_t *pI2CHandler);
void I2C_MasterSendData (I2C_Handler_t *pI2CHandler, uint8_t *pTxBuffer, uint8_t SlaveAddress,uint8_t Len);
void I2C_MasterReceiveData (I2C_Handler_t *pI2CHandler, uint8_t *pRxBuffer, uint8_t SlaveAddress,uint8_t Len);
//IT
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQPriority, uint8_t IRQnumber);
uint8_t I2C_MasterSendData_IT (I2C_Handler_t *pI2CHandler, uint8_t *pTxBuffer, uint8_t SlaveAddress,uint8_t Len,uint8_t Start);
uint8_t I2C_MasterReceiveData_IT (I2C_Handler_t *pI2CHandler, uint8_t *pRxBuffer, uint8_t SlaveAddress,uint8_t Len, uint8_t Start);
void I2C_EV_IRQHandling(I2C_Handler_t *pI2CHandler);
void I2C_ER_IRQHandling(I2C_Handler_t *pI2CHandler);
void I2C_CloseSendData(I2C_Handler_t *pI2CHandler);
void I2C_CloseReceiveData(I2C_Handler_t *pI2CHandler);




//clk enable disable macros
				#define I2C1_PCLK_EN 				( RCC->APB1ENR |= (1<<21) )
				#define I2C2_PCLK_EN 				( RCC->APB1ENR |= (1<<22) )
				#define I2C3_PCLK_EN 				( RCC->APB1ENR |= (1<<23) )
				#define I2C1_PCLK_DA 				( RCC->APB1ENR &= ~(1<<21) )
				#define I2C2_PCLK_DA 				( RCC->APB1ENR &= ~(1<<22) )
				#define I2C3_PCLK_DA 				( RCC->APB1ENR &= ~(1<<23) )
				
				
				
#endif