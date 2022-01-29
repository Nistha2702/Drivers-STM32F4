#ifndef INC_STM32F4X27X_USART_DRIVER_H_
#define INC_STM32F4X27X_USART_DRIVER_H_

#include <stm32f407xx.h>
#define __weak __attribute__((weak))
#include <stdio.h>

typedef struct
{
		uint8_t USART_mode;																																																	//@mode
		uint8_t USART_wordlength;																																														//@data_bytes
		uint8_t USART_stop_bits;																																														//@stop_bits
		uint32_t USART_baud_rate;																																														//@baud_rate
		uint8_t USART_hardware_flowctrl;																																										//@Hardware_flowControl
		uint8_t USART_parity_ctrl;																																													//@parity
		
}USART_Config_t;

typedef struct
{
	USART_TypeDef		*pUSART;
	USART_Config_t	USART_Config;

}USART_Handle_t;


//@mode
#define USART_MODE_TX_ONLY	0
#define	USART_MODE_RX_ONLY	1
#define USART_MODE_TX_RX		2

//@data_bytes
#define	USART_WORDLEN_8			0
#define	USART_WORDLEN_9			1

//@stop_bits
#define USART_STOP_BI1_1		0
#define USART_STOP_BI1_0_5	1
#define USART_STOP_BI1_2		2
#define USART_STOP_BI1_1_5	3

//@baud_rate
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 			15200
#define USART_STD_BAUD_230400 			230400
#define USART_STD_BAUD_460800 			460800
#define USART_STD_BAUD_921600 			921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

//@hardware_flowControl
#define USART_HW_FLOW_CTRL_DISABLE	0
#define USART_HW_FLOW_CTRL_ENABLE		1

//@parity
#define USART_PARITY_DISABLE				0
#define USART_PARITY_ENABLE					1

//clock enable macros
#define USART1_PCLK_EN	 RCC->APB2ENR |= (1 << 4)   
#define USART2_PCLK_EN	 RCC->APB1ENR |= (1 << 17) 
#define USART3_PCLK_EN	 RCC->APB1ENR |= (1 << 18) 
#define UART4_PCLK_EN		 RCC->APB1ENR |= (1 << 19) 
#define UART5_PCLK_EN		 RCC->APB1ENR |= (1 << 20) 
#define USART6_PCLK_EN	 RCC->APB2ENR |= (1 << 5) 

#define USART1_PCLK_DI	 RCC->APB2ENR &= ~(1 << 4) 
#define USART2_PCLK_DI	 RCC->APB1ENR &= ~(1 << 17) 
#define USART3_PCLK_DI	 RCC->APB1ENR &= ~(1 << 18) 
#define UART4_PCLK_DI		 RCC->APB1ENR &= ~(1 << 19)
#define UART5_PCLK_DI		 RCC->APB1ENR &= ~(1 << 20)
#define USART6_PCLK_DI	 RCC->APB2ENR &= ~(1 << 5) 


/******************************************************************************************************************
  					APIs supported by this driver											  						  
 ******************************************************************************************************************/
 void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnorDi);
 void USART_Init(USART_Handle_t *pUSARTHandle);
 uint8_t USART_SendData(USART_TypeDef *pUSART, uint8_t *pTxBuffer);
 uint8_t USART_ReceiveData(USART_TypeDef *pUSART, uint8_t *pTxBuffer);
 void USART_SetBaudRate(USART_TypeDef *pUSARTx, uint32_t BaudRate);
 uint32_t RCC_GetPCLKValue(void);
 uint32_t RCC_GetPLLOutputClock(void);
 uint32_t RCC_GetPCLK2Value(void);
//void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Length);

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
	
#define ENABLE 							1
#define DISABLE 						0
#define SET 								ENABLE
#define RESET 							DISABLE
#define GPIO_PIN_SET  			SET
#define GPIO_PIN_RESET  		RESET
#define FLAG_SET						SET
#define FLAG_RESET					RESET



#endif