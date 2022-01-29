#include <stm32f407xx.h>
#include <stdint.h>
#include <core_cm4.h>
#define __weak __attribute__((weak))
#define ENABLE 							1
#define DISABLE 						0
#define SET 								ENABLE
#define RESET 							DISABLE
#define GPIO_PIN_SET  			SET
#define GPIO_PIN_RESET  		RESET
#define FLAG_SET						SET
#define FLAG_RESET					RESET



/*
* Configuration Sturure for USARTx peripheral
*/
typedef struct
{
	uint8_t USART_Mode;																						//@USART_Modes
	uint32_t USART_BaudRate;																			//@USART_Baud_Rate
	uint8_t USART_NoOfStopBits;																		//@USART_Stop_Bits
	uint8_t USART_DataLength;																			//@USART_Data_Length
	uint8_t USART_ParityBit;																			//@USART_Parity_Ctrl
	uint8_t USART_HWflowCtrl;																			//@USART_Hardware_FlowCtrl
}USART_Confg_t;

/*
* Handler Structure for USARTx peripheral
*/

typedef struct
{
	USART_TypeDef 		*pUSARTx;
	USART_Confg_t			 USART_Confg;
	uint8_t						*pTxBuffer;  /* Store application Tx buffer address */
	uint8_t						*pRxBuffer;  /* Store application Rx buffer address */
	uint32_t 					 TxLen;      /* Store Tx length			    */
	uint32_t					 RxLen;      /* Store Rx length			    */
	uint32_t 					 TxBsyState;    /* Store Tx state			    */
	uint32_t					 RxBsyState;    /* Store Rx state			    */
}USART_Handle_t;

/*
* 
*Possible states during application
*/
#define USART_STATE_READY			0
#define USART_STATE_TX_BUSY		1
#define USART_STATE_RX_BUSY		2

/*
* @USART_Modes
*Possible Options for modes in USART
*/

#define USART_MODE_ONLY_TX		0
#define USART_MODE_ONLY_RX		1
#define USART_MODE_TX_RX			2

/*
*NVIC base address definitions
*/

#define NVIC_ISER0					(volatile uint32_t*)0xE000E100U
#define NVIC_ISER1					(volatile uint32_t*)0xE000E104U		  
#define NVIC_ISER2					(volatile uint32_t*)0xE000E108U
#define NVIC_ISER3					(volatile uint32_t*)0xE000E112U
	
#define NVIC_ICER0					(volatile uint32_t*)0xE000E180U
#define NVIC_ICER1					(volatile uint32_t*)0xE000E184U		  
#define NVIC_ICER2					(volatile uint32_t*)0xE000E188U
#define NVIC_ICER3					(volatile uint32_t*)0xE000E192U
	
#define NVIC_IPR_BASE	  		(volatile uint32_t*)0xE000E400U
	

/*
* @USART_Baud_Rate
*Possible Options for Baud_Rates in USART
*/

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

/*
* @USART_Stop_Bits
*Possible Options for Stop_Bits in USART
*/

#define USART_1_STOP_BIT			0
#define USART_0_5_STOP_BIT		1
#define USART_1_5_STOP_BIT		2
#define USART_2_STOP_BIT			3


/*
 *@USART_Data_Length
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS		0
#define USART_WORDLEN_9BITS		1

/*
 *@USART_Parity_Ctrl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_DISABLE  0
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1


/*
* @USART_Hardware_FlowCtrl
*Possible Options for modes in USART
*/
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
* Clock Enable Macros for USARTx peripheral
*/
		
				#define USART1_PCLK_EN 			( RCC->APB2ENR |= (1<<4) )
				#define USART6_PCLK_EN 			( RCC->APB2ENR |= (1<<5) )
				#define UART4_PCLK_EN 			( RCC->APB1ENR |= (1<<19) )
				#define UART5_PCLK_EN 			( RCC->APB1ENR |= (1<<20) )
				#define USART2_PCLK_EN 			( RCC->APB1ENR |= (1<<17) )
				#define USART3_PCLK_EN 			( RCC->APB1ENR |= (1<<18) )

/*
* Clock Disable Macros for USARTx peripheral
*/

				#define USART1_PCLK_DA 			( RCC->APB2ENR &= ~(1<<4) )
				#define USART6_PCLK_DA 			( RCC->APB2ENR &= ~(1<<5) )
				#define UART4_PCLK_DA 			( RCC->APB1ENR &= ~(1<<19) )
				#define UART5_PCLK_DA 			( RCC->APB1ENR &= ~(1<<20) )
				#define USART2_PCLK_DA 			( RCC->APB1ENR &= ~(1<<17) )
				#define USART3_PCLK_DA 			( RCC->APB1ENR &= ~(1<<18) )

/*
* Reset Macros for USARTx peripheral
*/


				#define USART1_RESET       do{ RCC->APB2RSTR|=(1<<4); RCC->APB2RSTR&=~(1<<4);}while(0) 
				#define USART6_RESET       do{ RCC->APB2RSTR|=(1<<5); RCC->APB2RSTR&=~(1<<5);}while(0)
				#define USART2_RESET       do{ RCC->APB1RSTR|=(1<<17); RCC->APB1RSTR&=~(1<<17);}while(0)
				#define USART3_RESET       do{ RCC->APB1RSTR|=(1<<18); RCC->APB1RSTR&=~(1<<18);}while(0)
				#define UART4_RESET  	     do{ RCC->APB1RSTR|=(1<<19); RCC->APB1RSTR&=~(1<<19);}while(0)
				#define UART5_RESET        do{ RCC->APB1RSTR|=(1<<20); RCC->APB1RSTR&=~(1<<20);}while(0)

/*
 * USART Flags
 */
#define USART_FLAG_TXE 		( 1 << USART_SR_TXE  )
#define USART_FLAG_RXNE 	( 1 << USART_SR_RXNE )
#define USART_FLAG_TC 		( 1 << USART_SR_TC   )
				
				


/*****************************************************************************************************
													APIS SUPPORTED BY THIS DRIVER
*****************************************************************************************************/

//USART Peripheral Clock control API
void USART_PClkCtrl( USART_TypeDef *pUSARTx,uint8_t ENorDA);


//USART initialization realted API
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_TypeDef *pUSARTx);			


//Data SEND-RECEIVE through USART 

void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);


//IRQ configuration and Handling
void USART_IRQInterruptConfg(uint8_t IRQNumber, uint8_t ENorDA);
void USART_IRQPriorityConfg(uint32_t IRQPriority, uint8_t IRQnumber);
void USART_IRQHandler(USART_Handle_t *pUSARTHandle);


/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_TypeDef *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_TypeDef *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_TypeDef *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);
