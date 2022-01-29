

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include <stm32f407xx.h>
#define __weak __attribute__((weak))

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_TypeDef				*pSPIx;	     /* This holds the base address of SPIx(x:0,1,2) peripheral */
	SPI_Config_t			SPIConfig;
	uint8_t						*pTxBuffer;  /* Store application Tx buffer address */
	uint8_t						*pRxBuffer;  /* Store application Rx buffer address */
	uint32_t 					 TxLen;      /* Store Tx length			    */
	uint32_t					 RxLen;      /* Store Rx length			    */
	uint32_t 					 TxState;    /* Store Tx state			    */
	uint32_t					 RxState;    /* Store Rx state			    */
}SPI_Handle_t;


/*
 * Possible SPI Application states
 */
#define SPI_READY		0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD		1 /* SPI Full-duplex bus configuration 		*/
#define SPI_BUS_CONFIG_HD		2 /* SPI Half-duplex bus configuration 		*/
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY 	3 /* SPI Simplex bux receive only configuration */


/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2		0 /* Baud rate control: fPCLK/2   */
#define SPI_SCLK_SPEED_DIV4		1 /* Baud rate control: fPCLK/4   */
#define SPI_SCLK_SPEED_DIV8		2 /* Baud rate control: fPCLK/8   */
#define SPI_SCLK_SPEED_DIV16		3 /* Baud rate control: fPCLK/16  */
#define SPI_SCLK_SPEED_DIV32		4 /* Baud rate control: fPCLK/32  */
#define SPI_SCLK_SPEED_DIV64		5 /* Baud rate control: fPCLK/64  */
#define SPI_SCLK_SPEED_DIV128		6 /* Baud rate control: fPCLK/128 */
#define SPI_SCLK_SPEED_DIV256		7 /* Baud rate control: fPCLK/256 */


/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS		0 /* 8-bit DFF is selected for transmission/reception  */
#define SPI_DFF_16BITS		1 /* 16-bit DFF is selected for transmission/reception */


/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH		1 /* Clock polarity to 1 when IDLE */
#define SPI_CPOL_LOW		0 /* Clock polarity to 0 when IDLE */

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH		1 /* Second clock transition <=> First data capture edge */
#define SPI_CPHA_LOW		0 /* First clock transition <=> First data capture edge  */


/*
 * @SPI_SSM
 */
#define SPI_SSM_EN		1 /* Software slave management enabled  */
#define SPI_SSM_DI		0 /* Software slave management disabled */


/*
 * SPI related status flag definitions
 */
#define SPI_FLAG_TXE 	( 1 << SPI_SR_TXE)
#define SPI_FLAG_RXNE 	( 1 << SPI_SR_RXNE)
#define SPI_FLAG_BUSY 	( 1 << SPI_SR_BSY)


/******************************************************************************************************************
 * 					APIs supported by this driver											  *
 * 			For more information about the APIs check the function definitions							  *
 ******************************************************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t EnorDi);


/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_TypeDef *pSPIx);


/*
 * Data send and receive
 */
void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t Length);
void SPI_ReceiveData(SPI_TypeDef *pSPIx, uint8_t *pRxBuffer, uint32_t Length);


/*
 * Data send and receive in Interrupt mode
 */
uint8_t SPI_SendDataInterruptMode(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length);
uint8_t SPI_ReceiveDataInterruptMode(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length);


/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQPriority, uint8_t IRQnumber);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 * Other peripheral APIs
 */
uint8_t SPI_GetFlagStatus(SPI_TypeDef *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_TypeDef *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_TypeDef *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_TypeDef *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_TypeDef *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);

/*
*SPI clock enable 
*/
				#define SPI2_PCLK_EN 				( RCC->APB1ENR |= (1<<14) )
				#define SPI3_PCLK_EN 				( RCC->APB1ENR |= (1<<15) )
				#define SPI1_PCLK_EN 				( RCC->APB2ENR |= (1<<12) )

/*
*SPI clock disable 
*/
				#define SPI2_PCLK_DA 				( RCC->APB1ENR &= ~(1<<14) )
				#define SPI3_PCLK_DA 				( RCC->APB1ENR &= ~(1<<15) )
				#define SPI1_PCLK_DA 				( RCC->APB2ENR &= ~(1<<12) )

#define ENABLE 							1
#define DISABLE 						0
#define SET 								ENABLE
#define RESET 							DISABLE
#define GPIO_PIN_SET  			SET
#define GPIO_PIN_RESET  		RESET
#define FLAG_SET						SET
#define FLAG_RESET					RESET
#define NULL								0x00



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

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */