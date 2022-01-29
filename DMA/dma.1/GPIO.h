#include <stm32f407xx.h>
#include <core_cm4.h>

#define ENABLE 							1
#define DISABLE 						0
#define SET 								ENABLE
#define RESET 							DISABLE
#define GPIO_PIN_SET  			SET
#define GPIO_PIN_RESET  		RESET



typedef struct
{
			uint8_t GPIO_PinNum;												//@Pin_Number
			uint8_t GPIO_PinMode;												//@Modes
			uint8_t GPIO_PinSpeed;											//@Speed
			uint8_t GPIO_PinPUPDctrl;										//@Pull_Up_Down
			uint8_t GPIO_PinOPtype;											//@Types
			uint8_t GPIO_PinAltFunMode;									//
	  	uint8_t GPIO_IntPort;                       //@INTPORT
}GPIO_PinConfg_t;

typedef struct
{	
			GPIO_TypeDef *pGPIOx;
			GPIO_PinConfg_t GPIO_PinCofg;
}GPIO_Handler_t;


/*****************************************************************************************************
													APIS SUPPORTED BY THIS DRIVER
*****************************************************************************************************/

//GPIO Peripheral Clock control API
void GPIO_PClkCtrl( GPIO_TypeDef *pGPIOx, uint8_t ENorDA);


//GPIO_initialization realted API
void GPIO_Init(GPIO_Handler_t *pGPIOHandler);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);			//deinitializing register means sending the values of the register back to its RESET value


//Data Read-Write to/from GPIO
uint8_t GPIO_ReadPin(GPIO_TypeDef *pGPIOx, uint8_t GPIO_PinNum );
uint16_t GPIO_ReadPort(GPIO_TypeDef *pGPIOx);
void GPIO_WritePin(GPIO_TypeDef *pGPIOx, uint8_t GPIO_PinNum, uint8_t value);
void GPIO_WritePort(GPIO_TypeDef *pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIO_TypeDef *pGPIOx, uint8_t GPIO_PinNum);


//IRQ configuration and Handling
void GPIO_IRQInterruptConfg(uint8_t IRQnumber, uint8_t ENorDA);
void GPIO_IRQPriorityConfg(uint8_t IRQPriority, uint8_t IRQnumber);
void GPIO_IRQHandler(uint8_t GPIO_PinNum);




//CLOCK ENABLE

				//clock enable macros for GPIO peripheral

				#define GPIOA_PCLK_EN 			( RCC->AHB1ENR |= (1<<0) )
				#define GPIOB_PCLK_EN			  ( RCC->AHB1ENR |= (1<<1) )
				#define GPIOC_PCLK_EN			  ( RCC->AHB1ENR |= (1<<2) )
				#define GPIOD_PCLK_EN 			( RCC->AHB1ENR |= (1<<3) )
				#define GPIOE_PCLK_EN 			( RCC->AHB1ENR |= (1<<4) )
				#define GPIOF_PCLK_EN 			( RCC->AHB1ENR |= (1<<5) )
				#define GPIOG_PCLK_EN 			( RCC->AHB1ENR |= (1<<6) )
				#define GPIOH_PCLK_EN 			( RCC->AHB1ENR |= (1<<7) )
				#define GPIOI_PCLK_EN 			( RCC->AHB1ENR |= (1<<8) )

				//clock enable for APB1 bus peripherals

				//#define UART4_PCLK_EN 			( RCC->APB1ENR |= (1<<19) )
				//#define UART5_PCLK_EN 			( RCC->APB1ENR |= (1<<20) )
				//#define USART2_PCLK_EN 			( RCC->APB1ENR |= (1<<17) )
				//#define USART3_PCLK_EN 			( RCC->APB1ENR |= (1<<18) )

				#define SPI2_PCLK_EN 				( RCC->APB1ENR |= (1<<14) )
				#define SPI3_PCLK_EN 				( RCC->APB1ENR |= (1<<15) )

				#define I2C1_PCLK_EN 				( RCC->APB1ENR |= (1<<21) )
				#define I2C2_PCLK_EN 				( RCC->APB1ENR |= (1<<22) )
				#define I2C3_PCLK_EN 				( RCC->APB1ENR |= (1<<23) )

				//clock enable for APB2 bus peripherals

			//	#define USART1_PCLK_EN 			( RCC->APB2ENR |= (1<<4) )
			//	#define USART6_PCLK_EN 			( RCC->APB2ENR |= (1<<5) )
				#define SPI1_PCLK_EN 				( RCC->APB2ENR |= (1<<12) )
				//#define EXT1_PCLK_EN 			( RCC->APB2ENR |= (1<<17) )
				#define SYSCFG_PCLK_EN 			( RCC->APB2ENR |= (1<<14) )

// CLOCK DISABLE

				//clock disable macros for GPIO peripheral

				#define GPIOA_PCLK_DA 			( RCC->AHB1ENR &= ~(1<<0) )
				#define GPIOB_PCLK_DA 			( RCC->AHB1ENR &= ~(1<<1) )
				#define GPIOC_PCLK_DA 			( RCC->AHB1ENR &= ~(1<<2) )
				#define GPIOD_PCLK_DA 			( RCC->AHB1ENR &= ~(1<<3) )
				#define GPIOE_PCLK_DA 			( RCC->AHB1ENR &= ~(1<<4) )
				#define GPIOF_PCLK_DA 			( RCC->AHB1ENR &= ~(1<<5) )
				#define GPIOG_PCLK_DA 			( RCC->AHB1ENR &= ~(1<<6) )
				#define GPIOH_PCLK_DA 			( RCC->AHB1ENR &= ~(1<<7) )
				#define GPIOI_PCLK_DA 			( RCC->AHB1ENR &= ~(1<<8) )

				//clock disable for APB1 bus peripherals

				#define UART4_PCLK_DA 			( RCC->APB1ENR &= ~(1<<19) )
				#define UART5_PCLK_DA 			( RCC->APB1ENR &= ~(1<<20) )
				#define USART2_PCLK_DA 			( RCC->APB1ENR &= ~(1<<17) )
				#define USART3_PCLK_DA 			( RCC->APB1ENR &= ~(1<<18) )

				#define SPI2_PCLK_DA 				( RCC->APB1ENR &= ~(1<<14) )
				#define SPI3_PCLK_DA 				( RCC->APB1ENR &= ~(1<<15) )

				#define I2C1_PCLK_DA 				( RCC->APB1ENR &= ~(1<<21) )
				#define I2C2_PCLK_DA 				( RCC->APB1ENR &= ~(1<<22) )
				#define I2C3_PCLK_DA 				( RCC->APB1ENR &= ~(1<<23) )

				//clock disable for APB2 bus peripherals

				#define USART1_PCLK_DA 			( RCC->APB2ENR &= ~(1<<4) )
				#define USART6_PCLK_DA 			( RCC->APB2ENR &= ~(1<<5) )
				#define SPI1_PCLK_DA 				( RCC->APB2ENR &= ~(1<<12) )
				#define SYSCFG_PCLK_DA 			( RCC->APB2ENR &= ~(1<<14) )
	



/*****************************************************************************************************
													OUTPUT MODES 
*****************************************************************************************************/


//		Pin Number                                             (@Pin_Number)

#define GPIO_PIN_0							0
#define GPIO_PIN_1							1
#define GPIO_PIN_2							2
#define GPIO_PIN_3							3
#define GPIO_PIN_4							4
#define GPIO_PIN_5							5
#define GPIO_PIN_6							6
#define GPIO_PIN_7							7
#define GPIO_PIN_8							8
#define GPIO_PIN_9							9
#define GPIO_PIN_10							10
#define GPIO_PIN_11							11
#define GPIO_PIN_12							12
#define GPIO_PIN_13							13
#define GPIO_PIN_14							14
#define GPIO_PIN_15							15


//     GPIO Possible Modes																		(@Modes)

#define GPIO_MODE_IN 						0
#define GPIO_MODE_OUT						1
#define GPIO_MODE_ALT						2
#define GPIO_MODE_ANA 					3
#define GPIO_MODE_IT_FT  				4
#define GPIO_MODE_IT_RT					5
#define GPIO_MODE_IT_RFT				6


//   GPIO Output Types												  							(@Types)
//    Push-pull & Open-Drain

#define GPIO_OUT_PP 						0
#define GPIO_OUT_OD 						1


//     GPIO Output Speed																			(@Speed)

#define GPIO_SPEED_LOW 					0
#define GPIO_SPEED_MED 					1
#define GPIO_SPEED_HIG 					2
#define GPIO_SPEED_UHS 					3


//  GPIO Pull-up and Pull-down														(@Pull_Up_Down)   

#define GPIO_NPUPD   					  0
#define GPIO_PU									1
#define GPIO_PD									2

// GPIO Interrupt Macros																	(@INTPORT)
#define PA 0
#define PB 1
#define PC 2
#define PD 3
#define PE 4
#define PF 5
#define PG 6
#define PH 7




//RESET MACROS FOR GPIO

#define GPIOA_RESET       do{ RCC->AHB1RSTR|=(1<<0); RCC->AHB1RSTR&= ~(1<<0);}while(0)
#define GPIOB_RESET       do{ RCC->AHB1RSTR|=(1<<1); RCC->AHB1RSTR&= ~(1<<1);}while(0)
#define GPIOC_RESET       do{ RCC->AHB1RSTR|=(1<<2); RCC->AHB1RSTR&= ~(1<<2);}while(0)
#define GPIOD_RESET       do{ RCC->AHB1RSTR|=(1<<3); RCC->AHB1RSTR&= ~(1<<3);}while(0)
#define GPIOE_RESET       do{ RCC->AHB1RSTR|=(1<<4); RCC->AHB1RSTR&= ~(1<<4);}while(0)
#define GPIOF_RESET       do{ RCC->AHB1RSTR|=(1<<5); RCC->AHB1RSTR&= ~(1<<5);}while(0)
#define GPIOG_RESET       do{ RCC->AHB1RSTR|=(1<<6); RCC->AHB1RSTR&= ~(1<<6);}while(0)
#define GPIOH_RESET       do{ RCC->AHB1RSTR|=(1<<7); RCC->AHB1RSTR&= ~(1<<7);}while(0)
#define GPIOI_RESET       do{ RCC->AHB1RSTR|=(1<<8); RCC->AHB1RSTR&= ~(1<<8);}while(0)


//IRQ NUMBER definition fo EXTIx

#define IRQ_NO_EXTI0 				6
#define IRQ_NO_EXTI1 				7
#define IRQ_NO_EXTI2 				8
#define IRQ_NO_EXTI3 				9
#define IRQ_NO_EXTI4 				10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10 		40

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





