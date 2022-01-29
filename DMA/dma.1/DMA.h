#include <stm32f407xx.h>
#ifndef _DMA_H
#define _DMA_H

#define ENABLE 							1
#define DISABLE 						0
#define SET 								ENABLE
#define RESET 							DISABLE
#define GPIO_PIN_SET  			SET
#define GPIO_PIN_RESET  		RESET


typedef struct
{
	uint8_t DMA_Direction;					//@Direction
	uint8_t DMA_M_Size;							//@M_Size
	uint8_t DMA_P_Size;							//@P_Size
	uint8_t DMA_Mode;								//@Mode
	uint8_t DMA_Length;							
	uint32_t DMA_Message;
	uint8_t DMA_Interrupt_config;   
}DMA_Confg_t;


typedef struct
{	
			DMA_TypeDef *pDMAx;
			DMA_Confg_t DMA_Cofg;
			DMA_Stream_TypeDef *pSTREAM;
}DMA_Handler_t;








void DMA1_strem6_interrupt_def(void);
void DMA1_PClkCtrl( DMA_TypeDef *pDMAx, uint8_t ENorDA);
void DMA1_init(DMA_Handler_t *pDMAHandle);

//@Direction
#define DMA_Direction_P2M		0
#define DMA_Direction_M2P		1
#define DMA_Direction_M2M		2


//@M_Size
#define DMA_M_SIZE_8				0
#define DMA_M_SIZE_16				1
#define DMA_M_SIZE_32				2


//@P_Size
#define DMA_P_SIZE_8				0
#define DMA_P_SIZE_16				0
#define DMA_P_SIZE_32				0


//@Mode
#define DMA_MODE_DIRECT			0
#define DMA_MODE_FIFO				1


#define DMA1_PCLK_EN  			(RCC->AHB1ENR |=(1<<21))
#define DMA2_PCLK_EN  			(RCC->AHB1ENR |=(1<<22))
#define DMA1_PCLK_DA  			(RCC->AHB1ENR &= ~(1<<21))
#define DMA2_PCLK_DA  			(RCC->AHB1ENR &= ~(1<<22))

#endif