#include <stm32f407xx.h>
#include <stdint.h>

#ifndef _STM32_CAN_H_
#define _STM32_CAN_H_


#define ENABLE 							1
#define DISABLE 						0
#define SET 								ENABLE
#define RESET 							DISABLE
#define GPIO_PIN_SET  			SET
#define GPIO_PIN_RESET  		RESET

typedef struct
{
	uint32_t	prescalar;
	uint32_t	mode;					//@mode
	uint32_t	sync_jmp_bit; //max no. (time quanta) allowed to shorten or lengthen a bit for syncronization
	uint32_t	time1;
	uint32_t	time2;
	uint8_t		auto_retransmit;
	
}CAN_Init;


typedef struct
{
	CAN_TypeDef				*pCANx;
	CAN_Init					CANx_init;
}CAN_Handle_t;

void CAN_init (CAN_Handle_t *pCANHandle);


//@mode

#define	CAN_MODE_NORMAL
#define	CAN_MODE_LOOPBCK
#define	CAN_MODE_SILENT
#define	CAN_MODE_SILENT_LOOPBCK


































































#endif 