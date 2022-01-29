#include <stm32f407xx.h>
#include "CAN.h"


void CAN_init (CAN_Handle_t *pCANHandle)
{
	pCANHandle->pCANx->MCR |= 1;
	while (!(pCANHandle->pCANx->MSR & 1));
	


	pCANHandle->pCANx->MCR &= ~1;
	while (pCANHandle->pCANx->MSR & 1);
	
}