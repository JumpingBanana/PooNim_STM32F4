#ifndef COMMON_H_
#define COMMON_H_

#include "stm32f4xx.h"

/*--- Control by System Timer Module  ---*/
extern uint32_t volatile g_sysTicks_50ms;	// Control by TIM5 (stm32f4xx_it.c)
extern uint32_t volatile g_sysTicks_100ms;

#endif /* COMMON_H_ */
