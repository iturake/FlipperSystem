#ifndef RAKE_STM32_TIMER_LIB_H
#define RAKE_STM32_TIMER_LIB_H

#ifdef _cplusplus
extern "C" {
#endif


/* Private includes ----------------------------------------------------------*/

#include "stm32f1xx_hal.h"
#include "rake_stm32_extra_lib.h"

/* Private structures ----------------------------------------------------------*/

typedef struct timerStruct {
	uint16_t velocityCalculator_u16;
  uint16_t slowStartMotor_u16;
	uint16_t communicationUART_u16;
	uint16_t communicationCANBUS_u16;
	uint16_t ledDriver_u16;
	uint16_t pidCalculator_u16;
} TIMER_HandleTypeDef;

/* Private function prototypes -----------------------------------------------*/

void RAKE_TIMER_Init(void);

#ifdef _cplusplus
  }
#endif

#endif
