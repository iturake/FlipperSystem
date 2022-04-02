#ifndef RAKE_STM32_MOTOR_LIB_H
#define RAKE_STM32_MOTOR_LIB_H

#ifdef _cplusplus
extern "C" {
#endif


/* Private includes ----------------------------------------------------------*/

#include "stm32f1xx_hal.h"
#include "rake_stm32_encoder_lib.h"
#include "rake_stm32_timer_lib.h"
#include "rake_stm32_extra_lib.h"
#include "rake_stm32_motor_lib.h"


/* Private structures ----------------------------------------------------------*/

typedef struct motorStruct {
	uint16_t pwmValue_u16;
	uint16_t pwmLastValue_u16;
	_Bool lastDirection_bool;
	struct {
		uint16_t PWM_u16;
		float RPM_f32;
		_Bool direction;
	} desired;
}MOTOR_HandleTypeDef;


/* Private function prototypes -----------------------------------------------*/

void RAKE_MOTOR_Init(void);
void RAKE_Drive_Motor( float voltageValue, ENCODER_HandleTypeDef *encoder, MOTOR_HandleTypeDef *motor );


#ifdef _cplusplus
  }
#endif

#endif
