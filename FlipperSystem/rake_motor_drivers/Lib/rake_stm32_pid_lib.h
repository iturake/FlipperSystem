#ifndef RAKE_STM32_PID_LIB_H
#define RAKE_STM32_PID_LIB_H

#ifdef _cplusplus
extern "C" {
#endif


/* Private includes ----------------------------------------------------------*/

#include "rake_stm32_timer_lib.h"
#include "rake_stm32_extra_lib.h"
#include "rake_stm32_encoder_lib.h" 
#include "rake_stm32_motor_lib.h"
#include "rake_stm32_uart_lib.h"

/* Private structures ----------------------------------------------------------*/

typedef struct pidStruct {
	float error;
	float lastError;
	float derivative;
	float integral;
	float integralPrev;
	float output;
	struct {float kp, kd, ki;} values;
} PID_HandleTypeDef;


/* Private function prototypes -----------------------------------------------*/

void RAKE_PID_Init(void);
void RAKE_Pid_Calculation(TIMER_HandleTypeDef *timer, ENCODER_HandleTypeDef *encoder, MOTOR_HandleTypeDef *motor, FLAG_HandleTypeDef *flag, RAKE_UART_HandleTypeDef *uart,  PID_HandleTypeDef *PID);

#ifdef _cplusplus
  }
#endif

#endif
