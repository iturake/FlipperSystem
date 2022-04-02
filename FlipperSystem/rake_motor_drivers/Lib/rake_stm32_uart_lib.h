#ifndef RAKE_STM32_UART_LIB_H
#define RAKE_STM32_UART_LIB_H

#ifdef _cplusplus
extern "C" {
#endif


/* Private includes ----------------------------------------------------------*/

#include "stm32f1xx_hal.h"
#include "rake_stm32_motor_lib.h"
#include "rake_stm32_extra_lib.h"
#include "rake_stm32_timer_lib.h"
#include "rake_stm32_encoder_lib.h"


/* Private structures ----------------------------------------------------------*/

typedef struct uartStruct {
	unsigned char rxData[8];
	unsigned char txBuffer[30];
	unsigned char rxBuffer[30];
	uint16_t txBufferLen;
}RAKE_UART_HandleTypeDef;

/* Private function prototypes -----------------------------------------------*/

void RAKE_UART_Init(void);
void RAKE_Rx_Motor_Speed(MOTOR_HandleTypeDef *motor, FLAG_HandleTypeDef *flag, RAKE_UART_HandleTypeDef *uart);
void RAKE_Tx_Motor_Speed(TIMER_HandleTypeDef *timer, ENCODER_HandleTypeDef *encoder, RAKE_UART_HandleTypeDef *uart);


#ifdef _cplusplus
  }
#endif

#endif
