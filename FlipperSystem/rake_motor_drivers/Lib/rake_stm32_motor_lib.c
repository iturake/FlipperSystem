
/* Private includes ----------------------------------------------------------*/

#include "rake_stm32_motor_lib.h"

/* Extern variables -----------------------------------------------*/

extern uint32_t motorBackward_Pin;
extern uint32_t motorForward_Pin;
extern uint32_t encoderA_Pin; 
extern uint32_t encoderB_Pin; 

extern TIM_HandleTypeDef htim2;
//extern RAKE_UART_HandleTypeDef ruart1;
extern MOTOR_HandleTypeDef rmotor1;


/* Private functions -----------------------------------------------*/

// Motor structtaki degiskenlere deger atama islemi burada yapildi .
void RAKE_MOTOR_Init(void) {
	rmotor1.pwmValue_u16 = 0;
	rmotor1.pwmLastValue_u16 = 0;
	rmotor1.lastDirection_bool = 1;
	rmotor1.desired.PWM_u16 = 0;
	rmotor1.desired.RPM_f32 = 0;
	rmotor1.desired.direction = 0;
}

/**
  * @brief  Bu motor sürme fonksiyonunda PID hesaplamasindan gelen ( @arg PID->output ) voltaj degeri PWM degerine dönüstürülerek uygun timer çikis pinine verilir.  
  * @note   PID voltaj degerini PWM degerine çevirme "RAKE_Convert" fonksiyonunda yapilir. 
  *         Bu fonksiyonda motorun döndürülmek istenen yönü o anki yöne ters ise Timer delay kullanilarak hiz sifirlanir. 
  *         Devaminda istenilen yönde, hesaplanan PWM degeri motora verilir.
  * @param  voltageValue: calculated PID output value 
  * @param  *encoder: Rake ENCODER handle struct
  * @param  *motor: Rake MOTOR handle struct
  * @retval none
  */

void RAKE_Drive_Motor( float voltageValue, ENCODER_HandleTypeDef *encoder, MOTOR_HandleTypeDef *motor ) {
	motor->pwmValue_u16 = (int)RAKE_Convert(VOLTAGE_TO_PWM, voltageValue);
	uint32_t motorPin;
	if(motor->desired.direction == 0) {
//		HAL_GPIO_WritePin(MOTOR_BACKWARD_GPIO_Port, MOTOR_BACKWARD_Pin, 1);
//		HAL_GPIO_WritePin(MOTOR_FORWARD_GPIO_Port, MOTOR_FORWARD_Pin, 0);
		if(motor->desired.direction != encoder->measuredDirection_bool) {
		for(int a = motor->pwmValue_u16 ; a > 0; a -= 3)  {
			__HAL_TIM_SET_COMPARE(&htim2, motorForward_Pin, a);
			HAL_Delay(1);
		}
	}
		motorPin = motorBackward_Pin;
	} else if (motor->desired.direction == 1) {
//		HAL_GPIO_WritePin(MOTOR_BACKWARD_GPIO_Port, MOTOR_BACKWARD_Pin, 0);
//		HAL_GPIO_WritePin(MOTOR_FORWARD_GPIO_Port, MOTOR_FORWARD_Pin, 1);
		if(motor->desired.direction != encoder->measuredDirection_bool) {
		for(int a = motor->pwmValue_u16 ; a > 0; a -= 3)  {
			__HAL_TIM_SET_COMPARE(&htim2, motorBackward_Pin, a);
			HAL_Delay(1);
		}
	}
		motorPin = motorForward_Pin;
	}
	
//	if(motor->desired.RPM_f32 <= 10){
//		
//		for(int a = motor->pwmValue_u16 ; a > 0; a -= 3)  {
//			__HAL_TIM_SET_COMPARE(&htim2, motorPin, a);
//			HAL_Delay(1);
//		}
//	}
//	else{
//		__HAL_TIM_SET_COMPARE(&htim2, motorPin, motor->pwmValue_u16);
//	}
	__HAL_TIM_SET_COMPARE(&htim2, motorPin, motor->pwmValue_u16);
	motor->pwmLastValue_u16 = motor->pwmValue_u16;
}
