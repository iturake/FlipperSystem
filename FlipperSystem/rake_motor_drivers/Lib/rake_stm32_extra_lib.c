
/* Private includes ----------------------------------------------------------*/

#include "rake_stm32_extra_lib.h"

/* Private variables -----------------------------------------------*/

// Pin degerleri degistirilmeyecegi icin const olarak tanimlandi .
const uint32_t motorBackward_Pin = TIM_CHANNEL_2;
const uint32_t motorForward_Pin = TIM_CHANNEL_3;
const uint32_t encoderA_Pin = TIM_CHANNEL_1;
const uint32_t encoderB_Pin = TIM_CHANNEL_2;

/* Extern variables -----------------------------------------------*/

// main.c içindeki typedef bu dosyada kullanilmak üzere extern edildi .
extern FLAG_HandleTypeDef rflag1;

/* Private functions -----------------------------------------------*/

// Flag structtaki degiskenlere deger atama islemi burada yapildi .
void RAKE_FLAG_Init(void) {
	rflag1.LED.adminMode_bit = 0;
	rflag1.LED.CANBUS_bit = 0;
	rflag1.LED.ERROR_bit = 0;
	rflag1.LED.motorBackward_bit = 0;
	rflag1.LED.motorForward_bit = 0;
	rflag1.LED.normalMode_bit = 0;
	rflag1.LED.testMode_bit = 0;
	rflag1.LED.UART_bit = 0;
	
	rflag1.UART.rxComplete_bool = 0;
	rflag1.UART.rxIndex_bool = 0;
}

// Tip dönüsüpleri yapma islemi bu fonksiyon ile yapildi.
float RAKE_Convert(uint8_t convertMode_u8, float convertingValue_f32) {
	
	uint32_t convert_result ;
	
	switch(convertMode_u8){
		case VOLTAGE_TO_PWM:
			convert_result = PWM_ZERO + (convertingValue_f32 / VOLTAGE_MAX) * (PWM_MAX - PWM_ZERO);
			break;
		case PWM_TO_VOLTAGE:
			convert_result = (convertingValue_f32 / PWM_MAX) * VOLTAGE_MAX;
			break;
		case RPM_TO_PWM:
			convert_result = (convertingValue_f32 / RPM_MAX) * PWM_MAX;
			break;
		case PWM_TO_RPM:
			convert_result = (convertingValue_f32 / PWM_MAX) * RPM_MAX;
			break;
	}
	return convert_result;
}

// Ledlerin sürülme islemi bu fonksiyon ile yapildi .
void RAKE_Drive_Led(FLAG_HandleTypeDef *flag, TIMER_HandleTypeDef *timer) {
	_Bool shiftingData[8] = {flag->LED.motorForward_bit, flag->LED.motorBackward_bit, flag->LED.UART_bit, flag->LED.CANBUS_bit, flag->LED.adminMode_bit, flag->LED.testMode_bit, flag->LED.normalMode_bit, flag->LED.ERROR_bit};
	if(timer->ledDriver_u16 > LED_TIME) {
		HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
		
		//Shift and Write
		for(uint8_t bitCount = 0; bitCount < 8; bitCount++){
			HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, shiftingData[bitCount]);
			HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, GPIO_PIN_RESET);
		}
		
		HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
		timer->ledDriver_u16 = 0;
	}
}


void RAKE_Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}



