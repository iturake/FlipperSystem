
/* Private includes ----------------------------------------------------------*/
#include "rake_stm32_encoder_lib.h"


/* Extern structures -----------------------------------------------*/

extern ENCODER_HandleTypeDef rencoder1;    // main.c içindeki typedef burada extern edildi.
extern uint32_t hiz;       
extern uint32_t encoder_counter;

/* Private functions -----------------------------------------------*/


//Encoder structtaki degiskenlere deger atama islemi burada yapildi .
void RAKE_ENCODER_Init(void) {
	rencoder1.counter_u32 = 0;
	rencoder1.measuredDirection_bool = 0;
	rencoder1.measuredSpeed_f32 = 0.0;
}


/**
  * @brief  Encoder üzerinden RPM türünde motor hizi ölçülür.  
  * @note   Bu fonksiyon okunanan RPM hiz degerini ( @arg encoder->measuredSpeed_f32 ) degiskenine atar. 
	*					ENCODER_PULS, VELOCITY_TIME, GEAR_RATIO degiskenleri Extra Library'de tanimlanmistir.
  * @param  *timer: Rake TIMER handle struct
  * @param  *flag: Rake FLAG handle struct
  * @param  *encoder: Rake ENCODER handle struct
  * @retval none
  */
void RAKE_Measure_Speed(TIMER_HandleTypeDef *timer, FLAG_HandleTypeDef *flag, ENCODER_HandleTypeDef *encoder){
	//ileri yönde(saat yönü) encoder okuma
	if(TIM3->CR1 == 1) {
		encoder->measuredDirection_bool = 0;
		encoder->counter_u32 = (uint32_t)(TIM3->CNT);
		if(timer->velocityCalculator_u16 > VELOCITY_TIME) {
			TIM3->CNT = 0;
			encoder->measuredSpeed_f32 = (((float)encoder_counter / ENCODER_PULS) * (SECOND / (VELOCITY_TIME + 1)) * MINUTE) / GEAR_RATIO;
			hiz = (((float)encoder_counter / ENCODER_PULS) * (SECOND / (VELOCITY_TIME + 1)) * MINUTE) / GEAR_RATIO; // debugda test için tanimlandi 
			encoder_counter = 0;
			timer->velocityCalculator_u16 = 0;
			flag->LED.motorForward_bit = 1;
			flag->LED.motorBackward_bit = 0;
		}
	
	} 
	//geri yönde (saatin tersi yön) encoder okuma
	else if(TIM3->CR1 == 17) {  //neden 17? !!!!!!!!!!!!!!!
		encoder->measuredDirection_bool = 1;
		encoder->counter_u32 = 65535 - (uint32_t)TIM3->CNT;
		if(timer->velocityCalculator_u16 > VELOCITY_TIME) {
			TIM3->CNT = 65535;
			encoder->measuredSpeed_f32 = (((float)encoder_counter / ENCODER_PULS) * (SECOND / (VELOCITY_TIME + 1)) * MINUTE) / GEAR_RATIO;
			hiz = (((float)encoder_counter / ENCODER_PULS) * (SECOND / (VELOCITY_TIME + 1)) * MINUTE) / GEAR_RATIO;
			encoder_counter = 0;
			timer->velocityCalculator_u16 = 0;
			flag->LED.motorForward_bit = 0;
			flag->LED.motorBackward_bit = 1;
		}
	}
}
