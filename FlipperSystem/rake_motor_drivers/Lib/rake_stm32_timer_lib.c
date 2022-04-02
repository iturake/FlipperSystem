
/* Private includes ----------------------------------------------------------*/

#include "rake_stm32_timer_lib.h"

/* Extern variables -----------------------------------------------*/

extern TIMER_HandleTypeDef rtimer1;

/* Private functions -----------------------------------------------*/

// Timer structtaki degiskenlere deger atama islemi burada yapildi .
void RAKE_TIMER_Init(void) {
	rtimer1.communicationCANBUS_u16 = 0;
	rtimer1.communicationUART_u16 = 0;
	rtimer1.ledDriver_u16 = 0;
	rtimer1.pidCalculator_u16 = 0;
	rtimer1.slowStartMotor_u16 = 0;
	rtimer1.velocityCalculator_u16 = 0;
}








