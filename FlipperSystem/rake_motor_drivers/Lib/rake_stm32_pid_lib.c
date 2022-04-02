
/* Private includes ----------------------------------------------------------*/

#include "rake_stm32_pid_lib.h"

/* Extern structures -----------------------------------------------*/

extern PID_HandleTypeDef rpid1;

/* Private functions -----------------------------------------------*/

// Pid structtaki degiskenlere deger atama islemi burada yapildi .
void RAKE_PID_Init(void) {
	rpid1.error = 0;
	rpid1.lastError = 0;
	rpid1.derivative = 0;
	rpid1.integral = 0;
	rpid1.integralPrev = 0;
	rpid1.output = 0;
	rpid1.values.kp = 0.004;    // default: 0.032
	rpid1.values.kd = 0;   //default: 0.0008
	rpid1.values.ki = 0.00002; //default: 0.00005
}
//deneme: kp=0,007  ki:0.00004
// deneme 18 subat: p=0,004  ki:0.00002 kd:0 
/**
  * @brief  Motor hizini kontrol etmek için PID parametreleri ile kontrol yöntemi kullanilir. 
  *         Error, Integral ve Derivative degerleri Encoder ve UART'tan gelen motor hiz verileri ile hesaplanmaktadir. 
  * @note   Kp, Ki, Kd Parametrelerine göre hesaplanan PID output degeri ( @arg PID->output ) degiskenine atanir. 
  *         PID çikis gerilimi max 3.3 Volt olacak sekilde sinirlandirilmistir.  
	*					"PID_TIME" degeri kontrolcünün örnekleme zamanini(sampling time) ifade eder. 
  *         "PID_TIME" ve "VOLTAGE_MAX" degiskenleri Extra Library'de tanimlanmistir.
  * @param  *timer: Rake TIMER handle struct
  * @param  *encoder: Rake ENCODER handle struct
  * @param  *motor: Rake MOTOR handle struct
  * @param  *flag: Rake FLAG handle struct
  * @param  *uart: Rake UART handle struct
  * @param  *PID: Rake PID handle struct
  * @retval none
  */

// Pid hesaplama fonksiyonu burada tanimlandi.
void RAKE_Pid_Calculation(TIMER_HandleTypeDef *timer, ENCODER_HandleTypeDef *encoder, MOTOR_HandleTypeDef *motor, FLAG_HandleTypeDef *flag, RAKE_UART_HandleTypeDef *uart, PID_HandleTypeDef *PID) {
	if(timer->pidCalculator_u16 > PID_TIME) {
		RAKE_Rx_Motor_Speed(motor, flag, uart);
		
		PID->error = motor->desired.RPM_f32 - encoder->measuredSpeed_f32;//burasi su an PWM-RPM formunda !!!(aslinda sistemin dogru çalismasi için uarttan rpm datasi basmak zorundayiz)
		PID->integral = PID->integralPrev + (PID_TIME * (PID->error + PID->lastError) / 2);
		
		PID->derivative = (PID->error - PID->lastError) / PID_TIME;
		PID->output = (PID->error * PID->values.kp) + (PID->derivative * PID->values.kd) + (PID->integral * PID->values.ki);
//		
//		if(motor->desired.RPM_f32 == 0){
//		  PID->output=0;
//		}
		
		if(PID->output > VOLTAGE_MAX) {
			PID->output = VOLTAGE_MAX;
			PID->integral = PID->integralPrev;
		} else if(PID->output < -VOLTAGE_MAX) {
			PID->output = -VOLTAGE_MAX;
			PID->integral = PID->integralPrev;
		}
		
		RAKE_Drive_Motor( PID->output, encoder, motor);//PID->output  motor->desired.PWM_u16
		
		PID->lastError = PID->error; //yeni error degeri "previous" olana atandi
		PID->integralPrev = PID->integral; //yeni integral degeri "previous" olana atandi
		timer->pidCalculator_u16 = 0; //PID counter sifirlandi
	}
}


