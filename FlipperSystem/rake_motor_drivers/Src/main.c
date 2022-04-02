/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


//RAKE kütüphaneleri main.c içinde kullanilmak amaciyla main.c içine dahil edildi .
#include "rake_stm32_encoder_lib.h"
#include "rake_stm32_uart_lib.h"
#include "rake_stm32_pid_lib.h"
#include "rake_stm32_motor_lib.h"
#include "rake_stm32_timer_lib.h"
#include "rake_stm32_extra_lib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint16_t count=0;
uint32_t hiz = 0;
uint32_t encoder_counter = 0;
uint8_t isMoved = 0;

/*
	Pin tanimlamalari main.c ye extern edildi .
	Pinler sonradan degistirilmedigi icin const olarak tanimlandi .
*/
extern const uint32_t motorBackward_Pin;
extern const uint32_t motorForward_Pin;
extern const uint32_t encoderA_Pin;
extern const uint32_t encoderB_Pin;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

// Typedef strcut tipindeki degerlerden degisken olusturuldu .
ENCODER_HandleTypeDef rencoder1;
RAKE_UART_HandleTypeDef ruart1;
PID_HandleTypeDef rpid1;
MOTOR_HandleTypeDef rmotor1;
TIMER_HandleTypeDef rtimer1;
FLAG_HandleTypeDef rflag1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/*
	Prototip fonksiyon tanimlamasi yapildi .
	Fonksiyonlarin içerikleri USER CODE BEGIN 4 kisminde bulunmaktadir .
*/
static void RAKE_INIT_Functions(void);                              //Structlara deger atama islemleri RAKE_INIT fonksiyonlari ile yapildi. 
static void RAKE_START_It(RAKE_UART_HandleTypeDef *uart);           //Interrupt baslatma fonksiyonu tanimlandi.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); 				//Timer interruptlarinin dallandigi fonksiyon buraya eklendi.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);     				//UART interruptlarinin dallandigi fonksiyon buraya eklendi.
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);           //Timer input capture interrupt fonksyionu prototipi tanimlandi.

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	RAKE_INIT_Functions();       //Structlara deger atama yapan fonksiyon burada cagirildi .
	RAKE_START_It(&ruart1);      //Interruptlari baslatan fonksiyona burada cagirildi .
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 100); //encoder deneme için eklendi!
  	
		RAKE_Rx_Motor_Speed(&rmotor1, &rflag1, &ruart1);
		
			if(rmotor1.desired.RPM_f32 > 0){
				if(rmotor1.desired.direction){
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 40000);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
					isMoved = 1;
				}
				else if(!rmotor1.desired.direction){
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 40000);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
					isMoved = 1;
				}
			}

		
		//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, rmotor1.desired.RPM_f32*65);
		//RAKE_Measure_Speed(&rtimer1, &rflag1, &rencoder1);
		
		//RAKE_Pid_Calculation(&rtimer1, &rencoder1, &rmotor1, &rflag1, &ruart1, &rpid1);
		
		
		//if(rmotor1.desired.RPM_f32 < 1) {
		//	__HAL_TIM_SET_COMPARE(&htim3, motorBackward_Pin, 0);
		//	__HAL_TIM_SET_COMPARE(&htim3, motorForward_Pin, 0);
		//} 
		//else {
		//RAKE_Pid_Calculation(&rtimer1, &rencoder1, &rmotor1, &rflag1, &ruart1, &rpid1);
		//}
		
		//RAKE_Tx_Motor_Speed(&rtimer1, &rencoder1, &ruart1);
		//RAKE_Drive_Led();--------> !!!! Bu fonksiyon burada çagirildiginda .o dosyalarinda multiply defined hatasi veriyor .
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8750;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CLOCK_Pin|LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CLOCK_Pin LATCH_Pin */
  GPIO_InitStruct.Pin = CLOCK_Pin|LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DATA_Pin */
  GPIO_InitStruct.Pin = DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DATA_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*
	Interruptlari baslatan fonksiyon içerigi burada yazildi . 
	Baska bir dosyada kullanilmayacagi icin static olarak tanimlandi .
*/
static void RAKE_START_It(RAKE_UART_HandleTypeDef *uart) {
	
	//HAL_TIM_Base_Start_IT(&htim2);
	
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim3);
	
	HAL_UART_Receive_IT(&huart1, uart->rxData, 1);
}

/*
	Structlara deger atama islemi yapan fonksiyon icerigi burada yazildi .
	Baska bir dosyada kullanilmayacagi icin static olarak tanimlandi .
*/
static void RAKE_INIT_Functions(void) {
	RAKE_ENCODER_Init();
	RAKE_MOTOR_Init();
	RAKE_PID_Init();
	RAKE_UART_Init();
	RAKE_FLAG_Init();
	RAKE_TIMER_Init();
	RAKE_START_It(&ruart1);
}

/*
	Timer interruptlarinin dallandigi fonksiyon burada tanimlandi .
	Interrupt oldugunda degerlerimizin arttirilmasi saglandi .
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		if (htim == &htim3){
			count=count+1;
			if(count > 1120 * (rmotor1.desired.RPM_f32 / 360)){
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
				
			}
  }
		
}



/* 
	TIM3 de encoder verisi input capture sayesinde aliniyor.
	Interrupt oldugu için fonksiyon her pulse buraya dallaniyor.
*/
/*
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		encoder_counter++;
	}
}
*/

/*
	Uart intterruptlarinin dallandigi fonksiyon burada tanimlandi .
	Dogru index degerlerin gelmesi durumunda veri almasi saglandi .
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART1){
		rflag1.LED.UART_bit = 1;
		//----------------------------------------
		//Reset RX Buffer Code
		//----------------------------------------
		if(rflag1.UART.rxIndex_bool == 0) {
			for(uint8_t index; index < 30; index++) {
				ruart1.rxBuffer[index] = 0;
			}
		}
		
		if(ruart1.rxData[0] == 'S') {
			rflag1.UART.rxIndex_bool = 0;
		}
		//----------------------------------------
		//Write Data to Buffer Code
		//----------------------------------------
		if(ruart1.rxData[0] != 'F'){
			ruart1.rxBuffer[rflag1.UART.rxIndex_bool++] = ruart1.rxData[0];
		} else {
			rflag1.UART.rxIndex_bool = 0;
			rflag1.UART.rxComplete_bool = 1;
		}
		//----------------------------------------
		HAL_UART_Receive_IT(&huart1, ruart1.rxData, 1);
		//----------------------------------------
	} else {
		rflag1.LED.UART_bit = 0;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
