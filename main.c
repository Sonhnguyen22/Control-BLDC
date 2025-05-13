/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define A_En   1
#define B_En	 2
#define C_En   3
#define A_In   8
#define B_In   9
#define C_In   10



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void TIM2_IRQHandler(){
	TIM2->SR &= ~(1<<0);


}



/* USER CODE END 0 */
//uint32_t j;
/**
  * @brief  The application entry point.
  * @retval int
  */
//
#define PWM_PERIOD 7200
#define M_PI 3.141592653f

void Init_TIM(void){
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	//======= GPIO ==========
	GPIOA->CRH = 0x0000;
	GPIOB->CRH |= (0b1011<<((13-8)*4)) | (0b1011<<((14-8)*4)) | (0b1011<<((15-8)*4));
	
	GPIOA->CRH |= (1<<3*4);
	
	GPIOA->CRH |= (0b1011<<((A_In-8)*4)); // PA8  CH1
	GPIOA->CRH |= (0b1011<<((B_In-8)*4)); // PA9  CH1
	GPIOA->CRH |= (0b1011<<((C_In-8)*4)); // PA10 CH1
	TIM1->CCMR1 |= (0b110<<4) | TIM_CCMR1_OC1CE | TIM_CCMR1_OC1PE | (0b110<<12) | TIM_CCMR1_OC2CE | TIM_CCMR1_OC2PE;	// PWM mode 1, bit 3 Y/N ok
	TIM1->CCMR2 |= (0b110<<4) | TIM_CCMR2_OC3CE | TIM_CCMR2_OC3PE;
	TIM1->PSC = 0;
	TIM1->ARR = 7200;
	TIM1->CCR1 = 3600;
	TIM1->CCR3 = 3600;
	TIM1->CCR2 = 3600;
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE; // CH1 + CH1N
	TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE; // CH2 + CH2N
	TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NE; // CH3 + CH3N
	TIM1->BDTR |= TIM_BDTR_OSSI|TIM_BDTR_OSSR; 		// OSSi, OSSR
	TIM1->BDTR |= (4 << TIM_BDTR_DTG_Pos);
	
	TIM1->SMCR |= (14<<8);
	TIM1->EGR |= TIM_EGR_UG; 											// bit UG	
	TIM1->BDTR |= TIM_BDTR_MOE; 									// bit MOE
	TIM1->CR1 &= ~(3 << TIM_CR1_CMS_Pos);    			// Xóa bit CMS
	TIM1->CR1 |=  (2 << TIM_CR1_CMS_Pos);     		// Chon Center-aligned mode 3
	TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS |TIM_CR1_CEN;  // Reload ARR, EN counter tim1 
	TIM1->DIER |= (1<<0);
	NVIC->ISER[0] |= (1<<25);

//========= tim2 interrupt========
//	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
//	TIM2->PSC = 79;
//	TIM2->ARR = 999;
//	TIM2->CR1 |= TIM_CR1_CEN;
//	TIM2->DIER |= TIM_DIER_UDE;
//	NVIC->ISER[0] |= (1<<28);
	
//	RCC->APB1ENR |= (1<<0);
//	TIM2->PSC = 63;
//	TIM2->ARR = 249;
//	TIM2->CR1 |= (1<<0);
//	TIM2->DIER |= (1<<0);
//	NVIC->ISER[0] |= (1<<28);
}
//
uint16_t s_lookup[1024];
uint16_t modulation_index; // gia tri tu 0-65535
uint16_t angle_index;      // index tu 0–1023
uint16_t ccr1, ccr2, ccr3;
//uint8_t current_sector;    // gia tri tu 0–5

       // Bien tich luy banh rang lon (64-bit)
#define GEAR_TEETH (1 << 30)     // So rang banh lon = 2^30
#define LUT_SIZE 6144            // Tong diem LUT (6 sectors)
uint16_t check1;


void init_sin_table(void) {
	for (int i = 0; i < 1024; i++) {
		s_lookup[i] = (uint16_t)(57600 * sinf((float)i * M_PI / 3069.0f)); // M_PI / (6 × 1024)
	}
}
//
uint16_t Update_Angle(float target_freq) {
	static uint64_t gear_counter = 0;
	// 1. Tinh so rang can dich tren banh lon
	// Cong thuc: j = (target_freq * LUT_SIZE / PWM_FREQ) * GEAR_TEETH
	uint64_t j = (uint64_t)((target_freq * GEAR_TEETH * LUT_SIZE) / 5000.0f);
	// 2. Tich luy vao gear_couter (banh lon quay)
	gear_counter += j;
	// 3. Tinh so rang dich thuc te tren banh nho (k = j / GEAR_TEETH)
	uint64_t k = gear_counter >> 30;  // Dich 30 bit = chia cho 2^30
//	// 4. Giu lai phan du cho lan sau
//	gear_counter &= (GEAR_TEETH - 1); // Giu 30 bit thap
	// 5. Lay index trong LUT (0-6143)
	uint16_t lut_index = k % LUT_SIZE;
	
	return lut_index;
}

//
void SVM_Calc(uint16_t V_index, float frequency, uint16_t *ccr1, uint16_t *ccr2, uint16_t *ccr3) {

	// 1. Cap nhat goc quay dung Coaxial Gears
	uint16_t Angle = Update_Angle(frequency);
	// 2. Xac dinh sector va angle_index trong Angle
	uint8_t sector = Angle / 1024;       // 6 sectors (0-5)
	uint16_t angle_idx = Angle % 1024;  // 1024 diem/sector
	
	uint32_t m_i = V_index;
	uint32_t sin_phi = s_lookup[angle_idx];
	uint32_t sin_60_minus_phi = s_lookup[1023 - angle_idx];

	uint32_t Tccw = (m_i * sin_60_minus_phi) >> 19; // chia 524288
	uint32_t Tcw = (m_i * sin_phi) >> 19;
	uint32_t Tz = (PWM_PERIOD - Tcw - Tccw) >> 1;

	switch (sector) {
		case 0: *ccr1 = (Tccw + Tcw + Tz); 	*ccr2 = (Tcw + Tz); 				*ccr3 = (Tz); 							break;
		case 1: *ccr1 = (Tccw +  Tz); 			*ccr2 = (Tccw + Tcw + Tz); 	*ccr3 = (Tz); 							break;
		case 2: *ccr1 = (Tz); 							*ccr2 = (Tccw + Tcw + Tz); 	*ccr3 = (Tcw + Tz); 				break;
		case 3: *ccr1 = (Tz); 							*ccr2 = (Tccw + Tz); 				*ccr3 = (Tccw + Tcw + Tz); 	break;
		case 4: *ccr1 = (Tcw + Tz); 				*ccr2 = (Tz); 							*ccr3 = (Tccw + Tcw + Tz); 	break;
		case 5: *ccr1 = (Tccw + Tcw + Tz); 	*ccr2 = (Tz); 							*ccr3 = (Tccw + Tz); 				break;
	}
}

//


uint16_t V_F = 5000;
float tanso= 4.0f;

void TIM1_UP_IRQHandler(void)
{
  TIM1->SR &= ~TIM_SR_UIF;
	SVM_Calc(V_F, tanso, &ccr1, &ccr2, &ccr3); 
	TIM1->CCR1 = ccr1;
	TIM1->CCR2 = ccr2;
	TIM1->CCR3 = ccr3;

}

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
	Init_TIM();
	init_sin_table();

	while (1) {

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
