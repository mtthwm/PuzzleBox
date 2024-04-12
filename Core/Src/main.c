/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
uint32_t firstTime = 0;
uint32_t secondTime = 0;
uint32_t knockCount = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void config_red () {
	GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER6_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk);
	GPIOC->ODR &= ~GPIO_ODR_6;
}
void toggle_red (char mode) {
	switch (mode) {
		case 2:
			GPIOC->ODR ^= GPIO_ODR_6;
			break;
		case 1:
			GPIOC->ODR |= GPIO_ODR_6;
			break;
		case 0:
			GPIOC->ODR &= ~GPIO_ODR_6;
			break;
	}
}
void config_blue () {
	GPIOC->MODER &= ~(GPIO_MODER_MODER7_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR7_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7_Msk);
	GPIOC->ODR &= ~GPIO_ODR_7;
}
void toggle_blue (char mode) {
	switch (mode) {
		case 2:
			GPIOC->ODR ^= GPIO_ODR_7;
			break;
		case 1:
			GPIOC->ODR |= GPIO_ODR_7;
			break;
		case 0:
			GPIOC->ODR &= ~GPIO_ODR_7;
			break;
	}
}
void config_orange () {
	GPIOC->MODER &= ~(GPIO_MODER_MODER8_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR8_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8_Msk);
	GPIOC->ODR &= ~GPIO_ODR_8;
}
void toggle_orange (char mode) {
	switch (mode) {
		case 2:
			GPIOC->ODR ^= GPIO_ODR_8;
			break;
		case 1:
			GPIOC->ODR |= GPIO_ODR_8;
			break;
		case 0:
			GPIOC->ODR &= ~GPIO_ODR_8;
			break;
	}
}
void config_green () {
	GPIOC->MODER &= ~(GPIO_MODER_MODER9_Msk);
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_9);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR9_Msk);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9_Msk);
	GPIOC->ODR &= ~GPIO_ODR_9;
}
void toggle_green (char mode) {
	switch (mode) {
		case 2:
			GPIOC->ODR ^= GPIO_ODR_9;
			break;
		case 1:
			GPIOC->ODR |= GPIO_ODR_9;
			break;
		case 0:
			GPIOC->ODR &= ~GPIO_ODR_9;
			break;
	}
}

enum PuzzleStateType {
	Puzzle1,
	Puzzle2,
	Puzzle3,
	GameEnd
};

/**
 * Non-blocking function to handle Puzzle 1 - knock detection.
 * Returns true if the puzzle is completed, false otherwise
 *
 * @return 1 if the puzzle is completed, 0 otherwise
 */
int doPuzzle1() {
	// Function GLOBALS
	const uint32_t INPUT_DELAY = 5000; // Delay between the start of puzzle, and accepting user input
	const uint32_t MIN_TIME_KNOCKSET_1 = 1000; // MIN time allowed that we will accept 2nd knock
	const uint32_t MAX_TIME_KNOCKSET_1 = 2000; // MAX time allowed that we will accept 2nd knock
	
	uint32_t elapsedTime = 0;
	
	// Function Non-blocking sentinels (track if something has occured or not)
	static uint32_t promptDelayDone = 0; // Non-blocking way to track if delay has elapsed
	static uint32_t promptTunePlayed = 0; // Track if the buzzer has played the puzzle tune

	
	// Puzzle tune plays ONCE, disabled afterwards.
	if (!promptTunePlayed){
		firstTime = HAL_GetTick();
		promptTunePlayed = 1;
		// TODO: PLAY BUZZER TUNE HERE
	}
	
	// Wait a certain amount of time before taking user-input (knocks)
	secondTime = HAL_GetTick();
	elapsedTime = secondTime - firstTime;
	if (elapsedTime < INPUT_DELAY)
		promptDelayDone = 1;
	if (!promptDelayDone)
		return 0;
	
	// PUZZLE PHASE - After buzzer plays and a short delay
	// Note that the cases denote puzzle progress, rather than
	// actual knocks received. See comments
	switch(knockCount){
		case 0:
			break;
		
		// FIRST KNOCK - Simply log the timestamp of first knock
		case 1:
			firstTime = HAL_GetTick();
			knockCount++; // See next case
			break;
		
		// WAIT PHASE -- wait for 2nd knock, or soft-reset puzzle if waiting too long
		case 2:
			secondTime = HAL_GetTick();
			elapsedTime = secondTime - firstTime;
			if (elapsedTime > MAX_TIME_KNOCKSET_1){
				knockCount = 0;
				firstTime = 0;
				secondTime = 0;
			}
			break;
		
		// SECOND KNOCK - Check if it was within the range threshold.
		// Do a soft-reset of the puzzle if it wasn't
		case 3:
			secondTime = HAL_GetTick();
		  elapsedTime = secondTime - firstTime;
		
			if (elapsedTime >= MIN_TIME_KNOCKSET_1 
				&& elapsedTime <= MAX_TIME_KNOCKSET_1)
				return 1; // Success! Puzzle complete!
			else{
				knockCount = 0;
				firstTime = 0;
				secondTime = 0;
			}
			
			// Error - received too many knocks before we could even check!
			default:
				knockCount = 0;
				firstTime = 0;
				secondTime = 0;
				promptDelayDone = 0;
				promptTunePlayed = 0;
		};
				
	return 0;
}

int doPuzzle2() {
	return 0;
}

int doPuzzle3() {
	return 0;
}

void doGameEnd() {
	
}

void playFanfare() {
	
}

void init() {
	
	// none of this works yet
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	//enable clock to timer 3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	// set PWM mode 1 on channel 2
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk); // clear
	TIM3->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos); // PWM mode 1
	
	//enable
	TIM3->CCER |= TIM_CCER_CC2E;
	
	TIM3->PSC = (short)99; // divide clock to 8000 khz
	TIM3->ARR = 100;
	
	// duty cycle
	TIM3->CCR2 = 20;
	
	//preload
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;
	
	// alternate function
	GPIOC->AFR[0] &= ~GPIO_AFRL_AFRL7_Msk;
	
}

void config_adc () {
	// Set PC0 to analog mode
	GPIOC->MODER |= 3 << GPIO_MODER_MODER0_Pos;
	// Enable the clock to the ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	// Set ADC1 to 8 bit resolution, continuous conversion mode, hardware triggers disabled.
	ADC1->CFGR1 |= 2 << ADC_CFGR1_RES_Pos;
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN_Msk;
	// Set ADC1 to use channel 10 (ADC_IN10 additional function)
	ADC1->CHSELR |= ADC_CHSELR_CHSEL10;
	// Start calibration
	ADC1->CR |= ADC_CR_ADCAL;
		
	// Wait until the calibration bit is reset.
	while (ADC1->CR & ADC_CR_ADCAL_Msk) {
		HAL_Delay(1);
	}
	
	// Enable the ADC
	ADC1->CR |= ADC_CR_ADEN;
	
	// Wait until the ADC is ready
	while (!(ADC1->ISR & ADC_ISR_ADRDY)) {
		HAL_Delay(1);
	}
		
	// Signal that we are ready for conversion
	ADC1->CR |= ADC_CR_ADSTART;
}

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
	
	config_red();
	config_adc();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

	
	
	
	enum PuzzleStateType mainState = Puzzle1;
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
		////////////////////////
		// TODO: Indicate different puzzle stages with board LED's?
		// E.g: puzzle1 = RED ON
		//      puzzle2 = GREEN ON
		////////////////////////
		switch (mainState) {
			case Puzzle1:
				if (doPuzzle1()) {
					playFanfare();
					mainState = Puzzle2;
				}
				break;
			
			case Puzzle2:
				if (doPuzzle2()) {
					playFanfare();
					mainState = Puzzle3;
				}
				break;
				
			case Puzzle3:
				if (doPuzzle3()) {
					playFanfare();
					mainState = GameEnd;
				}
				break;
			
			case GameEnd:
				doGameEnd();
		}
				
		HAL_Delay(20);

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
