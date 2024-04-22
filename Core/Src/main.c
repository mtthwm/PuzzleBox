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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


////////////////// UART ////////////////////////////////////////////////////////////////////////
#define RX_BUFF_SIZE 2

static char rx_chars[RX_BUFF_SIZE];
static int rx_i = 0;

/**
	* @brief Configures USART with PC4=RX, PC5=TX
	*/
void config_usart (uint32_t baudrate) {
	// Set the mode of the GPIO pins to use an alternate function
	GPIOC->MODER &= ~(GPIO_MODER_MODER10_Msk);
	GPIOC->MODER &= ~(GPIO_MODER_MODER11_Msk);
	GPIOC->MODER |= (2 << GPIO_MODER_MODER10_Pos);
	GPIOC->MODER |= (2 << GPIO_MODER_MODER11_Pos);

	
	// Set GPIO Pins PC10 and PC11 to use alternate function AF1: USART3
	GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;
	GPIOC->AFR[1] |= (GPIO_AF1_USART3 << GPIO_AFRH_AFSEL10_Pos); // TX
	GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL11_Msk;
	GPIOC->AFR[1] |= (GPIO_AF1_USART3 << GPIO_AFRH_AFSEL11_Pos); // RX
	
	// Enable USART TX and RX
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	
	// Set the USART Baud Rate to 115200 bit/sec
	USART3->BRR = (HAL_RCC_GetHCLKFreq()/baudrate);
	
	// Enable RXNE Interrupt
	USART3->CR1 |= USART_CR1_RXNEIE;
	
	// Configure interrupt handler for RXNE
	NVIC_EnableIRQ(USART3_4_IRQn);
	
	// Configure interrupt priorities
	NVIC_SetPriority(USART3_4_IRQn, 1);
	NVIC_SetPriority(SysTick_IRQn, 0);
	
	// Enable USART. Config vars become readonly!
	USART3->CR1 |= USART_CR1_UE;
}
void usart_transmit_char (char c) {
	int wait = 1;
	while (wait) {
		if ((USART3->ISR & USART_ISR_TXE_Msk)) {
			wait = 0;
		}
	} // Wait until the register is empty for transmission

	USART3->TDR = c;
}
void usart_transmit_str (char* s) {
	int i = 0;
	do {
		usart_transmit_char(s[i]);
		i++;
	} while (s[i] != '\0');
	
	usart_transmit_char('\0');
}
void usart_transmit_int (uint16_t num) {
	char buff[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int8_t i = 0;
	while (num != 0 && i < 8) {
		buff[i] = (num % 10) + 48;
		num = num / 10;
		i++;
	}
	
	for (int8_t i = 7; i > -1; i--) {
		usart_transmit_char(buff[i]);
	}
	
	usart_transmit_char('\n');
}
void USART3_4_IRQHandler () {
	char received = USART3->RDR;
	if (rx_i >= RX_BUFF_SIZE || rx_i < 0) {
		return;
	}
	rx_chars[rx_i] = received;
	rx_i++;
}
////////////////////////////////////////////////////////////////////////////////////////

void playKnockPrompt();

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

#define KNOCK_THRESH_LO 16
#define KNOCK_THRESH_HI 18

static volatile uint16_t knockCount = 0;
static uint32_t lastKnockTransmissionTime = 0;

uint32_t firstTime = 0;
uint32_t secondTime = 0;

/**
 * Non-blocking function to handle Puzzle 1 - knock detection.
 * Returns true if the puzzle is completed, false otherwise
 *
 * @return 1 if the puzzle is completed, 0 otherwise
 */
int getKnockTiming() {
	// Function GLOBALS
	const uint32_t INPUT_DELAY = 500; // Delay between the start of puzzle, and accepting user input
	const uint32_t MIN_DELAY_BETWEEN = 400; // MIN time allowed that we will accept the first knock
	const uint32_t MAX_DELAY_BETWEEN = 1000; // MAX time allowed that we will accept 2nd knock
	const uint32_t DELAY_BEFORE_REPLAYING = 10000;
	
	uint32_t elapsedTime = 0;
	
	// Function Non-blocking sentinels (track if something has occured or not)
	static uint32_t promptDelayDone = 0; // Non-blocking way to track if delay has elapsed
	static uint32_t promptTunePlayed = 0; // Track if the buzzer has played the puzzle tune
	static uint32_t firstTimerTimed = 0;
	static uint32_t secondTimerTimed = 0;
		
	// Puzzle tune plays ONCE, disabled afterwards.
	if (!promptTunePlayed){
		firstTime = HAL_GetTick();
		promptTunePlayed = 1;
		playKnockPrompt();
	}
	
	elapsedTime = HAL_GetTick() - firstTime;
	
	if (!promptDelayDone) {
		// Wait a certain amount of time before taking user-input (knocks)
		if (elapsedTime >= INPUT_DELAY) {
			knockCount = 0;
			promptDelayDone = 1;
		} else {
			return 0;
		}
	}
	
	char fail = 0;
	
	// How long has it been since we last prompted the user?
	if (elapsedTime >= DELAY_BEFORE_REPLAYING) {
		fail = 1; // Timed out before we got two knocks. Re-prompt the user
	}
	
	if (secondTime != 0) { 
		// The second knock was received!
		uint32_t delayBetween = secondTime - firstTime;
		usart_transmit_int(delayBetween);
		if ((delayBetween < MIN_DELAY_BETWEEN) || (delayBetween > MAX_DELAY_BETWEEN)) {
			fail = 1; // Timing was off. Re-prompt the user
		} else {
			return 1; // Timing was good!
		}
	}
	
	if (fail)
	{
		knockCount = 0;
		firstTime = 0;
		secondTime = 0;
		promptDelayDone = 0;
		promptTunePlayed = 0;
		firstTimerTimed = 0;
		secondTimerTimed = 0;
		toggle_orange(0);
	}
	
	toggle_orange(1);
	
	// PUZZLE PHASE - After buzzer plays and a short delay
	// Note that the cases denote puzzle progress, rather than
	// actual knocks received. See comments
	switch(knockCount){
		case 0:
			break;
		
		// FIRST KNOCK - Simply log the timestamp of first knock
		case 1:
			if (!firstTimerTimed){
				firstTime = HAL_GetTick();
				firstTimerTimed = 1;
			}
			break;
		
		// WAIT PHASE -- wait for 2nd knock, or soft-reset puzzle if waiting too long
		case 2:		
			if (!secondTimerTimed) {
				secondTime = HAL_GetTick();
				secondTimerTimed = 1;
			}
			break;
	};
				
	return 0;
}

enum PuzzleStateType {
	Puzzle1,
	Puzzle2,
	Puzzle3,
	GameEnd
};

void handleKnocks () {
	static uint32_t debouncer = 0;
	debouncer <<= 1;
	
	if (KNOCK_THRESH_LO > ADC1->DR || ADC1->DR > KNOCK_THRESH_HI) {
		debouncer |= 1;
	}
	
	if (debouncer == 0x7FFFFFFF) {
		knockCount++;
	}
}

// returns true when puzzle is solved
int doPuzzle1() {
	handleKnocks();
	return getKnockTiming();
}

int doPuzzle2() {
	return 0;
}

int doPuzzle3() {
	return 0;
}

void doGameEnd() {
	
}


void pwmInit() {
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	
	GPIO_InitTypeDef initStrPWM = {GPIO_PIN_5,
	GPIO_MODE_AF_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL,
	GPIO_AF1_TIM3};
	
	HAL_GPIO_Init(GPIOB, &initStrPWM);
	
	TIM3 -> PSC = 2; // Set PSC to divide clock by 250 (equivalent to 1/32 of a ms)
	TIM3 -> ARR = 100; // 40 * 1/32 = 1.25 ms (aka 800 hz)
	
	TIM3 -> CCMR1 &= ~TIM_CCMR1_CC2S; // Clear CC2S aka set CC1S to 0b0000
	TIM3 -> CCMR1 |= TIM_CCMR1_OC2M; // Set OC2M to PWM Output 2 mode (0b111)
	TIM3 -> CCMR1 |= TIM_CCMR1_OC2PE; // Preload enable channel 2
	TIM3 -> CCER |= TIM_CCER_CC2E; // Enable capture/compare for channel 2
	
	TIM3 -> CCR2 = 2; // Set duty cycle to 20% (of ARR)
		
}

void playTone(uint16_t freq) {
	uint16_t arr = 8000000 / (3 * freq);
	TIM3->ARR = arr;
	TIM3->CCR2 = arr / 2;
}

void playTune(uint16_t *frequencies, uint16_t *durations, uint16_t length) {
	for (int i = 0; i < length; i++) {
		if (frequencies[i] == 0) {
			TIM3->CR1 &= ~TIM_CR1_CEN;  // stop the timer
		}
		else {
			TIM3->CR1 |= TIM_CR1_CEN;  // start the timer
			playTone(frequencies[i]);
		}
		
		HAL_Delay(durations[i]);
	}
	TIM3->CR1 &= ~TIM_CR1_CEN;  // stop the timer after the tune completes
}

void playKnockPrompt () {
	uint16_t frequencies[10] = {196, 0, 146, 0, 146, 0, 164, 0, 146};
	uint16_t durations[10] = {500, 10, 250, 10, 250, 10, 250, 260, 500};

	playTune(frequencies, durations, 10);
}

void playFanfare() {
	uint16_t frequencies[] = {261, 329, 392, 523};
	uint16_t durations[] = {200, 200, 200, 600};
		
	playTune(frequencies, durations, 4);
}

enum adcUtil_resolution {
	adcUtil_12bit,
	adcUtil_10bit,
	adcUtil_8bit,
	adcUtil_6bit
};

void adcUtil_setup (ADC_TypeDef* adcInstance, enum adcUtil_resolution res) {
	// Enable the clock to the ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	
	// Set ADC to appropriate bit resolution, continuous conversion mode, hardware triggers disabled.
	adcInstance->CFGR1 |= res << ADC_CFGR1_RES_Pos;
	adcInstance->CFGR1 |= ADC_CFGR1_CONT;
	adcInstance->CFGR1 &= ~ADC_CFGR1_ALIGN_Msk;
}

void adcUtil_calibrate (ADC_TypeDef* adcInstance) {
	// Start calibration
	adcInstance->CR |= ADC_CR_ADCAL;
			
	// Wait until the calibration bit is reset.
	while (adcInstance->CR & ADC_CR_ADCAL_Msk) {
		HAL_Delay(1);
	}
		
	// Enable the ADC
	adcInstance->CR |= ADC_CR_ADEN;
	
	// Wait until the ADC is ready
	while (!(adcInstance->ISR & ADC_ISR_ADRDY)) {
		HAL_Delay(1);
	}
		
	// Signal that we are ready for conversion
	adcInstance->CR |= ADC_CR_ADSTART;
}
void adcUtil_enableChannel (uint8_t channelNumber) {
	// Set ADC to use channel 10 (ADC_IN10 additional function)
	ADC1->CHSELR |= (1 << channelNumber);
}

void config_knock_adc () {
	adcUtil_setup(ADC1, adcUtil_6bit);
	
	// Set PB1 to analog mode
	GPIOC->MODER |= 3 << GPIO_MODER_MODER0_Pos;
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR1_Msk;

	adcUtil_enableChannel(10);
	
	adcUtil_calibrate(ADC1);
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
	

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	
	
	enum PuzzleStateType mainState = Puzzle1;
	
	// Enable the RCC clocks
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	pwmInit();
		
	config_red();
	config_blue();
	config_green();
	config_orange();
	
	config_knock_adc();
	config_usart(115200);

	usart_transmit_str("USART READY!\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		/*
		if (HAL_GetTick() - lastKnockTransmissionTime >= 500) {
			usart_transmit_int(knockCount);
			lastKnockTransmissionTime = HAL_GetTick();
		}
		*/
	
    /* USER CODE END WHILE */
		switch (mainState) {
			case Puzzle1:
				if (doPuzzle1()) {
					playFanfare();
					mainState = Puzzle2;
				}
				break;
			
			case Puzzle2:
				toggle_blue(1);
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
