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
	GPIOC->MODER &= ~(GPIO_MODER_MODER4_Msk);
	GPIOC->MODER &= ~(GPIO_MODER_MODER5_Msk);
	GPIOC->MODER |= (2 << GPIO_MODER_MODER4_Pos);
	GPIOC->MODER |= (2 << GPIO_MODER_MODER5_Pos);

	
	// Set GPIO Pins PC4 and PC5 to use alternate function AF1: USART3
	GPIOC->AFR[1] &= ~GPIO_AFRL_AFSEL4_Msk;
	GPIOC->AFR[1] |= (GPIO_AF1_USART3 << GPIO_AFRL_AFSEL4_Pos); // TX
	GPIOC->AFR[1] &= ~GPIO_AFRL_AFSEL5_Msk;
	GPIOC->AFR[1] |= (GPIO_AF1_USART3 << GPIO_AFRL_AFSEL5_Pos); // RX
	
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
	char buff[8] = {0,0,0,0,0,0,0,0};
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

#define KNOCK_THRESHOLD 22

static volatile uint16_t knockCount = 0;
static uint32_t lastKnockTransmissionTime;

enum PuzzleStateType {
	Puzzle1,
	Puzzle2,
	Puzzle3,
	GameEnd
};

void handleKnocks () {
	static uint8_t debouncer = 0;
	debouncer <<= 1;
	
	if (ADC1->DR > KNOCK_THRESHOLD) {
		debouncer |= 1;
	}
	
	if (debouncer == 0x03) {
		knockCount++;
	}
	
	if (HAL_GetTick() - lastKnockTransmissionTime >= 1000) {
		usart_transmit_int(knockCount);
		lastKnockTransmissionTime = HAL_GetTick();
	}
}

// returns true when puzzle is solved
int doPuzzle1() {
	handleKnocks();
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


void config_knock_adc () {
	adcUtil_setup(ADC1, adcUtil_6bit);
	
	// Set PA0 to analog mode
	GPIOA->MODER |= 3 << GPIO_MODER_MODER0_Pos;

	// Set ADC to use channel 0 (ADC_IN0 additional function)
	ADC1->CHSELR |= ADC_CHSELR_CHSEL0;

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
    /* USER CODE END WHILE */		
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
				
		HAL_Delay(1);

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
