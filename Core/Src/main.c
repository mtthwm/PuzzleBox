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

#include "servo.h"
#include "accel.h"
#include "adc.h"

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

// Contains values for 3 photoresistors and PIEZO
// [0] contains piezo
// [1] is the top photoresistor
// [2] is the bottom photoresistor
// [3] is the back photoresistor

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void victoryBoxLEDs();
void unlock ();

////////////////// UART ////////////////////////////////////////////////////////////////////////
#define RX_BUFF_SIZE 2

static char rx_chars[RX_BUFF_SIZE];
static int rx_i = 0;

/**
	* @brief Configures USART with PC10=RX, PC11=TX
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
	usart_transmit_char('\r');
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

//////////////////////////////
//// BOX SIDE-LED CONFIG /////
//////////////////////////////
void config_sideLEDs()
{
	
	GPIO_InitTypeDef initStr = {GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_15,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	
	HAL_GPIO_Init(GPIOA, &initStr); 
//	// Clear state of Moder reg for pins [8-12], [15] (00)
//	GPIOA->MODER &= ~((1 << 31) | (1 << 30));
//	GPIOA->MODER &= ~((1 << 24) | (1 << 22) | (1 << 20) | (1 << 18) | (1 << 16));

//	// Pins [8-12], [15] to General Purpose Output (01)
//	GPIOA->MODER |= (1 << 30);
//	GPIOA->MODER |= (1 << 24) | (1 << 22) | (1 << 20) | (1 << 18) | (1 << 16);


//	// Pins [8-12], [15] to Output push-pull (0)
//	GPIOA->OTYPER &= ~((1 << 15) | (1 << 12) | (1 << 11) | (1 << 10) | (1 << 9) | (1 << 8));

//	// Pins [8-12], [15] to Low Speed (x0)
//	GPIOA->OSPEEDR &= ~((1 << 30) | (1 << 24) | (1 << 22) | (1 << 20) | (1 << 18) | (1 << 16));

//	// Pins [8-12], [15] to "No Pull-Up, Pull-Down" (00)
//	GPIOA->PUPDR &= ~((1 << 31) | (1 << 30)); // pin 15
//	GPIOA->PUPDR &= ~((1 << 25) | (1 << 24)); // 12
//	GPIOA->PUPDR &= ~((1 << 23) | (1 << 22)); // 11
//	GPIOA->PUPDR &= ~((1 << 21) | (1 << 20)); // 10
//	GPIOA->PUPDR &= ~((1 << 19) |(1 << 18)); // 9
//	GPIOA->PUPDR &= ~((1 << 17) | (1 << 16)); // 8

//	 // Pins [8-12], [15] set to OFF (0)
//	GPIOA->ODR &= ~((1 << 15) | (1 << 12) | (1 << 11) | (1 << 10) | (1 << 9) | (1 << 8));
}

// Pin PA8
void toggle_LED_front(char mode){
	switch(mode){
		case 0:
			GPIOA->ODR &= ~(1 << 8);
			break;
		case 1:
			GPIOA->ODR |= (1 << 8);
			break;
		case 2:
			GPIOA->ODR ^= (1 << 8);
			break;
	}
}

// Pin PA9
void toggle_LED_back(char mode){
	switch(mode){
		case 0:
			GPIOA->ODR &= ~(1 << 9);
			break;
		case 1:
			GPIOA->ODR |= (1 << 9);
			break;
		case 2:
			GPIOA->ODR ^= (1 << 9);
			break;
	}
}

// Pin PA0
void toggle_LED_top(char mode){
	switch(mode){
		case 0:
			GPIOA->ODR &= ~(1 << 0);
			break;
		case 1:
			GPIOA->ODR |= (1 << 0);
			break;
		case 2:
			GPIOA->ODR ^= (1 << 0);
			break;
	}
}

// Pin PA1
void toggle_LED_bottom(char mode){
	switch(mode){
		case 0:
			GPIOA->ODR &= ~(1 << 1);
			break;
		case 1:
			GPIOA->ODR |= (1 << 1);
			break;
		case 2:
			GPIOA->ODR ^= (1 << 1);
			break;
	}
}

// Pin PA10
void toggle_LED_left(char mode){
	switch(mode){
		case 0:
			GPIOA->ODR &= ~(1 << 10);
			break;
		case 1:
			GPIOA->ODR |= (1 << 10);
			break;
		case 2:
			GPIOA->ODR ^= (1 << 10);
			break;
	}
}

// Pin PA15
void toggle_LED_right(char mode){
	switch(mode){
		case 0:
			GPIOA->ODR &= ~(1 << 15);
			break;
		case 1:
			GPIOA->ODR |= (1 << 15);
			break;
		case 2:
			GPIOA->ODR ^= (1 << 15);
			break;
	}
}

/**
 * Configures all LEDs based on input.
 * @param mode '0' = ALL OFF | '1' = ALL ON | '2' = ALL INVERT
 */
void toggle_LED_all(char mode){
	switch(mode){
		case 0:
			toggle_LED_top(0);
			toggle_LED_bottom(0);
			toggle_LED_front(0);
			toggle_LED_back(0);
			toggle_LED_left(0);
			toggle_LED_right(0);
			break;
		case 1:
			toggle_LED_top(1);
			toggle_LED_bottom(1);
			toggle_LED_front(1);
			toggle_LED_back(1);
			toggle_LED_left(1);
			toggle_LED_right(1);
			break;
		case 2:
			toggle_LED_top(2);
			toggle_LED_bottom(2);
			toggle_LED_front(2);
			toggle_LED_back(2);
			toggle_LED_left(2);
			toggle_LED_right(2);
			break;
	}
}

//////////////////////////////////////////////

//////////////////////////
//// BOARD LED CONFIG ////
//////////////////////////
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
/////////////////////////////////////////////////

#define KNOCK_THRESH_HI 3500

////////////////////////////////////
///// PHOTORESISTOR GET() functions
/////////////////////////////////////
uint16_t getResistorTop(){
	return dmaUtil_buffer[1];
}

uint16_t getResistorBottom(){
	return dmaUtil_buffer[2];
}

uint16_t getResistorBack(){
	return dmaUtil_buffer[3];
}
////////////////////////////////////////////

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
	static uint32_t firstTimerTimed = 0; // Track if the first knock time has been recorded
	static uint32_t secondTimerTimed = 0; // Track if the second knock time has been recorded
		
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
	static uint8_t debouncer = 0;
	debouncer <<= 1;
	
	if (dmaUtil_buffer[0] > KNOCK_THRESH_HI) {
		debouncer |= 1;
	}
	
	if (debouncer == 0x1) {
		knockCount++;
	}
}

// returns true when puzzle is solved
int doPuzzle1() {
	handleKnocks();
	return getKnockTiming();
}

/**
 * Uses the accelerometer orientation to check if the user
 * (1) flips the box upside-down
 * (2) places the box on its right side
 * (3) Re-orients the box upright (default starting orientation)
 *
 * @return 1 if puzzle is completed, 0 otherwise
 */
int doPuzzle2() {
	int orientation = accelReadAxis();
	static uint16_t puzzleStage = 0;
	switch(puzzleStage){
		case 0:
			puzzleStage = 1;
			toggle_LED_bottom(1);
			break;
		// Wait until box is upside down
		case 1:
			if (orientation == Y_POS){
				puzzleStage = 2;
				toggle_LED_bottom(0);
				toggle_LED_right(1);
			}
			break;
		// Wait until box is on its right side
		case 2:
			if (orientation == X_POS){
				puzzleStage = 3;
				toggle_LED_right(0);
				toggle_LED_top(1);
			}
			break;
		// Wait until box is upright
		case 3:
			if (orientation == Y_NEG){
				toggle_LED_top(0);
				return 1;
			}
			break;
		
		// Error, reset puzzle back to stage 0
		default:
			puzzleStage = 0;
			return 0;
	}
	return 0;
}

/**
 * Puzzle 3 - Photoresistor Checks
 * Puzzle will include lighting the photoresistors
 * Each photoresistor will be associated with an LED
 * A lit LED indicates the hole will need to be lit up
 *
 * This function will sequentially check if
 * (1) The top photoresistor is lit up
 * (2) The bottom photoresistor is lit up
 * (3) Both the top and front photoresistors are lit (small challenge)
 */
int doPuzzle3() {
	const uint16_t LIGHT_THRESHOLD = 3700;
	static uint16_t puzzleStage = 0;

	switch(puzzleStage){
		case 0:
			toggle_LED_top(1);
			puzzleStage = 1;
			break;

		// Light TOP resistor
		case 1:
			if (getResistorTop() > LIGHT_THRESHOLD){
				toggle_LED_top(0);
				toggle_LED_bottom(1);
				puzzleStage = 2;
			}
			break;
		
		// Light BOTTOM resistor
		case 2:
			if(getResistorBottom() > LIGHT_THRESHOLD){
				toggle_LED_bottom(0);
				toggle_LED_back(1);
				toggle_LED_top(1);
				puzzleStage = 3;
			}
			break;

		// Light TOP and BACK resistor
		case 3:
			if(getResistorTop() > LIGHT_THRESHOLD && getResistorBack() > LIGHT_THRESHOLD){
				toggle_LED_top(0);
				toggle_LED_back(0);
				return 1;
			}
			break;
	}
	return 0;
}

/**
 * Ends the game by flashing box LEDs, then unlocking the servo-lock
 * This function never exits. The system must reset.
 */
void doGameEnd() {
	victoryBoxLEDs();
	unlock(); // TODO: Is this the correct servo angle for "unlock"?
	while (1){
		toggle_blue(2);
		toggle_green(2);
		toggle_orange(2);
		toggle_red(2);
		HAL_Delay(250);
	}
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

void playFanfare(int num) {
	uint16_t chord1[] = {261, 329, 392, 523};
	uint16_t chord2[] = {294, 370, 440, 587};
	uint16_t chord3[] = {330, 415, 494, 831, 659, 831, 988};
	uint16_t durations[] = {200, 200, 200, 600};
	uint16_t durations2[] = {200, 200, 200, 200, 200, 200, 600};
	
	switch (num) {
		case 1:
			playTune(chord1, durations, 4);
			break;
		
		case 2:
			playTune(chord2, durations, 4);
			break;
		
		case 3:
			playTune(chord3, durations2, 7);
			break;
	}
	
}

/**
 * Function that flashes ALL box LEDs 5 times
 * Intended to indicate progress in the puzzle.
 * Function is somewhat blokcing, uses HAL_Delay
 */
void flashBoxLEDs(){
	toggle_LED_all(0);
	for(int i = 0; i < 6; i++){
		toggle_LED_all(2);
		HAL_Delay(50);
	}
}

/**
 * Used to indicate something major occuring (the puzzle box being solved)
 * Function is somewhat blocking, using HAL_Delay
 */
void victoryBoxLEDs(){
	toggle_LED_all(0);
	for (int i = 0; i < 6; i++){
		// All on sequentially
		toggle_LED_front(1);
		HAL_Delay(50);

		toggle_LED_left(1);
		HAL_Delay(50);

		toggle_LED_back(1);
		HAL_Delay(50);

		toggle_LED_right(1);
		HAL_Delay(50);

		// All off sequentially
		toggle_LED_front(0);
		HAL_Delay(50);

		toggle_LED_left(0);
		HAL_Delay(50);

		toggle_LED_back(0);
		HAL_Delay(50);

		toggle_LED_right(0);
		HAL_Delay(50);
	}
	for(int i = 0; i < 6; i++){
		toggle_LED_all(2);
		HAL_Delay(50);
	}
	toggle_LED_all(1);
}

void config_knock_adc () {
	adcUtil_setup(ADC1, adcUtil_12bit);
	
	// Set PC0 to analog mode
	GPIOC->MODER |= 3 << GPIO_MODER_MODER0_Pos;
	GPIOC->MODER |= 3 << GPIO_MODER_MODER3_Pos;
	GPIOC->MODER |= 3 << GPIO_MODER_MODER4_Pos;
	GPIOC->MODER |= 3 << GPIO_MODER_MODER5_Pos;
	
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR0;
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR1;
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR2;
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR3;

	adcUtil_enableChannel(ADC1, 10);
	adcUtil_enableChannel(ADC1, 13);
	adcUtil_enableChannel(ADC1, 14);
	adcUtil_enableChannel(ADC1, 15);

	
	adcUtil_calibrate(ADC1, 1);	
}

void unlock () {
	servoUtil_setServo(8);
}

void lock () {
	servoUtil_setServo(0);
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
	
	/* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN Init */
	
	// Enable the RCC clocks
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();
	
	config_red();
	config_green();
	config_blue();
	config_orange();
	config_sideLEDs(); // additional LED's on pins PA8-PA12, and PA15
	
	
	
	pwmInit();
	
	dmaUtil_configChannel();
	config_knock_adc();
	servoUtil_configServo();
	lock();
	config_usart(115200);

	usart_transmit_str("USART READY!\n");
	
	HAL_Delay(1000);

	int8_t initVal = initAccelerometer();
	if(initVal == 1) {
		while (1) {
			toggle_red(2);
			HAL_Delay(250);
		}
	}
	else if(initVal == 2) {
		while (1) {
			toggle_orange(2);
			HAL_Delay(250);
		}
	}
	
  /* USER CODE END Init */

  

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

	/*
	if(!accelSetupRegisters()) {
		toggle_red(1);
		while(1);
	}
	*/
	
	accelSetupRegisters();
	
	usart_transmit_str("Config done!\n\r");
	toggle_blue(1);
	HAL_Delay(1000);
	toggle_blue(0);
	
	HAL_Delay(5000);
	lock(); // Lock the servo on start after 5 seconds
	
	enum PuzzleStateType mainState = Puzzle1;
	
	//toggle_LED_all(1);
	//victoryBoxLEDs();
	//while(1);
	

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

	/*	
	usart_transmit_str("ADC OVR: ");
	usart_transmit_int(ADC1->ISR & ADC_ISR_OVR);
	usart_transmit_str("Channel 1: ");
	usart_transmit_int(dmaUtil_buffer[0]);
	usart_transmit_str("Channel 2: ");
	usart_transmit_int(dmaUtil_buffer[1]);
	usart_transmit_str("Channel 3: ");
	usart_transmit_int(dmaUtil_buffer[2]);
	usart_transmit_str("Channel 4: ");
	usart_transmit_int(dmaUtil_buffer[3]);*/
	//usart_transmit_int(ADC1->DR);
	//HAL_Delay(100);
	//continue; // SKIP SWITCH HERE FOR DEBUG
  

	switch (mainState) {
		case Puzzle1:
			if (doPuzzle1()) {
				playFanfare(1);
				flashBoxLEDs();
				mainState = Puzzle2;
			}
			break;
		
		case Puzzle2:
			toggle_blue(1);	
			if (doPuzzle2()) {
				playFanfare(2);
				flashBoxLEDs();
				mainState = Puzzle3;
			}
			break;

		case Puzzle3:
			if (doPuzzle3()) {
				playFanfare(3);
				flashBoxLEDs();
				mainState = GameEnd;
			}
			break;
			
		case GameEnd:
			doGameEnd();
			break;
		}
	
		HAL_Delay(1);
  }
  /* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
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
