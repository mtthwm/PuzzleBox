#include "servo.h"

void servoUtil_configServo() {
	GPIO_InitTypeDef initStrPWM = {GPIO_PIN_1,
	GPIO_MODE_AF_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL,
	GPIO_AF2_TIM2};
	
	HAL_GPIO_Init(GPIOA, &initStrPWM);
	
	TIM2->PSC = 799;
	TIM2->ARR = 200;
	
	TIM2->CCMR1 &= ~TIM_CCMR1_CC2S; 
	TIM2->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos); 
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE; 
	TIM2->CCER |= TIM_CCER_CC2E; 
}

// Angle ranges from 0 to 16 "ticks"
void servoUtil_setServo(uint8_t angle) {	
	TIM2->CR1 &= ~TIM_CR1_CEN; // Disable Timer
	
	if (angle < 0 || angle > 16) {
		return;
	}
	TIM2->CCR2 = 6 + angle; // Set the duty cycle to be between 3 and 11 %
	
	TIM2->CR1 |= TIM_CR1_CEN; // Enable Timer
}