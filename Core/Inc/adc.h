# ifndef ADC_H
# define ADC_H

#include "stm32f0xx_hal.h"


enum adcUtil_resolution {
	adcUtil_12bit,
	adcUtil_10bit,
	adcUtil_8bit,
	adcUtil_6bit
};
extern volatile uint16_t dmaUtil_buffer[4];

void adcUtil_setup (ADC_TypeDef* adcInstance, enum adcUtil_resolution res);
void adcUtil_calibrate (ADC_TypeDef* adcInstance, char enableDMA);
void adcUtil_enableChannel (ADC_TypeDef* adcInstance, uint8_t channelNumber);
void dmaUtil_configChannel ();

#endif