#include "adc.h"

volatile uint16_t dmaUtil_buffer[4];

void adcUtil_setup (ADC_TypeDef* adcInstance, enum adcUtil_resolution res) {
	// Enable the clock to the ADC
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	// Set ADC to appropriate bit resolution, continuous conversion mode, hardware triggers disabled.
	adcInstance->CFGR1 |= res << ADC_CFGR1_RES_Pos;
	adcInstance->CFGR1 |= ADC_CFGR1_CONT;
	adcInstance->CFGR1 &= ~ADC_CFGR1_ALIGN_Msk;
	adcInstance->CFGR2 |= ADC_CFGR2_CKMODE_1;
}

void adcUtil_calibrate (ADC_TypeDef* adcInstance, char enableDMA) {
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
	
	if (enableDMA) {
		adcInstance->CFGR1 |= ADC_CFGR1_DMACFG;
		adcInstance->CFGR1 |= ADC_CFGR1_DMAEN;
	}
		
	// Signal that we are ready for conversion
	adcInstance->CR |= ADC_CR_ADSTART;
}
void adcUtil_enableChannel (ADC_TypeDef* adcInstance, uint8_t channelNumber) {
	// Set ADC to use channel 10 (ADC_IN10 additional function)
	adcInstance->CHSELR |= (1 << channelNumber);
}

void dmaUtil_configChannel () {
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	
	// Enable ADC to DMA remap on DMA channel
	SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_ADC_DMA_RMP;
	
	// Set memory size of our DMA transfers to 16 bit
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;
	
	// Set peripheral register size of our DMA transfers to 16 bit
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;
	
	// Set the memory address registers to increment after each read
	DMA1_Channel1->CCR |= DMA_CCR_MINC;
	DMA1_Channel1->CCR &= ~DMA_CCR_PINC;
	
	// Set circular mode
	DMA1_Channel1->CCR |= DMA_CCR_CIRC;
	
	// Set priority to high
	DMA1_Channel1->CCR |= DMA_CCR_PL_1;
	
	// Configure the registers to move data between
	DMA1_Channel1->CPAR = (uint32_t) &(ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t) dmaUtil_buffer;
	
	// Set the number of data to transmit
	DMA1_Channel1->CNDTR = 4;
	
	// Activate the channel
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	
}