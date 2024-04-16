#define ACCEL_H

static const uint8_t ACCEL_ADDR = 0x68;
static const uint8_t X_ADDR = 0x28;
static const uint8_t Y_ADDR = 0x2A;
static const uint8_t CFG_REG = 26;
static const uint8_t ACCEL_CFG_REG = 28;
static const uint8_t ACCEL_CFG2_REG = 29;
static const uint8_t PWR_MGMT_REG = 107;
static const uint8_t PWR_MGMT2_REG = 108;


void initI2C() {
	// Clock to GPIO B
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// enable alternate function on pin B11
	GPIOB->MODER &= ~GPIO_MODER_MODER11_Msk;
	GPIOB->MODER |= GPIO_MODER_MODER11_1;
	
	// alternate function 1 is SDA for pin B11
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11_Msk;
	GPIOB->AFR[1] |= GPIO_AF1_I2C2 << GPIO_AFRH_AFSEL11_Pos;
	
	// enable alternate function on pin B13
	GPIOB->MODER &= ~GPIO_MODER_MODER13_Msk;
	GPIOB->MODER |= GPIO_MODER_MODER13_1;
	
	// alternate function 5 is SCL for pin B13
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL13_Msk;
	GPIOB->AFR[1] |= GPIO_AF5_I2C2 << GPIO_AFRH_AFSEL13_Pos;
	
	// output mode on pin B14
	GPIOB->MODER &= ~GPIO_MODER_MODER14_Msk;
	GPIOB->MODER |= GPIO_MODER_MODER14_0;
	
	// output mode on pin C0
	GPIOC->MODER &= ~GPIO_MODER_MODER0_Msk;
	GPIOC->MODER |= GPIO_MODER_MODER0_0;
	
	// open-drain for SDA and SCL on pins B11, B13
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
	
	// Pull up DA and SCL on pins B11, B13
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR13_0;
	
	// RCC clock to I2C2
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// Configure I2C timing for 400 kHz
	I2C2->TIMINGR |= (0x1  << I2C_TIMINGR_PRESC_Pos);
	I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);
	I2C2->TIMINGR |= (0xF  << I2C_TIMINGR_SCLH_Pos);
	I2C2->TIMINGR |= (0x2  << I2C_TIMINGR_SDADEL_Pos);
	I2C2->TIMINGR |= (0x4  << I2C_TIMINGR_SCLDEL_Pos);
	
	
	////////debug
	/*
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	// output mode on pin C0
	GPIOC->MODER &= ~GPIO_MODER_MODER0_Msk;
	GPIOC->MODER |= GPIO_MODER_MODER0_0;
	
	// start pins B14 and C0 high
	//GPIOB->ODR |= GPIO_ODR_14;
	//GPIOC->ODR |= GPIO_ODR_0;
	*/
	///////////
	
	// lastly, enable the I2C
	I2C2->CR1 |= I2C_CR1_PE;
	
}

static uint8_t sendI2CBytes(uint8_t address, uint8_t numBytes, uint8_t* data) {
	I2C2->CR2 = 0;
	
	// set send address
	I2C2->CR2 |= (address << 1) << I2C_CR2_SADD_Pos;
	// Send 1 byte
	I2C2->CR2 |= (numBytes) << I2C_CR2_NBYTES_Pos;
	// write operation
	I2C2->CR2 &= ~I2C_CR2_RD_WRN_Msk;
	// set start bit
	I2C2->CR2 |= I2C_CR2_START;
	
	// wait for send to complete
	while (!(I2C2->ISR & I2C_ISR_TXIS_Msk) && !(I2C2->ISR & I2C_ISR_NACKF_Msk));
	if (I2C2->ISR & I2C_ISR_NACKF_Msk) {
		// NACK
		I2C2->ICR |= I2C_ICR_NACKCF;
		
		return 0;
	}
	
	// send bytes
	for (size_t i = 0; i < numBytes; i++) {
		while (!(I2C2->ISR & I2C_ISR_TXE_Msk));
		I2C2->TXDR = data[i];
	}
	// wait for transfer complete
	while (!(I2C2->ISR & I2C_ISR_TC_Msk));
	
	return 1;
}

static uint8_t readI2CBytes(uint8_t address, uint8_t numBytes, uint8_t* data) {
	I2C2->CR2 = 0;
	// set send address
	I2C2->CR2 |= (address << 1) << I2C_CR2_SADD_Pos;
	// read 1 byte
	I2C2->CR2 |= (numBytes) << I2C_CR2_NBYTES_Pos;
	// read operation
	I2C2->CR2 |= I2C_CR2_RD_WRN;
	// set start bit for restart condition
	I2C2->CR2 |= I2C_CR2_START;
	
	// wait for read request to complete
	while (!(I2C2->ISR & I2C_ISR_RXNE_Msk) && !(I2C2->ISR & I2C_ISR_NACKF_Msk));
	if (I2C2->ISR & I2C_ISR_NACKF_Msk) {
		// NACK
		I2C2->ICR |= I2C_ICR_NACKCF;
		
		return 0;
	}
	
	// transfer good, read data
	for (size_t i = 0; i < numBytes; i++) {
		while (!(I2C2->ISR & I2C_ISR_RXNE_Msk));
		data[i] = I2C2->RXDR;
	}
	
	// wait for transfer complete
	while (!(I2C2->ISR & I2C_ISR_TC_Msk));
	
	return 1;
}

static void sendI2CStop() {
	I2C2->CR2 |= I2C_CR2_STOP;
}

/** send the request of a register to read from device at address, 
  * then read numBytes from that register into data
  */
static uint8_t sendI2CQuery(uint8_t address, uint8_t request, uint8_t numBytes, uint8_t* data) {
	if (!sendI2CBytes(address, 1, &request)) {
		return 0;
	}
	
	if (!readI2CBytes(address, numBytes, data)) {
		return 0;
	}
	
	sendI2CStop();
	return 1;
}

// checks if the accelerometer WHO_AM_I register matches
// return -1 on I2C failure
// returns 0 if address does not match
// returns 1 if address does match
int accelCheckWhoAmI() {
	uint8_t data = 0;
	
	if (!sendI2CQuery(ACCEL_ADDR, 0x75, 1, &data)) {
		return -1;
	}
	
	if (data == 0x71) {
		return 1;
	}
	
	return 0;
}


int accelSetupRegisters() {
	uint8_t RESET[] = {PWR_MGMT_REG, 0x80}; // reset!
	if (!sendI2CBytes(ACCEL_ADDR, 2, RESET)) {
		return 0;
	}
	sendI2CStop();
	
	HAL_Delay(50); // ensure reset I guess
	
	uint8_t PWR_MGMT2[] = {PWR_MGMT2_REG, 0x07}; // accelerometer on, gyro off
	if (!sendI2CBytes(ACCEL_ADDR, 2, PWR_MGMT2)) {
		return 0;
	}
	sendI2CStop();
	
	uint8_t ACCEL_CFG[] = {ACCEL_CFG_REG, 0x08}; //4g range
	if (!sendI2CBytes(ACCEL_ADDR, 2, ACCEL_CFG)) {
		return 0;
	}
	sendI2CStop();
	
	uint8_t ACCEL_CFG2[] = {ACCEL_CFG2_REG, 0x06}; // 5hz low pass
	if (!sendI2CBytes(ACCEL_ADDR, 2, ACCEL_CFG2)) {
		return 0;
	}
	sendI2CStop();
	
	
	return 1;
}

