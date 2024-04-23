#include "accel.h"

#include "main.h"
#include "stm32f0xx_hal.h"

const uint8_t ACCEL_ADDR = 0x68 << 1;
const uint8_t X_ADDR = 0x3B;
const uint8_t Y_ADDR = 0x3D;
const uint8_t Z_ADDR = 0x3F;
const uint8_t CFG_REG = 26;
const uint8_t ACCEL_CFG_REG = 28;
const uint8_t ACCEL_CFG2_REG = 29;
const uint8_t PWR_MGMT_REG = 107;
const uint8_t PWR_MGMT2_REG = 108;

// HAL I2C struct
I2C_HandleTypeDef hi2c2;

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

//initialize
int8_t initAccelerometer() {
	
	MX_GPIO_Init();
  MX_I2C2_Init();
	
	HAL_Delay(1000);
	
	accelCheckWhoAmI(); // DO NOT DELETE. sends a message to reset everything ???
	
	if (accelCheckWhoAmI()) {
		return 1;
	}
	
	if (accelSetupRegisters()) {
		return 2;
	}
	
	return 0;
}

//SCL: B10
//SCD: B11
// checks if the accelerometer WHO_AM_I register matches
// returns 2 on I2C failure
// returns 1 if address does not match
// returns 0 if address does match
uint8_t accelCheckWhoAmI() {
	uint8_t retdata[] = {0};
	HAL_StatusTypeDef status;
	
	uint8_t WHOAMI[] = {0x75};
	status = HAL_I2C_Master_Transmit(&hi2c2, ACCEL_ADDR, WHOAMI, 1, 1000);
	if (status) {
		if (status == HAL_ERROR) {
			toggle_blue(1);
		}
		return 2;
	}
	
	status = HAL_I2C_Master_Receive(&hi2c2, ACCEL_ADDR, retdata, 1, 1000);
	if (status) {
		return 2;
	}
	
	if (retdata[0] == 0x71) {
		return 0;
	}
	
	return 1;
}

// return 0 on success
// return 1 on I2C error
int accelSetupRegisters() {
	HAL_StatusTypeDef status;
	
	uint8_t RESET[] = {PWR_MGMT_REG, 0x80}; // reset!
	status = HAL_I2C_Master_Transmit(&hi2c2, ACCEL_ADDR, RESET, 2, 1000);
	if (status) {
		return 1;
	}
	
	uint8_t PWR_MGMT2[] = {PWR_MGMT2_REG, 0x07}; // accelerometer on, gyro off
	status = HAL_I2C_Master_Transmit(&hi2c2, ACCEL_ADDR, PWR_MGMT2, 2, 1000);
	if (status) {
		return 1;
	}
	
	uint8_t ACCEL_CFG[] = {ACCEL_CFG_REG, 0x08}; //4g range
	status = HAL_I2C_Master_Transmit(&hi2c2, ACCEL_ADDR, ACCEL_CFG, 2, 1000);
	if (status) {
		return 1;
	}
	
	uint8_t ACCEL_CFG2[] = {ACCEL_CFG2_REG, 0x06}; // 5hz low pass
	status = HAL_I2C_Master_Transmit(&hi2c2, ACCEL_ADDR, ACCEL_CFG2, 2, 1000);
	if (status) {
		return 1;
	}
	
	return 0;
}

int abs(int num) {
	if (num < 0) {
		return -num;
	}
	return num;
}

// Return the most positive axis
// negative on error
AccelDirection accelReadAxis() {
	uint8_t retdata[6];
	HAL_StatusTypeDef status;
	
	uint8_t XOUT_H[] = {X_ADDR};
	status = HAL_I2C_Master_Transmit(&hi2c2, ACCEL_ADDR, XOUT_H, 1, 1000);
	if (status) {
		return ACCEL_DIR_ERROR;
		// see what error this is
	}
	
	status = HAL_I2C_Master_Receive(&hi2c2, ACCEL_ADDR, retdata, 6, 1000);
	if (status) {
		return ACCEL_DIR_ERROR;
	}
	
	int16_t X = ((uint8_t) retdata[0] << 8) | retdata[1];
	int16_t Y = ((uint8_t) retdata[2] << 8) | retdata[3];
	int16_t Z = ((uint8_t) retdata[4] << 8) | retdata[5];
	
	if (abs(X) > abs(Y)) {
		if (abs(Z) > abs(X)) {
			// Z is biggest
			if (Z > 0) {
				return Z_POS;
			}
			else {
				return Z_NEG;
			}
		}
		else {
			// X is biggest
			if (X > 0) {
				return X_POS;
			}
			else {
				return X_NEG;
			}
		}
		
	}
	else {
		// Y is biggest
		if (Y > 0) {
			return Y_POS;
		}
		else {
			return Y_NEG;
		}
	}
}
