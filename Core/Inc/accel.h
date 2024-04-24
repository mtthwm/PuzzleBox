#include "stm32f0xx_hal.h"

#ifndef ACCEL_H
#define ACCEL_H

extern const uint8_t ACCEL_ADDR;
extern const uint8_t X_ADDR;
extern const uint8_t Y_ADDR;
extern const uint8_t Z_ADDR;
extern const uint8_t CFG_REG;
extern const uint8_t ACCEL_CFG_REG;
extern const uint8_t ACCEL_CFG2_REG;
extern const uint8_t PWR_MGMT_REG;
extern const uint8_t PWR_MGMT2_REG;

extern const uint16_t AXIS_THRESHOLD;

// HAL I2C struct
extern I2C_HandleTypeDef hi2c2;

enum AccelerometerDirection {
	X_POS,
	X_NEG,
	Y_POS,
	Y_NEG,
	Z_POS,
	Z_NEG,
	NO_AXIS,
	ACCEL_DIR_ERROR
};

typedef enum AccelerometerDirection AccelDirection;

//initialize
int8_t initAccelerometer();

//SCL: B10
//SCD: B11
// checks if the accelerometer WHO_AM_I register matches
// returns 2 on I2C failure
// returns 1 if address does not match
// returns 0 if address does match
uint8_t accelCheckWhoAmI();

// return 0 on success
// return 1 on I2C error
int accelSetupRegisters();

int abs(int num);

// Return the axis with the largest magnitude
// ACCEL_DIR_ERROR on error
AccelDirection accelReadAxis();


#endif