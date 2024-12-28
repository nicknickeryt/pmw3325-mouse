#ifndef __SENSOR_H
#define __SENSOR_H

#include "stdint.h"

#define POWER_UP_RESET   0x3A
#define SHUTDOWN         0x3B
#define PRODUCT_ID       0x00
#define CONFIG           0x40
#define RESOLUTION       0x1B

#define MOTION           0x02
#define DELTA_X_L        0x03
#define DELTA_X_H        0x04
#define DELTA_Y_L        0x05
#define DELTA_Y_H        0x06

void sensorTxCallback();
void sensorRxCallback();

void sensorRead(uint8_t* txBuffer, uint8_t* rxBuffer);
void sensorWrite(uint8_t* reg, uint8_t* data);

void testSensor();
void processSensor();
void powerUpSensor();

uint8_t getSensorDeltaX();
uint8_t getSensorDeltaY();

#endif // __SENSOR_H