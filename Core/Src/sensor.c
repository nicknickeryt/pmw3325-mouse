
#include "sensor.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

uint8_t txProductID = PRODUCT_ID;
uint8_t rxProductID;

uint8_t txPowerUp = POWER_UP_RESET;
uint8_t txPowerUpData = 0x5A;

uint8_t txShutdown = SHUTDOWN;
uint8_t txShutdownData = 0xB6;


uint8_t txConfig = CONFIG;
uint8_t rxConfig;

uint8_t txMotion = MOTION;
uint8_t rxMotion;

uint8_t txDeltaX_L = DELTA_X_L;
uint8_t rxDeltaX_L;
uint8_t txDeltaX_H = DELTA_X_H;
uint8_t rxDeltaX_H;

uint8_t txDeltaY_L = DELTA_Y_L;
uint8_t rxDeltaY_L;
uint8_t txDeltaY_H = DELTA_Y_H;
uint8_t rxDeltaY_H;

uint16_t fullDeltaX;
uint16_t fullDeltaY;

bool txComplete = 0;
bool rxComplete = 0;

void sensorTxCallback() { txComplete = 1; }

void sensorRxCallback() { rxComplete = 1; }

void sensorChipSelect() {
  // HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
}

void sensorChipDeselect() {
  // HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
}

void sensorRead(uint8_t *txBuffer, uint8_t *rxBuffer) {
  sensorChipSelect();

  txComplete = 0;
  HAL_SPI_Transmit_IT(&hspi1, txBuffer, 1);
  while (!txComplete) {
  }

  rxComplete = 0;
  HAL_SPI_Receive_IT(&hspi1, rxBuffer, 1);
  while (!rxComplete) {
  }

  sensorChipDeselect();
}

void sensorWrite(uint8_t *reg, uint8_t *data) {
  uint8_t reg_addr = *reg | 0x80;
  sensorChipSelect();

  txComplete = 0;
  HAL_SPI_Transmit_IT(&hspi1, &reg_addr, 1);
  while (!txComplete) {
  }

  txComplete = 0;
  HAL_SPI_Transmit_IT(&hspi1, data, 1);
  while (!txComplete) {
  }

  sensorChipDeselect();
}

void powerUpSensor() {
  HAL_Delay(50);

  sensorChipDeselect();
  sensorChipSelect();
  sensorChipDeselect();

  uint8_t reg, data, testRead;

  reg = 0x3a;
  data = 0x5a;
  sensorWrite(&reg, &data);

  HAL_Delay(10);

  reg = 0x18;
  data = 0x39;
  sensorWrite(&reg, &data);

  reg = 0x02;
  testRead = 0xFF;
  sensorRead(&reg, &testRead);

  reg = 0x03;
  testRead = 0xFF;
  sensorRead(&reg, &testRead);

  reg = 0x04;
  testRead = 0xFF;
  sensorRead(&reg, &testRead);

  reg = 0x05;
  testRead = 0xFF;
  sensorRead(&reg, &testRead);

  reg = 0x06;
  testRead = 0xFF;
  sensorRead(&reg, &testRead);

  // Begin these writes

  reg = 0x78;
  data = 0x80;
  sensorWrite(&reg, &data);

  reg = 0x79;
  data = 0x80;
  sensorWrite(&reg, &data);

  reg = 0x79;
  data = 0x80;
  sensorWrite(&reg, &data);

  reg = 0x14;
  data = 0x80;
  sensorWrite(&reg, &data);

  reg = 0x20;
  data = 0x40;
  sensorWrite(&reg, &data);

  reg = 0x1A;
  data = 0x40;
  sensorWrite(&reg, &data);

  reg = 0x47;
  data = 0x00;
  sensorWrite(&reg, &data);

  reg = 0x48;
  data = 0x01;
  sensorWrite(&reg, &data);

  // ################### //

  reg = 0x60;
  data = 0x01;
  sensorWrite(&reg, &data);

  reg = 0x69;
  data = 0x03;
  sensorWrite(&reg, &data);

  reg = 0x1D;
  data = 0x90;
  sensorWrite(&reg, &data);

  reg = 0x1B;
  data = 0x2E;
  sensorWrite(&reg, &data);

  reg = 0x24;
  data = 0x05;
  sensorWrite(&reg, &data);

  reg = 0x56;
  data = 0x00;
  sensorWrite(&reg, &data);

  reg = 0x2C;
  data = 0x8A;
  sensorWrite(&reg, &data);

  reg = 0x2D;
  data = 0x58;
  sensorWrite(&reg, &data);

  reg = 0x40;
  data = 0x80;
  sensorWrite(&reg, &data);

  reg = 0x7F;
  data = 0x01;
  sensorWrite(&reg, &data);

  reg = 0x7A;
  data = 0x32;
  sensorWrite(&reg, &data);

  reg = 0x6A;
  data = 0x93;
  sensorWrite(&reg, &data);

  reg = 0x6B;
  data = 0x68;
  sensorWrite(&reg, &data);

  reg = 0x6C;
  data = 0x71;
  sensorWrite(&reg, &data);

  reg = 0x6D;
  data = 0x50;
  sensorWrite(&reg, &data);

  reg = 0x7F;
  data = 0x00;
  sensorWrite(&reg, &data);

  reg = 0x7F;
  data = 0x02;
  sensorWrite(&reg, &data);

  reg = 0x29;
  data = 0x1C;
  sensorWrite(&reg, &data);

  reg = 0x2A;
  data = 0x1A;
  sensorWrite(&reg, &data);

  reg = 0x2B;
  data = 0x90;
  sensorWrite(&reg, &data);

  reg = 0x40;
  data = 0x80;
  sensorWrite(&reg, &data);

  reg = 0x7F;
  data = 0x00;
  sensorWrite(&reg, &data);

  reg = 0x1B;
  data = 0x04;
  sensorWrite(&reg, &data);

  reg = 0x1B;
  data = 0x17;
  sensorWrite(&reg, &data);

  reg = 0x1E;
  data = 0x04;
  sensorWrite(&reg, &data);

  HAL_Delay(100);

//   // Set resolution
//   reg = RESOLUTION;
//   data = 0x17;
//   sensorWrite(&reg, &data);

}

void testSensor() { sensorRead(&txProductID, &rxProductID); }

void readSensorDeltaX() {
  sensorRead(&txDeltaX_L, &rxDeltaX_L);
  sensorRead(&txDeltaX_H, &rxDeltaX_H);
  fullDeltaX = (int16_t)((rxDeltaX_L << 8) | rxDeltaX_H);
}

void readSensorDeltaY() {
  sensorRead(&txDeltaY_L, &rxDeltaY_L);
  sensorRead(&txDeltaY_H, &rxDeltaY_H);
  fullDeltaY = (int16_t)((rxDeltaY_L << 8) | rxDeltaY_H);

}

void processSensor() {
    sensorRead(&txMotion, &rxMotion);

    readSensorDeltaX();
    readSensorDeltaY();
}

uint8_t getSensorDeltaX() { return fullDeltaX / 256; }
uint8_t getSensorDeltaY() { return fullDeltaY / 256; }