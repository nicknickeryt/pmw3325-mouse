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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Rotary.h"
#include "sensor.h"
#include "usbd_hid.h"
#include <stdbool.h>
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define POLLING_MS 1

uint8_t rxDummy = 0x03;

bool buttonLeftPressed = 0;
bool buttonRightPressed = 0;

bool buttonCenterPressed = 0;
bool buttonCPIPressed = 0;

volatile int8_t scrollDirRaw = 0;
volatile uint8_t scrollDir = 0;

bool ENCA = 0;
bool ENCB = 0;

uint8_t HID_Buffer[4];

void processEncoder() {

  ENCA = HAL_GPIO_ReadPin(ENCA_GPIO_Port, ENCA_Pin);
  ENCB = HAL_GPIO_ReadPin(ENCB_GPIO_Port, ENCB_Pin);

  scrollDirRaw = Pin_process(ENCA, ENCB);
  if (scrollDirRaw == DIR_NONE)
    scrollDir = 0x00;
  else
    scrollDir = scrollDirRaw == DIR_CW ? 0x01 : 0xFF;
}

void setVibraPWM(uint8_t value);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
  case (LEFT_Pin):
    buttonLeftPressed = !HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin);
    break;
  case (RIGHT_Pin):
    buttonRightPressed = !HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin);
    break;
  case (CENTER_Pin):
    buttonCenterPressed = !HAL_GPIO_ReadPin(CENTER_GPIO_Port, CENTER_Pin);
    break;
  case (CPI_Pin):
    buttonCPIPressed = !HAL_GPIO_ReadPin(CPI_GPIO_Port, CPI_Pin);
    break;
  case (ENCA_Pin):
    ENCA = HAL_GPIO_ReadPin(ENCA_GPIO_Port, ENCA_Pin);
    break;
  case (ENCB_Pin):
    ENCB = HAL_GPIO_ReadPin(ENCB_GPIO_Port, ENCB_Pin);
    break;
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) { sensorTxCallback(); }

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) { sensorRxCallback(); }

uint8_t cpiMode = 0;
bool shouldChangeCPI = 0;
bool doubleClickEnabled = false;
uint32_t lastCpiPressTime = 0;
uint32_t cpiPressStartTime = 0;
bool cpiButtonHeld = false;

#define DEBOUNCE_DELAY 150    // 150 ms debounce delay
#define LONG_PRESS_DELAY 3000 // 3000 ms (3 seconds) long press delay

bool vibraEnabled = true;

void processCPI() {
  uint32_t currentTime = HAL_GetTick();

  if(buttonCPIPressed && buttonLeftPressed) {
    vibraEnabled = false;
    return;
  }

  if(buttonCPIPressed && buttonRightPressed) {
    vibraEnabled = true;
    return;
  }

  if (buttonCPIPressed && !cpiButtonHeld) {
    cpiButtonHeld = true;
    cpiPressStartTime = currentTime;
  }

  if (!buttonCPIPressed && cpiButtonHeld) {
    cpiButtonHeld = false;
    if ((currentTime - lastCpiPressTime) > DEBOUNCE_DELAY) {
    if ((currentTime - lastCpiPressTime) > DEBOUNCE_DELAY) {
      shouldChangeCPI = 1;
      lastCpiPressTime = currentTime;
    }
  }

  if (cpiButtonHeld && (currentTime - cpiPressStartTime) > LONG_PRESS_DELAY) {
    doubleClickEnabled = !doubleClickEnabled;
    cpiButtonHeld = false; // Reset the hold state
  }

  if (shouldChangeCPI) {
    cpiMode++;
    if (cpiMode > 5)
      cpiMode = 0;
    shouldChangeCPI = 0;
    buttonCPIPressed = 0;

    uint8_t reg = RESOLUTION;
    uint8_t data;

    switch (cpiMode) {
    case 0:
      data = 0x00;
      break;
    case 1:
      data = 0x05;
      break;
    case 2:
      data = 0x09;
      break;
    case 3:
      data = 0x17;
      break;
    case 4:
      data = 0x2F;
      break;
    case 5:
      data = 0x5F;
      break;
    }
    sensorWrite(&reg, &data);
  }
}

uint32_t lastReportTime = 0;

// TODO: Implement double click mode
void doubleClick() {}

int8_t signedX;
int8_t signedY;

void setVibraPWM(uint8_t value) {
  if (value > 80)
    value = 80;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, value);
}

uint8_t getVibraPWM(int8_t deltaX, int8_t deltaY) {
  uint8_t absDeltaX = (deltaX < 0) ? -deltaX : deltaX;
  uint8_t absDeltaY = (deltaY < 0) ? -deltaY : deltaY;

  uint8_t sum = absDeltaX + absDeltaY;

  uint8_t vibraPercent;
  if (sum < 10)
    vibraPercent = sum * 10;
  else if (sum < 50)
    vibraPercent = 100 + (sum - 10) * 2;
  else
    vibraPercent = 180 + (sum - 50);

  if (vibraPercent > 70)
    vibraPercent = 70;

  return vibraPercent + 10;
}

bool shouldVibrateButton = 0;
bool buttonPressed = 0;
uint32_t lastButtonVibraTime = 0;
uint32_t buttonPressStartTime = 0;
uint32_t scrollVibraStartTime = 0;
bool scrollVibraActive = 0;

void processVibra() {

  if(!vibraEnabled) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    return;
  }

  signedX = (int8_t)getSensorDeltaX();
  signedY = (int8_t)getSensorDeltaY();

  uint8_t value = (getVibraPWM(signedX, signedY) * 0.7);
  uint32_t currentTime = HAL_GetTick();

  if (buttonLeftPressed || buttonRightPressed || buttonCenterPressed || buttonCPIPressed || scrollDir != 0) {
    if (!buttonPressed) {
      buttonPressed = 1;
      buttonPressStartTime = currentTime;
      shouldVibrateButton = 1;
    }

    if (scrollDir != 0 && !scrollVibraActive) {
      scrollVibraStartTime = currentTime;
      scrollVibraActive = 1;
    }

    if (shouldVibrateButton && (currentTime - buttonPressStartTime) < 60) {
      value = 50;
    } else {
      shouldVibrateButton = 0;
    }

    lastButtonVibraTime = currentTime;
  } else {
    buttonPressed = 0;
    shouldVibrateButton = 0;
  }

  if (scrollVibraActive && (currentTime - scrollVibraStartTime) < 20) {
    value = 60;
  } else {
    scrollVibraActive = 0;
  }

  if (value > 80)
    value = 80;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, value);
}

void processMouse() {
  processCPI();

  if ((HAL_GetTick() - lastReportTime) >= POLLING_MS) {
    lastReportTime = HAL_GetTick();

    processSensor();
    processEncoder();
    processVibra();

    HID_Buffer[0] = 0;
    HID_Buffer[0] = (buttonLeftPressed << 0) | (buttonRightPressed << 1) |
                    (buttonCenterPressed << 2);

    HID_Buffer[1] = getSensorDeltaX();
    HID_Buffer[2] = getSensorDeltaY();
    HID_Buffer[3] = scrollDir;

    USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, 4);
  }
}

void initVibra() { HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); }

void initLEDs() {
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

uint32_t lastLEDUpdateTime = 0;
uint16_t pwmValueRed = 1000;
uint16_t pwmValueGreen = 0;
uint16_t pwmValueBlue = 0;
int8_t directionRed = -1;
int8_t directionGreen = 1;
int8_t directionBlue = 0;

void processLEDs() {
  uint32_t currentTime = HAL_GetTick();

  if ((currentTime - lastLEDUpdateTime) >= 10) {
    lastLEDUpdateTime = currentTime;

    pwmValueRed += directionRed;
    pwmValueGreen += directionGreen;
    pwmValueBlue += directionBlue;

    if (pwmValueRed == 1000 && pwmValueGreen == 0 && pwmValueBlue == 0) {
      directionRed = -1;
      directionGreen = 1;
      directionBlue = 0;
    } else if (pwmValueRed == 0 && pwmValueGreen == 1000 && pwmValueBlue == 0) {
      directionRed = 0;
      directionGreen = -1;
      directionBlue = 1;
    } else if (pwmValueRed == 0 && pwmValueGreen == 0 && pwmValueBlue == 1000) {
      directionRed = 1;
      directionGreen = 0;
      directionBlue = -1;
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwmValueBlue);  // BLUE
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwmValueRed);   // RED
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmValueGreen); // GREEN
  }
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
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  powerUpSensor();
  testSensor();
  initLEDs();
  initVibra();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    processMouse();
    processLEDs();
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : ENCB_Pin ENCA_Pin */
  GPIO_InitStruct.Pin = ENCB_Pin|ENCA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGHT_Pin CENTER_Pin LEFT_Pin CPI_Pin */
  GPIO_InitStruct.Pin = RIGHT_Pin|CENTER_Pin|LEFT_Pin|CPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
