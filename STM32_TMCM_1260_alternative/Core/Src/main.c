/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



// Chip Select pin
#define TMC5160_CS_GPIO_Port GPIOA
#define TMC5160_CS_Pin GPIO_PIN_4
#define DRV_ENN_GPIO_Port GPIOA
#define DRV_ENN_Pin GPIO_PIN_2

#define IHOLD_IRUN_VALUE 0x61905
// IHOLD bits 4...0 (5 = 0101; 0x05); IRUN bits 12...8 (25 = 11001; 0x19 << 8 -> 0x1900); IHOLDDELAY bits 19...16 (6 = 0110; 0x06 << 16 -> 0x60000);
// total set bits in 0x10 register: 0x05 + 0x1900 + 0x60000 = 0x61905
// so hex packed of 19 bits are set from LSB side for IHOLD_IRUN (0x10) register;


// TMC5160 register addresses
#define GCONF       0x00
#define IHOLD_IRUN  0x10
#define CHOPCONF    0x6C
#define VSTART      0x23
#define Amax        0x26
#define Dmax        0x28
#define VMAX        0x27
#define XTARGET     0x2D



// Example values for 0X6C: CHOPCONF – CHOPPER CONFIGURATION
#define CHOPCONF_TOFF    3   // off time
#define CHOPCONF_HSTRT   4   // hysteresis start
#define CHOPCONF_HEND    1   // hysteresis end
#define CHOPCONF_TBL     2   // blank time
#define CHOPCONF_MRES    8   // full step (MRES = 0b1000)
#define CHOPCONF_INTPOL  0   // interpolation disabled

// Compose CHOPCONF
#define CHOPCONF_VALUE  ( ((CHOPCONF_TOFF  & 0xF) << 0)  \
                        | ((CHOPCONF_HSTRT & 0x7) << 4)  \
                        | ((CHOPCONF_HEND  & 0xF) << 7)  \
                        | ((CHOPCONF_TBL   & 0x3) << 15) \
                        | ((CHOPCONF_MRES  & 0xF) << 24) \
                        | ((CHOPCONF_INTPOL & 0x1) << 28) )



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


// Enable driver (active low)
static inline void TMC5160_Enable(void) {
    HAL_GPIO_WritePin(DRV_ENN_GPIO_Port, DRV_ENN_Pin, GPIO_PIN_RESET);
}

// Disable driver (active low)
static inline void TMC5160_Disable(void) {
    HAL_GPIO_WritePin(DRV_ENN_GPIO_Port, DRV_ENN_Pin, GPIO_PIN_SET);
}



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */


static inline void TMC5160_CS_LOW(void) {
    HAL_GPIO_WritePin(TMC5160_CS_GPIO_Port, TMC5160_CS_Pin, GPIO_PIN_RESET);
}
static inline void TMC5160_CS_HIGH(void) {
    HAL_GPIO_WritePin(TMC5160_CS_GPIO_Port, TMC5160_CS_Pin, GPIO_PIN_SET);
}

uint32_t TMC5160_WriteReg(uint8_t address, uint32_t value) {
    uint8_t tx[5], rx[5];
    uint32_t ret;

    tx[0] = address & 0x7F; // MSB=0 for write
    tx[1] = (value >> 24) & 0xFF;
    tx[2] = (value >> 16) & 0xFF;
    tx[3] = (value >> 8) & 0xFF;
    tx[4] = value & 0xFF;

    TMC5160_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, HAL_MAX_DELAY);
    TMC5160_CS_HIGH();

    ret = ((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 8) | rx[4];
    return ret;
}

uint32_t TMC5160_ReadReg(uint8_t address) {
    uint8_t tx[5], rx[5];
    uint32_t ret;

    tx[0] = 0x80 | (address & 0x7F); // MSB=1 for read
    tx[1] = tx[2] = tx[3] = tx[4] = 0;

    TMC5160_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, HAL_MAX_DELAY);
    TMC5160_CS_HIGH();

    ret = ((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 8) | rx[4];
    return ret;
}

void TMC5160_Init(void) {
    TMC5160_WriteReg(GCONF, 0x00000000);
    TMC5160_WriteReg(IHOLD_IRUN, IHOLD_IRUN_VALUE);
    TMC5160_WriteReg(CHOPCONF, CHOPCONF_VALUE);
    TMC5160_WriteReg(VSTART, 0x00000000);
    TMC5160_WriteReg(Amax, 500);
    TMC5160_WriteReg(Dmax, 500);
    TMC5160_WriteReg(VMAX, 50000);
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  TMC5160_CS_HIGH();
  HAL_Delay(10);

  // Driver disabled by default (external pull-up)
  TMC5160_Disable();
  HAL_Delay(10); // let things stabilize


  TMC5160_Init();

  // Enable driver after configuration
  TMC5160_Enable();
  HAL_Delay(10); // small delay to let driver enable
  // Never enable DRV_ENN before configuring registers — can cause motor to jerk with undefined current/microstep.
  // Driver can be disabled any time if you need to stop the motor immediately — the motor will coast.


  // Move 10 revolutions
  TMC5160_WriteReg(XTARGET, 2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Wait while motor reaches target
	  uint32_t xactual = TMC5160_ReadReg(0x21); // XACTUAL
	  if (xactual >= 2000) { // in full steps, so 10 revolutions × 200 steps/rev = 2000 steps
		  break;
	  }
	  HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  // Motor reached position
  while (1);

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
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
