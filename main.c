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
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "mpu6500.h"  // Include header for MPU6500 register definitions and functions (if any)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6500_SPI &hspi1           // Define the SPI instance for MPU6500
#define MPU6500_CS_PIN GPIO_PIN_4    // Define the Chip Select (CS) pin for MPU6500
#define MPU6500_CS_PORT GPIOA        // Define the GPIO port for CS pin
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;            // SPI handle structure
UART_HandleTypeDef huart2;          // UART handle structure

/* USER CODE BEGIN PV */
uint8_t gyro_data[6];               // Buffer to store raw gyroscope data
int16_t gyro_x, gyro_y, gyro_z;     // Variables to store raw gyroscope values
float gyro_x_dps, gyro_y_dps, gyro_z_dps; // Variables to store gyroscope values in dps
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void MPU6500_Init(void);            // Initialize the MPU6500 sensor
void MPU6500_Read_Gyro(void);       // Read gyroscope data from MPU6500
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Select the MPU6500 (pull CS low)
void MPU6500_Select() {
    HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_RESET);
}

// Deselect the MPU6500 (pull CS high)
void MPU6500_Deselect() {
    HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_SET);
}

// Write to a register on the MPU6500
void MPU6500_Write_Reg(uint8_t reg, uint8_t data) {
    MPU6500_Select();
    uint8_t buffer[2] = {reg & 0x7F, data}; // MSB = 0 indicates a write operation
    HAL_SPI_Transmit(MPU6500_SPI, buffer, 2, HAL_MAX_DELAY);
    MPU6500_Deselect();
}

// Read from a register on the MPU6500
uint8_t MPU6500_Read_Reg(uint8_t reg) {
    MPU6500_Select();
    uint8_t tx_buffer = reg | 0x80;  // MSB = 1 indicates a read operation
    uint8_t rx_buffer;
    HAL_SPI_Transmit(MPU6500_SPI, &tx_buffer, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(MPU6500_SPI, &rx_buffer, 1, HAL_MAX_DELAY);
    MPU6500_Deselect();
    return rx_buffer;
}

// Initialize the MPU6500
void MPU6500_Init() {
    MPU6500_Write_Reg(0x6B, 0x00); // Exit sleep mode
    MPU6500_Write_Reg(0x1B, 0x18); // Set gyro full scale to +/- 2000 dps
    MPU6500_Write_Reg(0x1A, 0x03); // Set DLPF to 44 Hz
}

// Read gyroscope data from MPU6500
void MPU6500_Read_Gyro() {
    uint8_t reg = 0x43; // Gyroscope X_OUT_H register
    MPU6500_Select();
    HAL_SPI_Transmit(MPU6500_SPI, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(MPU6500_SPI, gyro_data, 6, HAL_MAX_DELAY);
    MPU6500_Deselect();

    // Combine high and low bytes for each axis
    gyro_x = (int16_t)(gyro_data[0] << 8 | gyro_data[1]);
    gyro_y = (int16_t)(gyro_data[2] << 8 | gyro_data[3]);
    gyro_z = (int16_t)(gyro_data[4] << 8 | gyro_data[5]);

    // Convert raw values to degrees per second (dps)
    gyro_x_dps = gyro_x / 16.4f;
    gyro_y_dps = gyro_y / 16.4f;
    gyro_z_dps = gyro_z / 16.4f;
}

// Redirect printf to UART
int _write(int file, char *data, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // User-defined variables or initialization can go here
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Additional user initialization code can go here
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Additional system initialization code
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MPU6500_Init(); // Initialize MPU6500 sensor
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    MPU6500_Read_Gyro(); // Read gyroscope data
    printf("Gyro X: %.2f dps, Y: %.2f dps, Z: %.2f dps\r\n", gyro_x_dps, gyro_y_dps, gyro_z_dps); // Print gyroscope data to UART
    HAL_Delay(500); // Wait 500 ms
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  // Optional pre-initialization code
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  // Optional pre-initialization code
  /* USER CODE END SPI1_Init 1 */

  /* SPI1 parameter configuration */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT; // Using 16-bit data size
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  // Optional post-initialization code
  /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  // Optional pre-initialization code
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  // Optional pre-initialization code
  /* USER CODE END USART2_Init 1 */

  /* UART parameter configuration */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200; // Set baud rate to 115200
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  // Optional post-initialization code
  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  // Optional pre-initialization code
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable clock for GPIOA

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Optional post-initialization code
  /* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq(); // Disable interrupts
  while (1) // Stay in infinite loop
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
  // Report file name and line number of assert failure
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
