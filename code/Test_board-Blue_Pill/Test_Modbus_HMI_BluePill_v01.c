/* USER CODE BEGIN Header */

/*
  Name: Test_Modbus_HMI_BluePill_v01.c
  Suggested board: Blue Pill (STM32F103C8T6)
  Suggested development environment: STM32CubeIDE
  
  Purpose:
    1. Receive Modbus-RTU command from the main board through RS-485 via the UART1.
    2. Generate the Modbus data with CRC and counting value.
    3. Send the Modbus data back.
    
  Suggested hardware setup:
    1. UART1: A "RS-485 to TTL module" is used to convert the RS-485 signal because STM32 NUCLEO-F446RE (and Blue Pill) does not support RS-485 directly.
  Suggested software (STM32CubeIDE) setup:
    1. Pinout & configuration/Connectivity
      1.1. Turn on USART1, Mode: Asynchronous, Basic parameters:   9600 8N1
  Suggested library for calculating CRC:
    1. The CRC 16 calculation function is available from Lammert Bies, https://github.com/lammertb/libcrc .
    2. On-line CRC calculation and free library, https://www.lammertbies.nl/comm/info/crc-calculation .
    3. The key files of the library used in this project is put in the [library](library).
 
  Date:   14 Sep. 2021
  Author: Dr. Hsien-Ching Chung
  ORCID:  https://orcid.org/0000-0001-9364-8858
 
  Project Link: https://github.com/HsienChing/RS-485_Modbus-RTU_Call_Response_and_HMI_Display_with_CRC_for_STM32
 
  License: MIT License
  Copyright (c) 2021 Hsien-Ching Chung
*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
  #include <stdbool.h>
  #include <stdlib.h>
  #include "checksum.h"
  // "checksum.h" is the header file for using the crc_modbus() function.
  // This file should be placed in the "[Project]/Core/Inc" folder.
  // Ref: Lammert Bies, https://github.com/lammertb/libcrc/blob/master/include/checksum.h
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  // CRC16-related functions
  static void      init_crc16_tab(void);
  static bool      crc_tab16_init = false;
  static uint16_t  crc_tab16[256];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint8_t     buf[8];     // Buffer for message.

    // Modbus-RTU Command
    uint8_t     Command_Modbus[8] = {0x01,0x04,0x30,0x00,0x00,0x0B,0xBE,0xCD};
    // [Modbus address convention]
    // The usage of the Modbus address (0x3000 = 12288) in "Command_Modbus[8]" does not follow the traditional Modbus address convention.
    // Convention: discrete input numbers (1 bit (off/on), 0 or 1) start with 1 and span from 10001 to 19999.
    // The users are suggested to design the system following the address convention (or following the device's address convention).
    // Ref: https://en.wikipedia.org/wiki/Modbus

    // Modbus-RTU Data
    uint8_t     Data_Modbus[27] = { 0x01,0x04,0x16,0x00,0x14,0x04,0xB0,0x02,0x00,0x00,
                                    0x15,0x00,0x0C,0x00,0xFF,0x00,0x0E,0x00,0x27,0x00,
                                    0x00,0x00,0x22,0x00,0x02,0x9A,0xC7};

    uint16_t    value = 0;  // Value generator
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if( HAL_UART_Receive(&huart1, buf, sizeof(buf), 2000) == HAL_OK ) {     // When receiving data from UART1 successfully
      if( buf[6] == Command_Modbus[6] && buf[7] == Command_Modbus[7] ) {    // When CRC16 check is passed successfully

    	// Turn on the On-board LED to show the beginning of the data processing.
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

        value += 1;                        // Value generator

        Data_Modbus[5] = value >> 8;       // Put the High byte of "value" in the array element.
        Data_Modbus[6] = value;            // Put the Low  byte of "value" in the array element.

        // Use crc_modbus(const unsigned char *input_str, size_t num_bytes) to calculate CRC16
        uint16_t CRC16_cal = crc_modbus( Data_Modbus, sizeof(Data_Modbus)-2 );

        Data_Modbus[25] = CRC16_cal;       // Put the Low  byte of "CRC16_cal" in the array element.
        Data_Modbus[26] = CRC16_cal >> 8;  // Put the High byte of "CRC16_cal" in the array element.

        HAL_UART_Transmit(&huart1, Data_Modbus, sizeof(Data_Modbus), 100);  // Send the Modbus data out through UART1.

        // Turn off the On-board LED to show the beginning of the data processing.
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      }
    }
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*
uint16_t crc_modbus( const unsigned char *input_str, size_t num_bytes );
The function crc_modbus() calculates the 16 bits Modbus CRC in one pass for
a byte string of which the beginning has been passed to the function. The
number of bytes to check is also a parameter.
Ref: Lammert Bies, https://github.com/lammertb/libcrc/blob/master/src/crc16.c
*/
uint16_t crc_modbus(const unsigned char *input_str, size_t num_bytes) {

	uint16_t crc;
	const unsigned char *ptr;
	size_t a;

	if (!crc_tab16_init) init_crc16_tab();

	crc = CRC_START_MODBUS;
	ptr = input_str;

	if (ptr != NULL) for (a = 0; a < num_bytes; a++) {

		crc = (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)*ptr++) & 0x00FF];
	}

	return crc;

}  /* crc_modbus */

/*
static void init_crc16_tab( void );
For optimal performance uses the CRC16 routine a lookup table with values
that can be used directly in the XOR arithmetic in the algorithm. This
lookup table is calculated by the init_crc16_tab() routine, the first time
the CRC function is called.
Ref: Lammert Bies, https://github.com/lammertb/libcrc/blob/master/src/crc16.c
*/
static void init_crc16_tab(void) {

	uint16_t i;
	uint16_t j;
	uint16_t crc;
	uint16_t c;

	for (i = 0; i < 256; i++) {

		crc = 0;
		c = i;

		for (j = 0; j < 8; j++) {

			if ((crc ^ c) & 0x0001) crc = (crc >> 1) ^ CRC_POLY_16;
			else                      crc = crc >> 1;

			c = c >> 1;
		}

		crc_tab16[i] = crc;
	}

	crc_tab16_init = true;

}  /* init_crc16_tab */

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
