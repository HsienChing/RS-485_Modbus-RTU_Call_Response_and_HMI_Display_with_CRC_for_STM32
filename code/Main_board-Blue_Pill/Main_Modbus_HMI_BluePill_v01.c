/* USER CODE BEGIN Header */

/*
  Name: Main_Modbus_HMI_BluePill_v01.c
  Suggested board: Blue Pill (STM32F103C8T6)
  Suggested development environment: STM32CubeIDE
 
  Purpose:
    1. Send Modbus-RTU command to the device through RS-485 via the UART1.
    2. Get responses from the device through RS-485.
    3. Send the device responses to the human-machine interface (HMI) via the UART3.
    4. Users can monitor the device responses from the serial monitor via UART2.
 
  Suggested hardware setup:
    1. UART1: A "RS-485 to TTL module" is used to convert the RS-485 signal because STM32 NUCLEO-F446RE (and Blue Pill) does not support RS-485 directly.
    2. UART3: A HMI with the UART interface is used to show the device responses through the UART3.
  Suggested software (STM32CubeIDE) setup:
    1. Pinout & configuration/Connectivity
      1.1. Turn on USART1, Mode: Asynchronous, Basic parameters:   9600 8N1
      1.2. Turn on USART2, Mode: Asynchronous, Basic parameters: 115200 8N1
      1.3. Turn on USART3, Mode: Asynchronous, Basic parameters: 115200 8N1
  Suggested library for calculating CRC:
    1. The CRC 16 calculation function is available from Lammert Bies, https://github.com/lammertb/libcrc .
    2. On-line CRC calculation and free library, https://www.lammertbies.nl/comm/info/crc-calculation .
    3. The key files of the library used in this project is put in the [library](library).

  Date:   15 Sep. 2021
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
  #include <stdio.h>
  #include <stdbool.h>
  #include <stdlib.h>
  #include <string.h>
  #include "checksum.h"
  // "checksum.h" is the header file for using the crc_modbus() function.
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
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  // CRC16-related functions
  static void     init_crc16_tab(void);
  static bool     crc_tab16_init = false;
  static uint16_t crc_tab16[256];
  long            map();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint8_t     buf[64];            // Buffer for message.

    // Modbus-RTU Command
    uint8_t     Command_Modbus[8] = {0x01,0x04,0x30,0x00,0x00,0x0B,0xBE,0xCD};
    // [Modbus address convention]
    // The usage of the Modbus address (0x3000 = 12288) in "Command_Modbus[8]" does not follow the traditional Modbus address convention.
    // Convention: discrete input numbers (1 bit (off/on), 0 or 1) start with 1 and span from 10001 to 19999.
    // The users are suggested to design the system following the address convention (or following the device's address convention).
    // Ref: https://en.wikipedia.org/wiki/Modbus

    uint8_t     Data_Modbus[27];    // Modbus-RTU Data

    // Device responses to Command_Modbus
    uint16_t    VAC_out;
    float       VAC_out_f;
    uint16_t    V_Batt;
    float       V_Batt_f;
    uint16_t    Load_percent;
    uint16_t    Load_power;

    uint8_t     EndHex[3] = {0xFF,0xFF,0xFF}; // End command for HMI
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_UART_Transmit(&huart1, Command_Modbus, sizeof(Command_Modbus), 100);            // Send Modbus command through UART1
    if( HAL_UART_Receive(&huart1, Data_Modbus, sizeof(Data_Modbus), 2000) == HAL_OK ) { // When receiving data from UART1 successfully
      
      // Send the received Modbus data to serial monitor through UART2
      HAL_UART_Transmit(&huart2, Data_Modbus, sizeof(Data_Modbus), 100);                
 
      // Get the CRC16 of the received Modbus data
      uint16_t CRC16_data = ( Data_Modbus[sizeof(Data_Modbus)-1] << 8 ) | Data_Modbus[sizeof(Data_Modbus)-2];
 
      // Calculate the CRC16 of the received Modbus data by the function
      //                   crc_modbus(const unsigned char *input_str, size_t num_bytes)
      uint16_t CRC16_cal = crc_modbus( Data_Modbus, sizeof(Data_Modbus)-2 );
 
      if ( CRC16_data == CRC16_cal ) {                                  // When CRC16 check is passed successfully
  
        // Send message to serial monitor through UART2
        strcpy((char*)buf, "CRC16 PASSED.\r\n");
        HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
   
        uint16_t value = (Data_Modbus[5] << 8) | Data_Modbus[6];        // Combine/Merge two bytes into one
        VAC_out = value;
        VAC_out_f = (float)value / 10.0;                                // Resolve the value
   
        sprintf((char*)buf, "VAC_out(V): %f\r\n", VAC_out_f);           // Show the value on the serial monitor
        HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
 
        sprintf((char*)buf, "x1.val=%d", VAC_out);                      // Send device response to HMI
        HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), 100);
        HAL_UART_Transmit(&huart3, EndHex, sizeof(EndHex), 100);
 
 
        value = (Data_Modbus[7] << 8) | Data_Modbus[8];                 // Combine/Merge two bytes into one
        V_Batt = value;
        V_Batt_f = (float)value / 10.0;                                 // Resolve the value
 
        sprintf((char*)buf, "V_Batt(V): %f\r\n", V_Batt_f);             // Show the value on the serial monitor
        HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
 
        sprintf((char*)buf, "x0.val=%d", V_Batt);                       // Send device response to HMI
        HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), 100);
        HAL_UART_Transmit(&huart3, EndHex, sizeof(EndHex), 100);
 
 
        Load_percent = Data_Modbus[12];                                 // Resolve the value
 
        sprintf((char*)buf, "Load_percent(%%): %d\r\n", Load_percent);  // Show the value on the serial monitor
        HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
 
        sprintf((char*)buf, "n0.val=%d", Load_percent);                 // Send device response to HMI
        HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), 100);
        HAL_UART_Transmit(&huart3, EndHex, sizeof(EndHex), 100);
 
 
        value = (Data_Modbus[13] << 8) | Data_Modbus[14];               // Combine/Merge two bytes into one
        Load_power = value;                                             // Resolve the value
 
        sprintf((char*)buf, "Load_power(W): %d\r\n", Load_power);       // Show the value on the serial monitor
        HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
 
        int HMI_Pointer = map(Load_power, 0, 2000, 0, 240);             // map(value, fromLow, fromHigh, toLow, toHigh)
 
        sprintf((char*)buf, "z0.val=%d", HMI_Pointer);                  // Send device response to HMI
        HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), 100);
        HAL_UART_Transmit(&huart3, EndHex, sizeof(EndHex), 100);
 
      } else {
        // Send message to serial monitor through UART2
        strcpy((char*)buf, "CRC16 FAILED.\r\n");
        HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
      }
    } else {
      // Send message to serial monitor through UART2
      strcpy((char*)buf, "Data receive FAILED. Resend Modbus command.\r\n");
      HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), 100);
    }
 
    //HAL_Delay(50); // Data processing rate control

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

// map(value, fromLow, fromHigh, toLow, toHigh)
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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
