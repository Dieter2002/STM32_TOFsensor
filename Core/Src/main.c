/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TCA9548A_ADDR  (0x70 << 1)  // Multiplexer adres
#define VL6180X_ADDR   (0x29 << 1)  // Sensor adres

int __io_putchar(int ch);
uint8_t VL6180X_ReadRange();
//void TCA9548A_SelectChannel();
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	       uint8_t status;
  	       uint8_t cmd;

  	       cmd = (1 << 4);
  	       HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR, &cmd, 1, HAL_MAX_DELAY);
  	       // Test of het sensoradres antwoordt
  	       status = HAL_I2C_IsDeviceReady(&hi2c1, VL6180X_ADDR, 3, 100);

  	       if(status == HAL_OK)
  	           printf("Sensor aanwezig op kanaal 4!\r\n");
  	       else
  	           printf("Sensor NIET gevonden op kanaal 4!\r\n");

  TCA9548A_SelectChannel(4);    // Selecteer kanaal 4
  HAL_Delay(10);
  VL6180X_SimpleInit();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      uint8_t afstand = VL6180X_ReadRange();
      printf("Afstand: %d mm\r\n", afstand);
//      HAL_Delay(250);

//	       printf("I2C scan op kanaal 4:\r\n");
//	       for(uint8_t addr=1; addr<128; addr++) {
//	           if(HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
//	               printf("Adres 0x%02X gevonden\r\n", addr);
//	           }
//	       }

//


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

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

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch){
  	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  	return ch;
}

void TCA9548A_SelectChannel(uint8_t channel)
{
		       uint8_t status;
		       uint8_t cmd;
    if (channel > 7) return;
//    uint8_t cmd = (1 << channel);
//    HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR, &cmd, 1, HAL_MAX_DELAY);
    cmd = (1 << 4);
    HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR, &cmd, 1, HAL_MAX_DELAY);
}

// VL6180X register schrijven (16-bit registeradres)
void VL6180X_WriteReg(uint16_t reg, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1, VL6180X_ADDR, reg, I2C_MEMADD_SIZE_16BIT, &val, 1, HAL_MAX_DELAY);
}

// VL6180X register lezen (16-bit registeradres)
uint8_t VL6180X_ReadReg(uint16_t reg)
{
    uint8_t val;
    HAL_StatusTypeDef status;

        status = HAL_I2C_Mem_Read(&hi2c1, VL6180X_ADDR, reg, I2C_MEMADD_SIZE_16BIT, &val, 1, HAL_MAX_DELAY);


    if (status != HAL_OK) {
        printf("FOUT: Leesfout bij reg 0x%04X, status = %d\r\n", reg, status);
    }

//    printf("FOUT: Leesfout bij reg 0x%04X, status = %d\r\n", reg, status);


    return val;
}

void VL6180X_SimpleInit(void)
{
    // Reset settings (optioneel, kan overslagen worden)
    VL6180X_WriteReg(0x0207, 0x01);
    VL6180X_WriteReg(0x0208, 0x01);
    VL6180X_WriteReg(0x0096, 0x00);
    VL6180X_WriteReg(0x0097, 0xfd);
    VL6180X_WriteReg(0x00e3, 0x00);
    VL6180X_WriteReg(0x00e4, 0x04);
    VL6180X_WriteReg(0x00e5, 0x02);
    VL6180X_WriteReg(0x00e6, 0x01);
    VL6180X_WriteReg(0x00e7, 0x03);
    VL6180X_WriteReg(0x00f5, 0x02);
    VL6180X_WriteReg(0x00d9, 0x05);
    VL6180X_WriteReg(0x00db, 0xce);
    VL6180X_WriteReg(0x00dc, 0x03);
    VL6180X_WriteReg(0x00dd, 0xf8);
    VL6180X_WriteReg(0x009f, 0x00);
    VL6180X_WriteReg(0x00a3, 0x3c);
    VL6180X_WriteReg(0x00b7, 0x00);
    VL6180X_WriteReg(0x00bb, 0x3c);
    VL6180X_WriteReg(0x00b2, 0x09);
    VL6180X_WriteReg(0x00ca, 0x09);
    VL6180X_WriteReg(0x0198, 0x01);
    VL6180X_WriteReg(0x01b0, 0x17);
    VL6180X_WriteReg(0x01ad, 0x00);
    VL6180X_WriteReg(0x00ff, 0x05);
    VL6180X_WriteReg(0x0100, 0x05);
    VL6180X_WriteReg(0x00ff, 0x00);
    VL6180X_WriteReg(0x010a, 0x30);
    VL6180X_WriteReg(0x003f, 0x46);
    VL6180X_WriteReg(0x01e3, 0xff);
    VL6180X_WriteReg(0x01e4, 0x01);
    VL6180X_WriteReg(0x01e5, 0x00);

    // Recommended : Public registers - See data sheet for more detail
    VL6180X_WriteReg(0x0011, 0x10); // Enables polling for 'New Sample ready'
                          // when measurement completes
    VL6180X_WriteReg(0x010a, 0x30); // Set the averaging sample period
                          // (compromise between lower noise and
                          // increased execution time)
    VL6180X_WriteReg(0x003f, 0x46); // Sets the light and dark gain (upper
                          // nibble). Dark gain should not be
                          // changed.
    VL6180X_WriteReg(0x0031, 0xFF); // sets the # of range measurements after
                          // which auto calibration of system is
                          // performed
    VL6180X_WriteReg(0x0041, 0x63); // Set ALS integration time to 100ms
    VL6180X_WriteReg(0x002e, 0x01); // perform a single temperature calibration
                          // of the ranging sensor

    // Optional: Public registers - See data sheet for more detail
    VL6180X_WriteReg(0x001b,
           0x09);         // Set default ranging inter-measurement
                          // period to 100ms
    VL6180X_WriteReg(0x003e, 0x31); // Set default ALS inter-measurement period
                          // to 500ms
    VL6180X_WriteReg(0x0014, 0x24); // Configures interrupt on 'New Sample
                          // Ready threshold event'
}

uint8_t VL6180X_ReadRange(void)
{
    // Stap 1: Start de range meting
    VL6180X_WriteReg(0x0018, 0x01);  // SYSRANGE__START

    // Stap 2: Wacht tot meting klaar is
    uint8_t status;
    uint32_t timeout = 1000;
    do {
        status = VL6180X_ReadReg(0x004F);  // RESULT__INTERRUPT_STATUS_GPIO
        HAL_Delay(1);
        timeout--;
        if (timeout == 0) {
            printf("Timeout tijdens meting\r\n");
            return 255;
        }
    } while ((status & 0x07) != 0x04);

    // Stap 3: Lees afstand
    uint8_t range = VL6180X_ReadReg(0x0062);  // RESULT__RANGE_VAL

    // Stap 4: Reset interrupt (anders blijven metingen hangen)
    VL6180X_WriteReg(0x0015, 0x07);  // SYSTEM__INTERRUPT_CLEAR

    // Stap 5: Check foutstatus
    uint8_t rangeStatus = VL6180X_ReadReg(0x0063);  // RESULT__RANGE_STATUS
    if (rangeStatus != 0x00) {
        printf("Foutstatus: 0x%02X\r\n", rangeStatus);
    }

    return range;
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
#ifdef USE_FULL_ASSERT
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
