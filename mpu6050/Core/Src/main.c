/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
const uint8_t mpu6050Addr = 0xD0; //endereço padrão do sensor

typedef struct leituraAcel { //estrutura para armazenar as leituras do acelerometro
	int16_t accelX;
	int16_t accelY;
	int16_t accelZ;
} leituraAcel;

typedef struct leituraGyro { //estrutura para armazenar as leituras do acelerometro
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
} leituraGyro;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void mpu6050Init(void) { //inicialização e configuração do sensor
	static const uint8_t whoAmIReg = 0x75;			// identificação do sensor
	static const uint8_t pwrMgmt1Reg = 0x6B;		// gerenciamento de consumo de energia
	static const uint8_t configReg = 0x1A;			// configurações gerais
	static const uint8_t accelConfigReg = 0x1C;	// configurações do acelerometro
	static const uint8_t gyroConfigReg = 0x1B;	// configurações do giroscópio
	static unsigned char msgErro[] = "Erro na inicializacao do MPU6050";	// mensagem para falha de identificação do sensor

	uint8_t check;
	uint8_t data;

	HAL_I2C_Mem_Read(&hi2c1, mpu6050Addr, whoAmIReg, 1, &check, 1, 1000);	// armazena o conteudo de whoAmIReg em check

	if(check == 0x68) { // espera-se que este valor seja 0x68

		data = 0x08; // este valor, neste registrador, desativa o sensor de temperatura, que não será utilizado
		HAL_I2C_Mem_Write(&hi2c1, mpu6050Addr, pwrMgmt1Reg, 1, &data, 1, 1000); // escreve o valor de data em pwrMgmtReg

		data = 0x06; // este valor, neste registrador, configura o filtro passa baixas para máxima filtragem
		HAL_I2C_Mem_Write(&hi2c1, mpu6050Addr, configReg, 1, &data, 1, 1000); // escreve o valor de data em configReg

		data = 0x00; // este valor, neste registrador, mantém suas configurações padrão
		HAL_I2C_Mem_Write(&hi2c1, mpu6050Addr, accelConfigReg, 1, &data, 1, 1000); // escreve o valor de data em accelConfigReg

		data = 0x00; // este valor, neste registrador, mantém suas configurações padrão
		HAL_I2C_Mem_Write(&hi2c1, mpu6050Addr, gyroConfigReg, 1, &data, 1, 1000); // escreve o valor de data em gyroConfigReg
	} else {
		HAL_UART_Transmit(&huart1, msgErro, sizeof(msgErro), 100); // em caso de erro, envia mensagem de erro
	}
}


void mpu6050ReadAccel(leituraAcel *leitura) { // lê medições do acelerometro
	static const uint8_t accelXoutHReg = 0x3B; // primeiro registrador com as medições
	uint8_t recData[6];	// vetor com a quantidade de valores a serem armazenados

	HAL_I2C_Mem_Read(&hi2c1, mpu6050Addr, accelXoutHReg, 1, recData, 6, 1000); // lê 6 registradores a partir de accelXoutHReg e armazena os valores em recData

	leitura->accelX = (int16_t) (recData[0] << 8 | recData [1]); // para cada eixo
	leitura->accelY = (int16_t) (recData[2] << 8 | recData [3]); // concatena os valores
	leitura->accelZ = (int16_t) (recData[4] << 8 | recData [5]); // dos respectivos registradores
}


void mpu6050ReadGyro(leituraGyro *leitura) { // lê medições do giroscópio
	static const uint8_t gyroXoutHReg = 0x43; // primeiro registrador com as medições
	uint8_t recData[6];	// vetor com a quantidade de valores a serem armazenados

	HAL_I2C_Mem_Read(&hi2c1, mpu6050Addr, gyroXoutHReg, 1, recData, 6, 1000); // lê 6 registradores a partir de gyroXoutHReg e armazena os valores em recData

	leitura->gyroX = (int16_t) (recData[0] << 8 | recData [1]); // para cada eixo
	leitura->gyroY = (int16_t) (recData[2] << 8 | recData [3]); // concatena os valores
	leitura->gyroZ = (int16_t) (recData[4] << 8 | recData [5]); // dos respectivos registradores
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t mensagem[64] = {0}; // buffer com bytes o suficiente para armazenar a mensagem
	leituraAcel leituraA;
	leituraGyro leituraG;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  mpu6050Init(); // inicializa o sensor
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1) {
    /* USER CODE END WHILE */
	  mpu6050ReadAccel(&leituraA); // lê as medições de aceleração
	  mpu6050ReadGyro(&leituraG);	 // lê as medições de velocidade angular

	  /* armazena, em mensagem, uma string formatada conforme o segundo argumento e composta pelos argumentos seguintes */
    sprintf((char *) mensagem, "%d %d %d %d %d %d\r\n", leituraA.accelX, leituraA.accelY, leituraA.accelZ, leituraG.gyroX, leituraG.gyroY, leituraG.gyroZ);
    HAL_UART_Transmit(&huart1, mensagem, sizeof(mensagem), 100); // envia mensagem pela UART1
    HAL_Delay(100); // aguarda 100 ms
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
