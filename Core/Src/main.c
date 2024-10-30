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
#include <stdio.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SENSORS 4
#define IMU_DATA_SIZE 31
#define PACKET_SIZE 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint32_t sensorValues[NUM_SENSORS];
//uint8_t packetBuffer[PACKET_SIZE];
//nt16_t pressureSensors[4];

uint8_t  IMU1array[IMU_DATA_SIZE];
uint8_t  IMU2array[IMU_DATA_SIZE];
uint8_t  IMU3array[IMU_DATA_SIZE];

char alpha[8] ={0};
char beta[8] ={0};
char gamm[8] ={0};

double roll1, pitch1, yaw1;
double roll2, pitch2, yaw2;
double roll3, pitch3, yaw3;

double alpha_value = 0.2;
double prevSensorValues[4] = {0};

double roll1_prev = 0, pitch1_prev = 0, yaw1_prev = 0;
double roll2_prev = 0, pitch2_prev = 0, yaw2_prev = 0;
double roll3_prev = 0, pitch3_prev = 0, yaw3_prev = 0;

double accelX1 = 0, accelY1 = 0, accelZ1 = 0;
double accelX2 = 0, accelY2 = 0, accelZ2 = 0;
double accelX3 = 0, accelY3 = 0, accelZ3 = 0;

double prevAccelX1 = 0, prevAccelX2 = 0, prevAccelX3 = 0;
double prevAccelY1 = 0, prevAccelY2 = 0, prevAccelY3 = 0;
double prevAccelZ1 = 0, prevAccelZ2 = 0, prevAccelZ3 = 0;

double gyroX1 = 0, gyroY1 = 0, gyroZ1 = 0;
double gyroX2 = 0, gyroY2 = 0, gyroZ2 = 0;
double gyroX3 = 0, gyroY3 = 0, gyroZ3 = 0;

double prevGyroX1 = 0, prevGyroX2 = 0, prevGyroX3 = 0;
double prevGyroY1 = 0, prevGyroY2 = 0, prevGyroY3 = 0;
double prevGyroZ1 = 0, prevGyroZ2 = 0, prevGyroZ3 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void Read_Pressure_Data(void);
void Read_All_IMU_Data(void);
int Get_IMU_Data(UART_HandleTypeDef* huart, char* IMUarray, double* roll, double* pitch, double* yaw, double* accelX, double* accelY, double* accelZ, double* gyroX, double* gyroY, double* gyroZ);
double ApplyLowPassFilter(double currentValue, double *prevValue);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim9){
		Read_Pressure_Data();

		HAL_UART_Receive_DMA(&huart1, IMU1array, IMU_DATA_SIZE);
		HAL_UART_Receive_DMA(&huart2, IMU2array, IMU_DATA_SIZE);
		HAL_UART_Receive_DMA(&huart6, IMU3array, IMU_DATA_SIZE);

		Read_All_IMU_Data();

		//SendPacket();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // IMU1 데이터 수신 및 파싱
    	roll1 = ApplyLowPassFilter(roll1, &roll1_prev);
		pitch1 = ApplyLowPassFilter(pitch1, &pitch1_prev);
		yaw1 = ApplyLowPassFilter(yaw1, &yaw1_prev);
		HAL_UART_Receive_DMA(&huart1, IMU1array, IMU_DATA_SIZE);
    }
    /*else if (huart->Instance == USART2) {
        // IMU2 데이터 수신 및 파싱
        Get_IMU_Data(&huart2, IMU2array, &roll2, &pitch2, &yaw2, &accelX2, &accelY2, &accelZ2, &gyroX2, &gyroY2, &gyroZ2);
        HAL_UART_Receive_DMA(&huart2, (uint8_t*)IMU2array, IMU_DATA_SIZE); // DMA 수신 재시작
    }
    else if (huart->Instance == USART6) {
        // IMU3 데이터 수신 및 파싱
        Get_IMU_Data(&huart6, IMU3array, &roll3, &pitch3, &yaw3, &accelX3, &accelY3, &accelZ3, &gyroX3, &gyroY3, &gyroZ3);
        HAL_UART_Receive_DMA(&huart6, (uint8_t*)IMU3array, IMU_DATA_SIZE); // DMA 수신 재시작
    }
    */
}



/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 수신 완료 시 데이터 파싱
        Parse_IMU_Data(IMU1array, &roll1, &pitch1, &yaw1, &accelX1, &accelY1, &accelZ1, &gyroX1, &gyroY1, &gyroZ1);
        HAL_UART_Receive_DMA(&huart1, (uint8_t*)IMU1array, IMU_DATA_SIZE); // 재시작
    }
}
*/


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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM9_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_UART_Receive_DMA(&huart1, (uint8_t*)IMU1array, IMU_DATA_SIZE);
  HAL_UART_Receive_DMA(&huart2, (uint8_t*)IMU2array, IMU_DATA_SIZE);
  HAL_UART_Receive_DMA(&huart6, (uint8_t*)IMU3array, IMU_DATA_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_BRK_TIM9_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* OTG_FS_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  for(int i=0;i < NUM_SENSORS; i++){
	  sConfig.Channel = ADC_CHANNEL_0 + i;
	  sConfig.Rank = i + 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  }
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 839;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 100-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

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
  __HAL_RCC_GPIOC_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Read_Pressure_Data(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    for (int i = 0; i < NUM_SENSORS; i++) {
        // ADC 채널을 각 센서 핀에 맞게 설정 (IN0, IN1, IN4, IN5)
        switch(i) {
            case 0: sConfig.Channel = ADC_CHANNEL_0; break; // IN0
            case 1: sConfig.Channel = ADC_CHANNEL_1; break; // IN1
            case 2: sConfig.Channel = ADC_CHANNEL_4; break; // IN4
            case 3: sConfig.Channel = ADC_CHANNEL_5; break; // IN5
        }

        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }

        if (HAL_ADC_Start(&hadc1) == HAL_OK) {
            if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
            	double rawValue = HAL_ADC_GetValue(&hadc1);
            	sensorValues[i] = ApplyLowPassFilter(rawValue, &prevSensorValues[i]);
            } else if (i == 0){
            	sensorValues[0] -= 2000;
            } else {
                Error_Handler();
            }
            HAL_ADC_Stop(&hadc1);
        } else {
            Error_Handler();
        }
    }
}




int Get_IMU_Data(UART_HandleTypeDef* huart, char* IMUarray, double* roll, double* pitch, double* yaw, double* accelX, double* accelY, double* accelZ, double* gyroX, double* gyroY, double* gyroZ) {
    unsigned char star = '*';
    unsigned char receivedChar;
    int cnt0 = 0, cnt1 = 0, cnt2 = 0, cnt3 = 0, cnt4 = 0, cnt5 = 0, cnt6 = 0;
    char* pos;


    // UART에서 한 글자씩 받아서 '*'인지 확인
    while (1) {
    	HAL_UART_Receive(huart, &receivedChar, 1, HAL_MAX_DELAY);
		if (receivedChar == star) {
			// '*' 문자를 받으면 IMU 데이터 수신 시작
			HAL_UART_Receive_DMA(huart, (uint8_t*)IMUarray, IMU_DATA_SIZE);
			break;
		}
	}

    for (int i = 1; i < IMU_DATA_SIZE; i++) {
		if (IMUarray[i] == ',') {
			if (cnt0 == 0) cnt1 = i + 1;
			else if (cnt0 == 1) cnt2 = i + 1;
			else if (cnt0 == 2) cnt3 = i + 1;
			else if (cnt0 == 3) cnt4 = i + 1;
			else if (cnt0 == 4) cnt5 = i + 1;
			cnt0++;
		} else {
			switch (cnt0) {
				case 0: alpha[i - 1] = IMUarray[i]; break;  // Roll
				case 1: beta[i - cnt1] = IMUarray[i]; break;  // Pitch
				case 2: gamm[i - cnt2] = IMUarray[i]; break;  // Yaw
				case 3: accelX[i - cnt3] = IMUarray[i]; break;  // Accel X
				case 4: accelY[i - cnt4] = IMUarray[i]; break;  // Accel Y
				case 5: accelZ[i - cnt5] = IMUarray[i]; break;  // Accel Z
				case 6: gyroX[i - cnt6] = IMUarray[i]; break;  // Gyro X
			}
		}
		if (IMUarray[i] == '\r') {
			cnt6 = i + 1;
			break;
		}
	}
    // 문자열이 짧은 경우 0으로 채움
       for (int j = cnt1 - 2; j < 7; j++) alpha[j] = '0';
       for (int k = cnt2 - (cnt1); k < 8; k++) beta[k - 1] = '0';
       for (int l = cnt3 - (cnt2); l < 8; l++) gamm[l - 1] = '0';
       for (int m = cnt4 - (cnt3); m < 8; m++) accelX[m - 1] = '0';
       for (int n = cnt5 - (cnt4); n < 8; n++) accelY[n - 1] = '0';
       for (int o = cnt6 - (cnt5); o < 8; o++) accelZ[o - 1] = '0';

       // 문자열을 double로 변환
       *roll = strtod(alpha, &pos);
       *pitch = strtod(beta, &pos);
       *yaw = strtod(gamm, &pos);
       *accelX = strtod(accelX, &pos);
       *accelY = strtod(accelY, &pos);
       *accelZ = strtod(accelZ, &pos);
       *gyroX = strtod(gyroX, &pos);
       *gyroY = strtod(gyroY, &pos);
       *gyroZ = strtod(gyroZ, &pos);
}

void Read_All_IMU_Data(void) {
    // IMU 1 (UART1)
    if (Get_IMU_Data(&huart1, IMU1array, &roll1, &pitch1, &yaw1, &accelX1, &accelY1, &accelZ1, &gyroX1, &gyroY1, &gyroZ1)) {
        roll1 = ApplyLowPassFilter(roll1, &roll1_prev);
        pitch1 = ApplyLowPassFilter(pitch1, &pitch1_prev);
        yaw1 = ApplyLowPassFilter(yaw1, &yaw1_prev);

        accelX1 = ApplyLowPassFilter(accelX1, &prevAccelX1);  // 가속도 X 축 필터 적용
        accelY1 = ApplyLowPassFilter(accelY1, &prevAccelY1);  // 가속도 Y 축 필터 적용
        accelZ1 = ApplyLowPassFilter(accelZ1, &prevAccelZ1);  // 가속도 Z 축 필터 적용

        gyroX1 = ApplyLowPassFilter(gyroX1, &prevGyroX1);  // 각속도 X 축 필터 적용
        gyroY1 = ApplyLowPassFilter(gyroY1, &prevGyroY1);  // 각속도 Y 축 필터 적용
        gyroZ1 = ApplyLowPassFilter(gyroZ1, &prevGyroZ1);  // 각속도 Z 축 필터 적용
    }
/*
    // IMU 2 (UART2)
    if (Get_IMU_Data(&huart2, IMU2array, &roll2, &pitch2, &yaw2, &accelX2, &accelY2, &accelZ2, &gyroX2, &gyroY2, &gyroZ2)) {
        roll2 = ApplyLowPassFilter(roll2, &roll2_prev);
        pitch2 = ApplyLowPassFilter(pitch2, &pitch2_prev);
        yaw2 = ApplyLowPassFilter(yaw2, &yaw2_prev);

        accelX2 = ApplyLowPassFilter(accelX2, &prevAccelX2);
        accelY2 = ApplyLowPassFilter(accelY2, &prevAccelY2);
        accelZ2 = ApplyLowPassFilter(accelZ2, &prevAccelZ2);

        gyroX2 = ApplyLowPassFilter(gyroX2, &prevGyroX2);
        gyroY2 = ApplyLowPassFilter(gyroY2, &prevGyroY2);
        gyroZ2 = ApplyLowPassFilter(gyroZ2, &prevGyroZ2);
    }

    // IMU 3 (UART6)
    if (Get_IMU_Data(&huart6, IMU3array, &roll3, &pitch3, &yaw3, &accelX3, &accelY3, &accelZ3, &gyroX3, &gyroY3, &gyroZ3)) {
        roll3 = ApplyLowPassFilter(roll3, &roll3_prev);
        pitch3 = ApplyLowPassFilter(pitch3, &pitch2_prev);
        yaw3 = ApplyLowPassFilter(yaw3, &yaw3_prev);

        accelX3 = ApplyLowPassFilter(accelX3, &prevAccelX3);
        accelY3 = ApplyLowPassFilter(accelY3, &prevAccelY3);
        accelZ3 = ApplyLowPassFilter(accelZ3, &prevAccelZ3);

        gyroX3 = ApplyLowPassFilter(gyroX3, &prevGyroX3);
        gyroY3 = ApplyLowPassFilter(gyroY3, &prevGyroY3);
        gyroZ3 = ApplyLowPassFilter(gyroZ3, &prevGyroZ3);
    }
    */
}
double ApplyLowPassFilter(double currentValue, double *prevValue){
	*prevValue = alpha_value * currentValue + (1.0 - alpha_value) * (*prevValue);
	return *prevValue;
}


// 패킷 빌드 함수
/*void BuildPacket(void) {
    memset(packetBuffer, 0, PACKET_SIZE);

    // 패킷 시작 신호 (예: 0xA0, 0xA1로 시작)
    packetBuffer[0] = 0xA0;
    packetBuffer[1] = 0xA1;

    // 압력 센서 데이터 추가 (8바이트: 각 센서당 2바이트)
    for (int i = 0; i < 4; i++) {
        packetBuffer[2 + i * 2] = (sensorValues[i] >> 8) & 0xFF;
        packetBuffer[3 + i * 2] = sensorValues[i] & 0xFF;
    }

    // IMU 1 가속도 데이터 추가 (6바이트: 각 축당 2바이트)
    packetBuffer[10] = (int16_t)(accelX1 * 100) >> 8 & 0xFF;
    packetBuffer[11] = (int16_t)(accelX1 * 100) & 0xFF;
    packetBuffer[12] = (int16_t)(accelY1 * 100) >> 8 & 0xFF;
    packetBuffer[13] = (int16_t)(accelY1 * 100) & 0xFF;
    packetBuffer[14] = (int16_t)(accelZ1 * 100) >> 8 & 0xFF;
    packetBuffer[15] = (int16_t)(accelZ1 * 100) & 0xFF;

    // IMU 1 각속도 데이터 추가 (6바이트: 각 축당 2바이트)
    packetBuffer[16] = (int16_t)(gyroX1 * 100) >> 8 & 0xFF;
    packetBuffer[17] = (int16_t)(gyroX1 * 100) & 0xFF;
    packetBuffer[18] = (int16_t)(gyroY1 * 100) >> 8 & 0xFF;
    packetBuffer[19] = (int16_t)(gyroY1 * 100) & 0xFF;
    packetBuffer[20] = (int16_t)(gyroZ1 * 100) >> 8 & 0xFF;
    packetBuffer[21] = (int16_t)(gyroZ1 * 100) & 0xFF;

    // IMU 1 각도 데이터 추가 (6바이트: roll, pitch, yaw 각도당 2바이트)
    packetBuffer[22] = (int16_t)(roll1 * 100) >> 8 & 0xFF;
    packetBuffer[23] = (int16_t)(roll1 * 100) & 0xFF;
    packetBuffer[24] = (int16_t)(pitch1 * 100) >> 8 & 0xFF;
    packetBuffer[25] = (int16_t)(pitch1 * 100) & 0xFF;
    packetBuffer[26] = (int16_t)(yaw1 * 100) >> 8 & 0xFF;
    packetBuffer[27] = (int16_t)(yaw1 * 100) & 0xFF;

    // IMU 2 가속도 데이터 추가 (6바이트)
    packetBuffer[28] = (int16_t)(accelX2 * 100) >> 8 & 0xFF;
    packetBuffer[29] = (int16_t)(accelX2 * 100) & 0xFF;
    packetBuffer[30] = (int16_t)(accelY2 * 100) >> 8 & 0xFF;
    packetBuffer[31] = (int16_t)(accelY2 * 100) & 0xFF;
    packetBuffer[32] = (int16_t)(accelZ2 * 100) >> 8 & 0xFF;
    packetBuffer[33] = (int16_t)(accelZ2 * 100) & 0xFF;

    // IMU 2 각속도 데이터 추가 (6바이트)
    packetBuffer[34] = (int16_t)(gyroX2 * 100) >> 8 & 0xFF;
    packetBuffer[35] = (int16_t)(gyroX2 * 100) & 0xFF;
    packetBuffer[36] = (int16_t)(gyroY2 * 100) >> 8 & 0xFF;
    packetBuffer[37] = (int16_t)(gyroY2 * 100) & 0xFF;
    packetBuffer[38] = (int16_t)(gyroZ2 * 100) >> 8 & 0xFF;
    packetBuffer[39] = (int16_t)(gyroZ2 * 100) & 0xFF;

    // IMU 2 각도 데이터 추가 (6바이트)
    packetBuffer[40] = (int16_t)(roll2 * 100) >> 8 & 0xFF;
    packetBuffer[41] = (int16_t)(roll2 * 100) & 0xFF;
    packetBuffer[42] = (int16_t)(pitch2 * 100) >> 8 & 0xFF;
    packetBuffer[43] = (int16_t)(pitch2 * 100) & 0xFF;
    packetBuffer[44] = (int16_t)(yaw2 * 100) >> 8 & 0xFF;
    packetBuffer[45] = (int16_t)(yaw2 * 100) & 0xFF;

    // IMU 3 가속도 데이터 추가 (6바이트)
    packetBuffer[46] = (int16_t)(accelX3 * 100) >> 8 & 0xFF;
    packetBuffer[47] = (int16_t)(accelX3 * 100) & 0xFF;
    packetBuffer[48] = (int16_t)(accelY3 * 100) >> 8 & 0xFF;
    packetBuffer[49] = (int16_t)(accelY3 * 100) & 0xFF;
    packetBuffer[50] = (int16_t)(accelZ3 * 100) >> 8 & 0xFF;
    packetBuffer[51] = (int16_t)(accelZ3 * 100) & 0xFF;

    // IMU 3 각속도 데이터 추가 (6바이트)
    packetBuffer[52] = (int16_t)(gyroX3 * 100) >> 8 & 0xFF;
    packetBuffer[53] = (int16_t)(gyroX3 * 100) & 0xFF;
    packetBuffer[54] = (int16_t)(gyroY3 * 100) >> 8 & 0xFF;
    packetBuffer[55] = (int16_t)(gyroY3 * 100) & 0xFF;
    packetBuffer[56] = (int16_t)(gyroZ3 * 100) >> 8 & 0xFF;
    packetBuffer[57] = (int16_t)(gyroZ3 * 100) & 0xFF;

    // IMU 3 각도 데이터 추가 (6바이트)
    packetBuffer[58] = (int16_t)(roll3 * 100) >> 8 & 0xFF;
    packetBuffer[59] = (int16_t)(roll3 * 100) & 0xFF;
    packetBuffer[60] = (int16_t)(pitch3 * 100) >> 8 & 0xFF;
    packetBuffer[61] = (int16_t)(pitch3 * 100) & 0xFF;
    packetBuffer[62] = (int16_t)(yaw3 * 100) >> 8 & 0xFF;
    packetBuffer[63] = (int16_t)(yaw3 * 100) & 0xFF;

    // 패킷 종료 신호 (예: 0xB0, 0xB1로 끝)
    packetBuffer[64] = 0xB0;
    packetBuffer[65] = 0xB1;
}

void SendPacket(void){
    char packet[PACKET_SIZE];  // 패킷 크기를 적절히 정의합니다.
    int packetIndex = 0;

    // Step 1: 압력 센서 값을 패킷에 추가
    for (int i = 0; i < NUM_SENSORS; i++) {
        memcpy(&packet[packetIndex], &sensorValues[i], sizeof(sensorValues[i]));
        packetIndex += sizeof(sensorValues[i]);
    }

    memcpy(&packet[packetIndex], &roll1, sizeof(roll1));
	packetIndex += sizeof(roll1);
	memcpy(&packet[packetIndex], &pitch1, sizeof(pitch1));
	packetIndex += sizeof(pitch1);
	memcpy(&packet[packetIndex], &yaw1, sizeof(yaw1));
	packetIndex += sizeof(yaw1);
	memcpy(&packet[packetIndex], &accelX1, sizeof(accelX1));
	packetIndex += sizeof(accelX1);
	memcpy(&packet[packetIndex], &accelY1, sizeof(accelY1));
	packetIndex += sizeof(accelY1);
	memcpy(&packet[packetIndex], &accelZ1, sizeof(accelZ1));
	packetIndex += sizeof(accelZ1);
	memcpy(&packet[packetIndex], &gyroX1, sizeof(gyroX1));
	packetIndex += sizeof(gyroX1);
	memcpy(&packet[packetIndex], &gyroY1, sizeof(gyroY1));
	packetIndex += sizeof(gyroY1);
	memcpy(&packet[packetIndex], &gyroZ1, sizeof(gyroZ1));
	packetIndex += sizeof(gyroZ1);

    memcpy(&packet[packetIndex], &roll2, sizeof(roll2));
	packetIndex += sizeof(roll2);
	memcpy(&packet[packetIndex], &pitch2, sizeof(pitch2));
	packetIndex += sizeof(pitch2);
	memcpy(&packet[packetIndex], &yaw2, sizeof(yaw2));
	packetIndex += sizeof(yaw2);
	memcpy(&packet[packetIndex], &accelX2, sizeof(accelX2));
	packetIndex += sizeof(accelX2);
	memcpy(&packet[packetIndex], &accelY2, sizeof(accelY2));
	packetIndex += sizeof(accelY2);
	memcpy(&packet[packetIndex], &accelZ2, sizeof(accelZ2));
	packetIndex += sizeof(accelZ2);
	memcpy(&packet[packetIndex], &gyroX2, sizeof(gyroX2));
	packetIndex += sizeof(gyroX2);
	memcpy(&packet[packetIndex], &gyroY2, sizeof(gyroY2));
	packetIndex += sizeof(gyroY2);
	memcpy(&packet[packetIndex], &gyroZ2, sizeof(gyroZ2));
	packetIndex += sizeof(gyroZ2);

    memcpy(&packet[packetIndex], &roll3, sizeof(roll3));
	packetIndex += sizeof(roll3);
	memcpy(&packet[packetIndex], &pitch3, sizeof(pitch3));
	packetIndex += sizeof(pitch3);
	memcpy(&packet[packetIndex], &yaw3, sizeof(yaw3));
	packetIndex += sizeof(yaw3);
	memcpy(&packet[packetIndex], &accelX3, sizeof(accelX3));
	packetIndex += sizeof(accelX3);
	memcpy(&packet[packetIndex], &accelY3, sizeof(accelY3));https://www.ti.com/tool/TMDSEMU200-U#order-start-development
	packetIndex += sizeof(accelY3);
	memcpy(&packet[packetIndex], &accelZ3, sizeof(accelZ3));
	packetIndex += sizeof(accelZ3);
	memcpy(&packet[packetIndex], &gyroX3, sizeof(gyroX3));
	packetIndex += sizeof(gyroX3);
	memcpy(&packet[packetIndex], &gyroY3, sizeof(gyroY3));
	packetIndex += sizeof(gyroY3);
	memcpy(&packet[packetIndex], &gyroZ3, sizeof(gyroZ3));
	packetIndex += sizeof(gyroZ3);

    // Step 3: 패킷을 UART를 통해 전송
	CDC_Transmit_HS((uint8_t*)packet, packetIndex);
}
*/
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
