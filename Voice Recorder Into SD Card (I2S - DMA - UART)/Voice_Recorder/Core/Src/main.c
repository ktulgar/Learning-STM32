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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	UNKNOWN,
	HALF_COMPLETED,
	FULL_COMPLETED
} CallBack_Result_t;

typedef enum {
	NOT_RECORDING,
	START_RECORDING,
	CURRENTLY_RECORDING,
	PAUSED_RECORDING,
	RESUME_RECORDING,
	STOP_RECORDING
}Recording_Status_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2S_DMA_LEN 25000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2S2_Init(uint32_t sample_rate);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FATFS       FatFs;                //Fatfs handle
FIL         fil;                  //File handle
FRESULT     fres;                 //Result after operations

uint32_t fread_size = 0;
uint32_t data_size  = 0;

CallBack_Result_t  callback_result;
Recording_Status_t recording_status;

int16_t recording[I2S_DMA_LEN];

uint8_t instruction[50];

char file_name[20] =  {0};

uint32_t Sample_Rate;
uint8_t test = 0;

uint32_t file_Size = 0;

void wav_header(char name[],uint32_t sample_Rate)
{
	  fres = f_mount(&FatFs, "", 1);
	  fres = f_open(&fil, name, FA_OPEN_ALWAYS | FA_WRITE);

	  uint8_t ChunkID[] = "RIFF";
	  fres = f_write(&fil,ChunkID,4,(UINT *) fread_size);

	  fres = f_lseek(&fil, 8);   // It will be filled after recording got completed.

	  uint8_t Format[] = "WAVE";
	  fres = f_write(&fil,Format,4,(UINT *) fread_size);

	  uint8_t Subchunk1ID[] = "fmt ";
	  fres = f_write(&fil,Subchunk1ID,4,(UINT *) fread_size);

	  uint32_t Subchunk1Size = 16;
	  fres = f_write(&fil,&Subchunk1Size,4,(UINT *) fread_size);

	  uint16_t AudioFormat = 1;
	  fres = f_write(&fil,&AudioFormat,2,(UINT *) fread_size);

	  uint16_t NumChannels = 2;
	  fres = f_write(&fil,&NumChannels,2,(UINT *) fread_size);

	  uint32_t SampleRate = sample_Rate;
	  fres = f_write(&fil,&SampleRate,4,(UINT *) fread_size);

	  uint16_t BitsPerSample = 16;
	  uint32_t ByteRate = (SampleRate * NumChannels  * BitsPerSample)/8;
	  fres = f_write(&fil,&ByteRate,4,(UINT *) fread_size);

	  uint16_t BlockAlign = (NumChannels * BitsPerSample)/8;

	  fres = f_write(&fil,&BlockAlign,2,(UINT *) fread_size);
	  fres = f_write(&fil,&BitsPerSample,2,(UINT *) fread_size);

	  uint8_t Subchunk2ID[] = "data";
	  fres = f_write(&fil,Subchunk2ID,4,(UINT *) fread_size);

	  fres = f_lseek(&fil, 44);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	test++;


	if(Size == 4)
	{
		recording_status = STOP_RECORDING;
	}

	else if(Size == 5)
	{
		recording_status = PAUSED_RECORDING;
	}

	else if(Size == 6)
	{
		recording_status = RESUME_RECORDING;
	}

	else
	{
		sscanf((const char *) instruction,"%d %s",(int *) &Sample_Rate ,file_name);
		recording_status = START_RECORDING;
	}

    memset(instruction,0,Size);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, instruction, 50);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, instruction, 50);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  if(recording_status == START_RECORDING)
	  {
		  recording_status =  CURRENTLY_RECORDING;
		  strcat(file_name,".wav");
		  MX_I2S2_Init(Sample_Rate);
		  wav_header(file_name,Sample_Rate);
		  memset(file_name,0,strlen(file_name));
		  HAL_I2S_Receive_DMA(&hi2s2,(uint16_t *) recording, I2S_DMA_LEN);
	  }


	  if(recording_status == PAUSED_RECORDING)
	  {
		  HAL_I2S_DMAPause(&hi2s2);
		  recording_status = NOT_RECORDING;
	  }

	  if(recording_status == RESUME_RECORDING)
	  {
		  recording_status = CURRENTLY_RECORDING;
		  HAL_I2S_DMAResume(&hi2s2);
	  }

	  if(recording_status == CURRENTLY_RECORDING)
	  {
		  if(callback_result == HALF_COMPLETED)
		  {
			  // some gain
			  for(uint16_t i = 0 ; i < I2S_DMA_LEN/2 ; i++)
			  {
				  recording[i] *= 7;
			  }
			  f_write(&fil , recording, I2S_DMA_LEN, (UINT *)fread_size);
			  callback_result = UNKNOWN;
		  }

		  if(callback_result == FULL_COMPLETED)
		  {
			  // some gain
			  for(uint16_t i = I2S_DMA_LEN/2 ; i < I2S_DMA_LEN ; i++)
			  {
				  recording[i] *= 7;
			  }
			  f_write(&fil, &recording[12500], I2S_DMA_LEN, (UINT *)fread_size);
			  callback_result = UNKNOWN;
		  }

	  }

	  if(recording_status == STOP_RECORDING)
	  {
		HAL_I2S_DMAStop(&hi2s2);
		uint16_t BitsPerSample = 16;
		uint16_t NumChannels = 2;
		uint32_t Subchunk2Size = (data_size * NumChannels * BitsPerSample)/8;
		f_lseek(&fil, 40);
		f_write(&fil,&data_size,4,(UINT *) fread_size);
		f_lseek(&fil, 4);
		uint32_t ChunkSize = Subchunk2Size + 36;
		f_write(&fil,&ChunkSize,4,(UINT *) fread_size);

		f_close(&fil);
		data_size = 0;
		recording_status = NOT_RECORDING;
      }


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(uint32_t sample_rate)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = sample_rate;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	callback_result = HALF_COMPLETED;
}
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	callback_result = FULL_COMPLETED;
	data_size += I2S_DMA_LEN*2;
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
