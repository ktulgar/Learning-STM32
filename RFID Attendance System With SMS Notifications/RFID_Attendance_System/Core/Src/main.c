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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbh_hid.h"
#include "string.h"
#include "rcc522.h"
#include "stdio.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	NONE,
	NAME,
	SURNAME,
	TEL_NUMBER
}Durum;


typedef struct{
	char name[15];
	char surname[15];
	char id[5];
	char telephone[11];
	bool presence;
}Student;



typedef enum
{
	NoNE,
	Create,
	Scan
}Button;


typedef enum {
	USB_WORKING,
	USB_NOT_WORKING
}USB;


typedef enum {
	None,
	Attendance_On,
	Attendance_Off
}Attendance;


typedef enum {
	NOTHING,
	AT_COMMAND,
	SET_TEXT_MODE_COMMAND
}GSM_Callbacks;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
char buffer[50];
uint8_t rf_test = 0;
char character;
char first[17];
char last[1] = "\"";
char keybord_buffer[15];
uint8_t keyboard_indis = 0;
uint8_t t_buffer[25] = {0};
uint8_t temp_buffer[25] = {0};
Durum durum = NONE;
Student students[5];
uint8_t number_of_students = 0;
Student student;
uint8_t status;
uint8_t CardID[5];
extern USBH_HandleTypeDef hUsbHostFS;
Button button = 0;
USB usb_status;
uint8_t time;
Attendance attendance_status;
uint8_t rfid[31] = {0};
uint8_t dataRecord[100];
uint8_t gsm_receive_buffer[50];
uint8_t at_command[] = "AT\n";
uint8_t set_text_mode[]   = "AT+CMGF=1\n";
uint8_t sms_template_beginning[] = "AT+CMGS=\"+90";
GSM_Callbacks gsm_callback;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
  if(USBH_HID_GetDeviceType(phost) == HID_KEYBOARD)
  {
	  HID_KEYBD_Info_TypeDef *keyboard;
	  keyboard = USBH_HID_GetKeybdInfo(phost);
	  character = USBH_HID_GetASCIICode(keyboard);
	  if(character != 0)
	  {
		  uint8_t total = 8;
		  keybord_buffer[keyboard_indis] = character;
		  strcat(temp_buffer,first);
		  strcat(temp_buffer,keybord_buffer);
		  keyboard_indis++;
		  total += keyboard_indis;
		  strcat(temp_buffer,last);
		  total++;
		  temp_buffer[total] = 0xFF;
		  temp_buffer[total + 1] = 0xFF;
		  temp_buffer[total + 2] = 0xFF;
		  total += 3;
		  memcpy(t_buffer,temp_buffer,strlen(temp_buffer));
		  memset(temp_buffer,0,strlen(temp_buffer));
		  HAL_UART_Transmit_DMA(&huart4, t_buffer, total);
	  }
  }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{


	if(huart->Instance == USART3)
	{
		switch(gsm_callback)
		{
		case NOTHING:
			break;
		case AT_COMMAND:
			gsm_callback = SET_TEXT_MODE_COMMAND;
			HAL_UART_Transmit_DMA(&huart3, set_text_mode, 10);
			break;
		case SET_TEXT_MODE_COMMAND:
			break;
		}

		 memset(buffer,0,strlen(gsm_receive_buffer));
	     HAL_UARTEx_ReceiveToIdle_DMA(&huart3, gsm_receive_buffer, 50);
		 __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

	}



	else if(huart->Instance == UART4)
	{
			switch(durum)
			{
			case NAME:
				memset(student.name,0,strlen(student.name));
				strcpy(student.name,keybord_buffer);
				break;
			case SURNAME:
				memset(student.surname,0,strlen(student.surname));
				strcpy(student.surname,keybord_buffer);
				break;
			case TEL_NUMBER:
					memset(student.telephone,0,strlen(student.telephone));
					strcpy(student.telephone,keybord_buffer);
					break;
			}

			if(strcmp(buffer,"name") == 0)
			{
				memset(student.name,0,strlen(student.name));
				memset(first,0,strlen(first));
				strcpy(first,"t4.txt=\"");
				memset(keybord_buffer,0,strlen(keybord_buffer));
				keyboard_indis = 0;
				durum = NAME;
			}
			else if(strcmp(buffer,"surname") == 0)
			{
				memset(student.surname,0,strlen(student.surname));
				memset(first,0,strlen(first));
				strcpy(first,"t5.txt=\"");
				memset(keybord_buffer,0,strlen(keybord_buffer));
				keyboard_indis = 0;
				durum = SURNAME;
			}
			else if(strcmp(buffer,"Tel Number") == 0)
			{
				memset(student.telephone,0,strlen(student.telephone));
				memset(first,0,strlen(first));
				strcpy(first,"t6.txt=\"");
				memset(keybord_buffer,0,strlen(keybord_buffer));
				keyboard_indis = 0;
				durum = TEL_NUMBER;
			}
			else if(strcmp(buffer,"Create") == 0)
			{
				button = Create;
			}
			else if(strcmp(buffer,"Scan") == 0)
			{
				button = Scan;
			}

			else if(strcmp(buffer,"Start Attendance") == 0)
			{
				attendance_status = Attendance_On;
			}

			else if(strcmp(buffer,"Stop Attendance") == 0)
			{
				attendance_status = Attendance_Off;
			}
			memset(buffer,0,strlen(buffer));
			HAL_UARTEx_ReceiveToIdle_DMA(&huart4, buffer, 50);
			__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
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
  MX_DMA_Init();
//  MX_USB_HOST_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, buffer, 50);
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, gsm_receive_buffer, 50);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

 gsm_callback = AT_COMMAND;
 HAL_UART_Transmit_DMA(&huart3, at_command, 3);
 MFRC522_Init();
 MX_USB_HOST_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    if(button == Scan)
    {
    	if(usb_status == USB_WORKING)
    	{
    		USBH_Stop(&hUsbHostFS);
    		usb_status = USB_NOT_WORKING;
    	}

        status = MFRC522_Request(PICC_REQIDL, CardID);	//MFRC522_Request(0x26, str)
    	status = MFRC522_Anticoll(CardID);//Take a collision, look up 5 bytes
    	    if (status == MI_OK)
    		  {
    	    	rf_test = 1;
    	    	student.id[0] = CardID[0];
    	    	student.id[1] = CardID[1];
    	    	student.id[2] = CardID[2];
    	    	student.id[3] = CardID[3];
    	    	student.id[4] = CardID[4];


    	        sprintf(rfid,"t7.txt=\"%03d-%03d-%03d-%03d-%03d\"",student.id[0],student.id[1],student.id[2],student.id[3],student.id[4]);

    	    	rfid[28] = 0xFF;
    	    	rfid[29] = 0xFF;
    	    	rfid[30] = 0xFF;

    	    	HAL_UART_Transmit_DMA(&huart4, rfid, 31);
    	    	button = NoNE;
    		  }

    }

    else if(button == Create)
    {
    	if(usb_status == USB_NOT_WORKING)
    	{
    		memcpy(students[number_of_students].name,student.name,strlen(student.name));
    		memcpy(students[number_of_students].surname,student.surname,strlen(student.surname));
    		memcpy(students[number_of_students].telephone,student.telephone,strlen(student.telephone));
    		memcpy(students[number_of_students].id,student.id,strlen(student.id));
    		number_of_students++;
    		USBH_Start(&hUsbHostFS);
    		usb_status = USB_WORKING;
    		button = NoNE;
    	}

    }


    if(attendance_status == Attendance_On)
    {


    	if(usb_status == USB_WORKING)
    	{
    		USBH_Stop(&hUsbHostFS);
    		usb_status = USB_NOT_WORKING;
    	}

        status = MFRC522_Request(PICC_REQIDL, CardID);	//MFRC522_Request(0x26, str)
    	status = MFRC522_Anticoll(CardID);//Take a collision, look up 5 bytes
    	if (status == MI_OK)
     {

    		for(uint8_t i = 0 ; i < number_of_students ; i++)
    		{
    			if(students[i].id[0] == CardID[0] && students[i].id[1] == CardID[1] && students[i].id[2] == CardID[2] && students[i].id[3] == CardID[3] && students[i].id[4] == CardID[4])
    				{
    				if(!students[i].presence)
    				{
    					uint8_t strLen = 0;
        				students[i].presence = true;
        				sprintf(dataRecord,"data0.insert(\"%s^%s^%03d-%03d-%03d-%03d-%03d\")",students[i].name,students[i].surname,students[i].id[0],students[i].id[1],students[i].id[2],students[i].id[3],students[i].id[4]);
        				strLen = strlen(dataRecord);
        				dataRecord[strLen]     = 0xFF;
        				dataRecord[strLen + 1] = 0xFF;
        				dataRecord[strLen + 2] = 0xFF;
        				HAL_UART_Transmit_DMA(&huart4, dataRecord, (strLen + 3));
    				}

    				}
    		}
     }

    }

    if(attendance_status == Attendance_Off)
    {
    	for(uint8_t i = 0 ; i < number_of_students ; i++)
    	{
    		if(!students[i].presence)
    		{
    			uint8_t attendance_message[80] = {0};
    			uint8_t send_sms[150] = {0};
    			strcpy(send_sms,sms_template_beginning);
    			strcat(send_sms,students[i].telephone);
    			strcat(send_sms,"\"\n");
    			sprintf(attendance_message,"%s %s did not come to class today. Regards ...",students[i].name,students[i].surname);
    			strcat(send_sms,attendance_message);
    			uint8_t strLen = strlen(send_sms);
    			send_sms[strLen] = 0x1A;
    			HAL_UART_Transmit(&huart3, send_sms, (strLen + 1), 100);
    			HAL_Delay(5000);
    		}
    	}

    	attendance_status = None;

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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
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
