/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "lwip.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* ETH_CODE: add lwiperf, see comment in StartDefaultTask function */
#include "lwip/apps/lwiperf.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "usbh_hid.h"
#include "string.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	NOT_CONNECTED_TO_SERVER,
	CONNECTED_TO_SERVER
}Server_Connection_Status;



typedef enum
{
	NONE,
	NEXTION_MESSAGE_ARRIVED,
	NEXTION_SERVER_INPUT,
	NEXTION_PORT_INPUT,
	NEXTION_CONNECT_COMMAND,
	TCP_MESSAGE_ARRIVED,
	NEXTION_MESSAGE_INPUT,
	SEND_TCP_MESSAGE,
	NEXTION_GO_TO_NEXT_PAGE,
	TCP_CONNECTION_ERROR
}Condition;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */
uint8_t received_message[100];
uint8_t keybord_buffer[100];
uint8_t keyboard_indis = 0;

uint8_t error_message[] = {'t','0','.','t','x','t','+','=','\"','.','.','.','C','o','n','n','e','c','t','i','o','n',' ','L','o','s','t',' ','.','.','.','\r','\n','\"',0xFF,0xFF,0xFF};
uint8_t clear_command[] = {'t','1','.','t','x','t','=','\"','\"',0xFF,0xFF,0xFF};
uint8_t make_button_visible[] = {'v','i','s',' ','b','1',',','1',0xFF,0xFF,0xFF};

uint8_t nextion_buffer[100];

struct tcp_pcb *tcp_block;
HID_KEYBD_Info_TypeDef *keyboard;
struct pbuf *received_buffer;

uint8_t server_adress_char[20];
uint8_t port_char[20];

uint8_t server_address[20];
uint8_t port[20];

bool sent = false;
Condition previous_condition = 0;

Server_Connection_Status server_connection_status;
Condition condition;

uint8_t ip[4];
uint8_t n_tokens = 4;

char **token_arr (const char *str, int *n_tokens)
{
    char **arr = malloc(*n_tokens * sizeof *arr);
    char str2 [strlen(str) + 1];
    int i = 0;

    strcpy (str2, str);

    char *p = strtok (str2, ".");

    while (i < *n_tokens && p != NULL) {    /* check used pointers */
        arr[i] = strdup (p);
        p = strtok (NULL, ".");
        i++;
    }
    *n_tokens = i;  /* assign i to make tokes assigned available */

    return arr;
}


bool compare(uint8_t string1[], uint8_t string2[])
{
	uint8_t len = strlen((const char *) string2);
	bool result = true;
	for(uint8_t i = 0 ; i < len ; i++)
	{
		if(string1[i] != string2[i])
		{
			result = false;
			break;
		}

	}
	return result;
}

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
  if(USBH_HID_GetDeviceType(phost) == HID_KEYBOARD)
  {
	  keyboard = USBH_HID_GetKeybdInfo(phost);
	   uint8_t character = USBH_HID_GetASCIICode(keyboard);
	  if(character != 0)
	{
		 if(keyboard->keys[0] == KEY_ENTER)
		 {
			 condition = SEND_TCP_MESSAGE;
		 }
		 else
		 {
		     keybord_buffer[keyboard_indis] = character;
		     keyboard_indis++;
		     sent = false;
		     if(condition == NEXTION_SERVER_INPUT || condition == NEXTION_PORT_INPUT)
		     {
		    	 condition = condition;
		     }
		     else
		     {
		    	 condition = NEXTION_MESSAGE_INPUT;
		     }

		 }

	}

  }
}



extern USBH_HandleTypeDef hUsbHostFS;

err_t tcp_message_received_callback(void *arg, struct tcp_pcb *tpcb,struct pbuf *p, err_t err)
{
	if(err == ERR_ABRT)
	{
		condition = TCP_CONNECTION_ERROR;
	}
	else
	{
		received_buffer = p;
		condition = TCP_MESSAGE_ARRIVED;
	}

	return err;
}

err_t tcp_message_sent_callback(void *arg, struct tcp_pcb *tpcb,  u16_t len)
{
	 return 0;
}


void  tcp_connection_error_callback(void *arg, err_t err)
{
	condition = TCP_CONNECTION_ERROR;
}

 err_t tcp_connected_to_server_callback(void *arg, struct tcp_pcb *tpcb, err_t err)
 {
	 server_connection_status = CONNECTED_TO_SERVER;
	 tcp_sent(tcp_block, tcp_message_sent_callback);
	 tcp_recv(tcp_block, tcp_message_received_callback);
	 tcp_err(tpcb, tcp_connection_error_callback);
	 condition = NEXTION_GO_TO_NEXT_PAGE;
	 return err;
 }

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 uint8_t nextion_cmd[50];
 static uint8_t buffer[50];
 void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
 {
	 condition = NEXTION_MESSAGE_ARRIVED;
	 memcpy(nextion_cmd,buffer,Size);
	 memset(buffer,0,Size);
	 HAL_UARTEx_ReceiveToIdle_IT(&huart4, buffer, 50);
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */

/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */

/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_IT(&huart4, buffer, 50);
  /* ETH_CODE: fixed core synchronization
   * Release M4 core after GPIO and peripherals init
   * to avoid conflict.
   */
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_HSEM_FastTake(HSEM_ID_0);
  HAL_HSEM_Release(HSEM_ID_0,0);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN 5 */


    /* Infinite loop */
    for(;;)
    {
    	switch(condition)
{

    	case NONE:

    	{
    		break;
    	}


    	case NEXTION_MESSAGE_ARRIVED:

    	{
      	    if(compare(nextion_cmd,(uint8_t *) "Server Address"))
      	    {

      		    memset(keybord_buffer,0,keyboard_indis);
      		    memset(server_address,0,strlen((const char *)server_address));
      		    keyboard_indis = 0;
      		    sent = false;
      		    previous_condition = NEXTION_SERVER_INPUT;
      		    condition = NEXTION_SERVER_INPUT;
      	    }
      	    else if(compare(nextion_cmd,(uint8_t *) "Port Number"))
      	    {
      		    memset(keybord_buffer,0,keyboard_indis);
      		    memset(port,0,strlen((const char *) port));
      		    keyboard_indis = 0;
      		    sent = false;
      		    previous_condition = NEXTION_PORT_INPUT;
      		    condition = NEXTION_PORT_INPUT;
      	    }
      	    else if(compare(nextion_cmd,(uint8_t *) "Connect"))
      	    {
      	  	    previous_condition = NEXTION_CONNECT_COMMAND;
      		    condition = NEXTION_CONNECT_COMMAND;
      	    }
      	    else
      	    {
      	    	condition = previous_condition;
      	    }


      	    memset(nextion_cmd,0,strlen((const char *) nextion_cmd));
      	    break;
    	}


    	case NEXTION_SERVER_INPUT:

    	{
      	  if(!sent)
      	  {
      		  memset(server_adress_char,0,strlen((const char *) server_adress_char));
      		  memcpy(server_adress_char,"t1.txt=\"",8);
      		  memcpy(server_address,keybord_buffer,keyboard_indis);
      		  strcat((char *) server_adress_char, (char *)keybord_buffer);
      		  strcat((char *) server_adress_char, (char *) "\"");
      		  uint8_t total = 9 + strlen((const char *)keybord_buffer);
      		  server_adress_char[total] = 0xFF;
      		  server_adress_char[total + 1] = 0xFF;
      		  server_adress_char[total + 2] = 0xFF;
      		  HAL_UART_Transmit(&huart4, server_adress_char, strlen((const char *) server_adress_char), 100);
      		  sent =  true;
      	  }
      	  break;
    	}


    	case NEXTION_PORT_INPUT:

    	{
      	  if(!sent)
      	  {
      		  memset(port_char,0,strlen((const char *) port_char));
      		  memcpy(port_char,"t3.txt=\"",8);
      		  memcpy(port,keybord_buffer,keyboard_indis);
      		  strcat((char *) port_char,(char *) keybord_buffer);
      		  strcat((char *) port_char,(char *) "\"");
      		  uint8_t total = 9 + strlen((const char *) keybord_buffer);
      		  port_char[total] = 0xFF;
      		  port_char[total + 1] = 0xFF;
      		  port_char[total + 2] = 0xFF;
      		  HAL_UART_Transmit(&huart4, port_char, strlen((const char *) port_char), 100);
      		  sent =  true;
      	  }
      	  break;
    	}


    	case NEXTION_CONNECT_COMMAND:

    	{
      	  if(server_connection_status == NOT_CONNECTED_TO_SERVER)
      	  {


        	    char **expression = token_arr ((const char *) server_address,(int *) &n_tokens);

        	    if (expression) {       /* validate token_arr succeeded */
        	        for (int i = 0; i < n_tokens; i++) { /* n_tokens times */
        	           ip[i] = atoi(expression[i]);
        	            free (expression[i]);
        	        }
        	        free (expression);
        	    }

        	    uint16_t port_number = atoi((const char *) port);

        	    memset(keybord_buffer,0,keyboard_indis);
        	    keyboard_indis = 0;

        	    ip4_addr_t server_Address;

        	    LOCK_TCPIP_CORE();

        	    tcp_block = tcp_new();
        	    IP4_ADDR(&server_Address, ip[0], ip[1], ip[2], ip[3]);
        	    tcp_connect(tcp_block, &server_Address, port_number, tcp_connected_to_server_callback);

        	    UNLOCK_TCPIP_CORE();

        	    condition = NONE;
      	  }
      	  break;
    	}


    	case TCP_MESSAGE_ARRIVED:

    	{
  	   	    memcpy(received_message,received_buffer->payload,received_buffer->len);
  	        LOCK_TCPIP_CORE();
  	        tcp_recved(tcp_block, received_buffer->len);
  	   	    UNLOCK_TCPIP_CORE();
  	   	    size_t message_len = strlen((const char *) received_message);
  	   	    memcpy(nextion_buffer,"t0.txt+=\"",9);
  	   	    strcat((char *) nextion_buffer,(char *) received_message);
      	    strcat((char *) nextion_buffer,(char *) "\r\n\"");
 	        uint8_t total = 12 +  message_len;
 	    	nextion_buffer[total] = 0xFF;
 	    	nextion_buffer[total + 1] = 0xFF;
 	    	nextion_buffer[total + 2] = 0xFF;
 	    	HAL_UART_Transmit(&huart4, nextion_buffer, total+3,100);
  	        memset(nextion_buffer,0,total + 3);
            memset( received_message, 0, message_len);
            condition =  NONE;
            break;
    	}


    	case SEND_TCP_MESSAGE:

    	{
    		uint8_t tcp_message[100] = "STM32-> ";
			strcat((char *) tcp_message, (const char *) keybord_buffer);
    	    LOCK_TCPIP_CORE();
    	    tcp_write(tcp_block, tcp_message,( keyboard_indis + 8), TCP_WRITE_FLAG_COPY );
    	    tcp_output(tcp_block);
    	    UNLOCK_TCPIP_CORE();
    		memcpy(nextion_buffer,"t0.txt+=\"STM32-> ",17);
    		strcat((char *) nextion_buffer, (char *) keybord_buffer);
    		strcat((char *) nextion_buffer, (char *) "\r\n\"");
    		uint8_t total = 17 + keyboard_indis + 3;
    		nextion_buffer[total] = 0xFF;
    		nextion_buffer[total + 1] = 0xFF;
    		nextion_buffer[total + 2] = 0xFF;
    		HAL_UART_Transmit(&huart4, nextion_buffer, total + 3,100);
    		HAL_UART_Transmit(&huart4, clear_command , 12, 100);
    		memset(keybord_buffer,0,keyboard_indis);
    		memset(nextion_buffer,0,total + 3);
    	    keyboard_indis = 0;
    	    condition = NONE;
    	    break;
    	}


    	case NEXTION_MESSAGE_INPUT:

    	{
    		memcpy(nextion_buffer,"t1.txt=\"",8);
    		strcat((char *) nextion_buffer, (char *) keybord_buffer);
    		strcat((char *) nextion_buffer, (char *) "\"");
    		uint8_t total = 8 + keyboard_indis + 1;
    		nextion_buffer[total] = 0xFF;
    		nextion_buffer[total + 1] = 0xFF;
    		nextion_buffer[total + 2] = 0xFF;
    		HAL_UART_Transmit(&huart4, nextion_buffer, total + 3,100);
    		memset(nextion_buffer,0,total + 3);
    		condition = NONE;
    		break;
    	}


    	case NEXTION_GO_TO_NEXT_PAGE:

    	{
            HAL_UART_Transmit(&huart4, make_button_visible , 11,100);
      	    condition = NONE;
      	    break;
    	}

    	case TCP_CONNECTION_ERROR:

    	{
    		HAL_UART_Transmit(&huart4, error_message, 38, 100);
      	    condition = NONE;
      	    break;
    	}

      }

    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */

   /* init code for USB_HOST */
  MX_USB_HOST_Init();

  /* Infinite loop */
  for(;;)
  {
	  USBH_Process(&hUsbHostFS);
  }
  /* USER CODE END StartTask02 */
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30020000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512B;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
