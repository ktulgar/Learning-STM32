#include "main.h"
#include "string.h"


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void initUART(void);
void USART1_IRQHandler(void);
void initPWM(void);

int currentIndex = 0;
int position = 150;
char message[50];

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  initUART();
  initPWM();

  while (1)
  {

  }

}


void initUART(void){

	RCC->APB2ENR |= (1 << 4);   // Enable UART1 Clock
	RCC->AHB1ENR |= (1 << 0);   // Enable GPIOA Clock

	GPIOA->MODER |= (2 << 20);  // Alternated Function Mode selected for pin 10
	GPIOA->AFR[1] |= (7 << 8);  // Pin10 => AF7

	USART1->CR1 &= ~(1 << 15);  // OverSampling is 16
	USART1->CR2 &= ~(3 << 12);  // 1 Stop bit
	USART1->CR1 &= ~(1 << 10);  // No Parity Control
	USART1->CR1 &= ~(1 << 12);  // 8 Data bits
	USART1->BRR = 0x1B2;        // Baud Rate is 115200
	USART1->CR1 |= (1 << 4);    // IDLE detection interrupt
	USART1->CR1 |= (1 << 5);    // RX  interrupt
	USART1->CR1 |= (1 << 2);    // Enable receiver
	NVIC_EnableIRQ(USART1_IRQn);
	USART1->CR1 |= (1 << 13);   // Enable Peripheral
}


void USART1_IRQHandler(void) {

	// When new data is available, put it into array
	// Increase the index
	if(USART1->SR & (1 << 5)) {
		message[currentIndex] = USART1->DR;
		currentIndex++;
	}

	// It means reception of message is over.
	else if(USART1->SR & (1 << 4)) {
		if(strcmp(message,"Turn Left") == 0) {
			if(position < 250) {
				position+=2;
				TIM2->CCR1 = position;
			}
		}
		else if(strcmp(message,"Turn Right") == 0) {
			if(position > 50) {
				position-=2;
				TIM2->CCR1 = position;
			}
		}
		memset(message,0,currentIndex+1);
		currentIndex = 0;
	}
}


void initPWM(void) {

	RCC->APB1ENR |= (1 << 0);   // Enable Timer 2
	RCC->AHB1ENR |= (1 << 0);   // Enable GPIOA Clock

	GPIOA->MODER |= (2 << 0);   // Alternated Function mode selected for pin 0
	GPIOA->AFR[0] |= (1 << 0);  // Pin 0 = AF1

	TIM2->CCER |= (1 << 0);     // Activate Channel 1
	TIM2->CCMR1 &= ~(3 << 0);   // Configure channel 1 as output
	TIM2->CCMR1 |= (6 << 4);    // PWM Mode1 selected
	TIM2->ARR = 1999;
	TIM2->PSC = 999;
	TIM2->CCR1 = 150;    // Starting is middle position.
	TIM2->CR1 |= 1;      // start the counter

}

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
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
