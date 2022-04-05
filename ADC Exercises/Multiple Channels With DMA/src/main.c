#include "stm32f4xx.h"

void initSystemClock(void);
void initGPIO(void);
void initADC(void);
void initDMA(void);
void startConversions(void);

uint8_t adcData[2];

int main(void)
{
  initSystemClock();
  initGPIO();
  initADC();
  initDMA();
  startConversions();

  while (1)
  {

  }
}


void initSystemClock(void) {

	RCC->CR |= (1 << 16);             // HSE clock enable
	while(!(RCC->CR & (1 << 17)));    // HSE clock ready flag

	RCC->PLLCFGR |= (1 << 22);        // HSE Selected as PLL Source

	RCC->PLLCFGR |= (12 << 0);         // PLLM => 12
	RCC->PLLCFGR |= (96 << 6);         // PLLN => 96
	RCC->PLLCFGR &= ~(3 << 16);        // PLLP => 2


	RCC->CR |= (1 << 24);             // Main PLL Enable
	while(!(RCC->CR & (1 << 25)))     // Main PLL clock ready flag

	RCC->CFGR |= (2 << 0);            // PLL selected as system clock
	while(!(RCC->CFGR & (1 << 3)));   // PLL used as the system clock

	RCC->CFGR |= (4 << 10);           // APB1 Prescaler = 2
	SystemCoreClockUpdate();          // Now System Clock is 100 MHz
}


void initGPIO(void) {

	RCC->AHB1ENR |= 1;              // Enable GPIOA

	GPIOA->MODER |= (3 << 2);       // GPIOA Pin 1 Analog Mode
	GPIOA->PUPDR &= ~(3 << 2);      // No Pull Up-Down

	GPIOA->MODER |= (3 << 4);       // GPIOA Pin 2 Analog Mode
	GPIOA->PUPDR &= ~(3 << 4);      // No Pull Up-Down


}

void initADC(void) {

	RCC->APB2ENR |= (1 << 8);       // Enable ADC Clock

	ADC1->CR2 |= (1 << 0);          // Turn ADC On

	                                // ADC can run at 36 MHZ at most.
    ADC->CCR |= (1 << 16);          // PCLK2 divided by 4. Now ADC clock is 25 MHz.

	ADC1->CR1 &= ~(3 << 24);
	ADC1->CR1 |= (2 << 24);         // 8 Bit Resolution

	ADC1->CR2 &= ~(1 << 11);        // Right Alignment

	ADC1->CR2 |= (1 << 1);          // Continuous Mode

	ADC1->CR1 |= (1 << 8);          // Scan Mode. We have two channels to read.

	ADC1->SQR3 |= (1 << 0);         // Channel 1
	ADC1->SQR3 |= (2 << 5);         // Channel 2
    ADC1->SQR1 |= (1 << 20);        // 2 Conversions

	ADC1->CR2  |= (1 << 8);         // Enable DMA
	ADC1->CR2  |= (1 << 9);         // DMA requests are issued as long as data are converted and DMA=1

}


void initDMA(void) {

	RCC->AHB1ENR |= (1 << 22);                      // Enable DMA2 Clock

	DMA2_Stream4->PAR  = (uint32_t) &(ADC1->DR);    // Set peripheral address where data is read from

	DMA2_Stream4->M0AR = (uint32_t) &(adcData);     // Set memory address where data is written to

	DMA2_Stream4->NDTR = 2;                         // Number of data --> 2 ADC channels - 2 data

	DMA2_Stream4->CR &= ~(7 << 25);                 // Channel 0 -> ADC1

	DMA2_Stream4->CR |= (3 << 16);                  // Very high priority

	DMA2_Stream4->CR |= (1 << 8);                   // Circular Mode

	DMA2_Stream4->CR |= (1 << 10);                  // Memory increment mode

	DMA2_Stream4->CR |= (1 << 0);                   // Stream enabled

}

void startConversions(void) {

	ADC1->CR2 |= (1 << 30);        // Start Conversions
	while(!(ADC1->SR & (1 << 4)));

}
