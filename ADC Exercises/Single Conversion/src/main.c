#include "stm32f4xx.h"

enum Resulotion{
	SIX_BIT = 3,
	EIGHT_BIT = 2,
	TEN_BIT = 1,
	TWELVE_BIT = 0
};

enum Allignment{
	RIGHT,LEFT
};

void initSystemClock(void);
void initGPIO(void);
void initADC(enum Resulotion,enum Allignment);
int getADCValue();


uint8_t adcValue;

int main(void)
{

  initSystemClock();
  initGPIO();
  initADC(EIGHT_BIT,RIGHT);


  while (1)
  {
		adcValue = getADCValue();
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
	GPIOA->PUPDR &= ~(3 << 0);      // No Pull Up-Down

}

void initADC(enum Resulotion resulotion,enum Allignment allignment){

	RCC->APB2ENR |= (1 << 8);       // Enable ADC Clock

	ADC1->CR2 |= (1 << 0);          // Turn ADC On

                                    // ADC can run at 36 MHZ at most.
	ADC->CCR |= (1 << 16);          // PCLK2 divided by 4. Now ADC clock is 25 MHz.

	// Resulotion
	ADC1->CR1 &= ~(3 << 24);
	ADC1->CR1 |= (resulotion << 24);

	// Allignment
	ADC1->CR2 &= ~(1 << 11);
	ADC1->CR2 |= (allignment << 11);

	// Channel 1
	ADC1->SQR3 = (1 << 0);

	// Sampling Time
	ADC1->SMPR2 = (4 << 3) ;         // 84 Cycles
}



int getADCValue(){

	ADC1->CR2 |= (1 << 30);           // Start Conversion
	while(!(ADC1->SR & (1 << 4)));    // Wait until conversion gets started
	while(!(ADC1->SR & (1 << 1)));    // Wait until conversion gets completed
	return ADC1->DR;                  // Get value

}
