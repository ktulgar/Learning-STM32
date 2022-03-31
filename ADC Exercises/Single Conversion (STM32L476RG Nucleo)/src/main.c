#include "stm32l4xx.h"


void initSystemClock(void);
void initGPIO(void);
void initADC(void);
int getADCValue(void);

uint8_t adc;

int main(void)
{

 initSystemClock();
 initADC();
 initGPIO();

  while (1)
  {
     adc = getADCValue();
  }
}



void initSystemClock(void) {

    FLASH->ACR &= ~(7 << 0);
	FLASH->ACR |= (4 << 0);       // Latency => Four wait states

	RCC->CR &= ~(1 << 24);        // Main PLL Disable
	while(RCC->CR & (1 << 24));

	RCC->PLLCFGR &= ~(3 << 0);
	RCC->PLLCFGR |= (1 << 0);     // MSI clock selected as PLL

	RCC->PLLCFGR &= ~(1 << 12);   // PLLN = 0 (just resetting)

	RCC->PLLCFGR &= ~(7 << 4);    // PLLM = 1
	RCC->PLLCFGR |= (40 << 8);    // PLLN = 40
	RCC->PLLCFGR &= ~(3 << 25);   // PLLR = 2
	RCC->PLLCFGR |= (1 << 24);    // Enable PLLR

	RCC->CR |= (1 << 24);           // Main PLL Enable
	while(!(RCC->CR & (1 << 25)));  // Wait until PLL is ready

	RCC->CFGR &= ~(3 << 0);
	RCC->CFGR |= (3 << 0);          // PLL selected as system clock

	while(!( RCC->CFGR & (3 << 2)));   // Wait until PLL used as system clock

	RCC->CFGR &= ~(1 << 7);  // AHB prescaler = 1
	RCC->CFGR &= ~(1 << 10); // APB low-speed prescaler (APB1) = 1
	RCC->CFGR &= ~(1 << 13); // APB high-speed prescaler (APB2) = 1

	 // Final Step
	SystemCoreClockUpdate();  // Update the System Clock
}

void initGPIO(void){
	RCC->AHB2ENR |= (1 << 0); // Enable GPIOA
	GPIOA->ASCR |= (1 << 1);  // Connect analog switch to the ADC input Pin A1
}

void initADC(void){

	RCC->AHB2ENR |= (1 << 13);       // Enable ADC Clock
	RCC->CCIPR |= (3 << 28);         // System clock selected as ADCs clock
	ADC1->CR &= ~(1 << 29);          // ADC not in Deep-power down
	ADC1->CR |= (1 << 28);           // ADC Voltage regulator enabled
	ADC1->CR |= (1 << 0);            // Start ADC
	while(!(ADC1->ISR & (1 << 0)));  // Wait until it gets started
	ADC1->CFGR |= (2 << 3);          // 8-bit data resulotion
	ADC1->CFGR &= ~(1 << 5);         // Right alignment
	ADC1->CFGR &= ~(1 << 13);        // Single Conversion
	ADC1->SQR1 |= (6 << 6);          // Channel 6 selected

}

int getADCValue(void) {
	  ADC1->CR |= (1 << 2);               // Start Conversion
	  while(!(ADC1->ISR &= (1 << 2)));    // Wait until conversion gets completed
	  int adc = ADC1->DR;                 // Get ADC value
	  ADC1->CR |= (1 << 4);               // Stop conversion
	  return adc;                         // Return the value
}
