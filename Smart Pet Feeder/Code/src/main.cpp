#include "HC05.h"
#include "HX711.h"
#include "HCSR04.h"
#include "DS3231.h"
#include "Speaker.h"
#include "ServoMotor.h"


extern "C" void UART4_IRQHandler(void);
extern "C" void EXTI2_IRQHandler(void);
extern "C" void TIM7_IRQHandler(void);

void initSystemClock(void);
void initTimer7(void);

enum Mode{
	NONE,
	ModeA,
	ModeB
};

HC05 bluetoothModule;
DS3231 realTimeClock;
Speaker speaker;
HCSR04 distanceSensor;
HX711 weightSensor;
ServoMotor motor;

enum Mode mode;

int gramA;
int gramB;
int weight;
int Distance;

int timeThatPassed;
bool firstAlarmStatus;

int  main()
{
    initSystemClock();
    initTimer7();
    mode = NONE;
    timeThatPassed = 0;
    firstAlarmStatus = true;

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

// This is what happens when alarm time came.
void EXTI2_IRQHandler(){


	EXTI->PR1 |= (1<<2);

	if(weightSensor.getWeight() < 5) {

		while(weightSensor.getWeight() < gramB){
			motor.startPouring();
		}
		motor.stopPouring();
		speaker.play();


		if(realTimeClock.alarms.size() == 0) {
			firstAlarmStatus = true;
			gramB = 0;
		}
		else {
			struct DateTime alarm = realTimeClock.alarms.front();
			auto iter = realTimeClock.alarms.begin();
			realTimeClock.alarms.erase(iter);
			struct DateTime currentDateTime;
		    realTimeClock.getDateTime(&currentDateTime);
			realTimeClock.setCurrentDateTime(currentDateTime);
			realTimeClock.sendAlarm(alarm);
			gramB = alarm.gram;
		}

	}

}
// I used UART as interrupt mode.Whenever a single byte of message is received i collect it in a buffer.When IDLE time
// detected i processed the message.
void UART4_IRQHandler() {


	if((UART4->ISR & (1 << 5))) {
		UART4->RQR = 1 << 3 ;
		char data;
		data = UART4->RDR;
		bluetoothModule.message.push_back(data);
	}
	if((UART4->ISR & (1 << 4))){
		   UART4->ICR = 1 << 4 ;
           // Adjusting time and date
		   if(bluetoothModule.message[0] == 'A' && bluetoothModule.message[1] == 'd'){

			   struct DateTime adjustment;

		       string currentDay = bluetoothModule.message.substr(11,2);
		       string currentMonth = bluetoothModule.message.substr(14,2);
		       string currentYear = bluetoothModule.message.substr(17,2);

		       string currentHour = bluetoothModule.message.substr(20,2);
		       string currentMinute = bluetoothModule.message.substr(23,2);
		       string currentSecond = bluetoothModule.message.substr(26,2);

               adjustment.second = atoi(currentSecond.c_str());
               adjustment.minute = atoi(currentMinute.c_str());
               adjustment.hour = atoi(currentHour.c_str());
               adjustment.day = atoi(currentDay.c_str());
               adjustment.month = atoi(currentMonth.c_str());
               adjustment.year = atoi(currentYear.c_str());

               realTimeClock.setCurrentDateTime(adjustment);

		   }
		  // Timed Mode
		  else if(bluetoothModule.message[0] == 'B') {

              if(mode != ModeA) {
    			  mode = ModeB;
    			  NVIC_EnableIRQ(EXTI2_IRQn);
    			  struct DateTime alarm;
    			  int size = bluetoothModule.message.size();

    			  string day = bluetoothModule.message.substr(2,2);
    			  string month = bluetoothModule.message.substr(5,2);
    			  string  year = bluetoothModule.message.substr(8,2);

    			  string hour = bluetoothModule.message.substr(11,2);
    			  string minute = bluetoothModule.message.substr(14,2);
    			  string second = bluetoothModule.message.substr(17,2);

    			  string stringGramB = bluetoothModule.message.substr(20,size - 20);

    			  alarm.second = atoi(second.c_str());
    			  alarm.minute = atoi(minute.c_str());
    			  alarm.hour = atoi(hour.c_str());
    			  alarm.day = atoi(day.c_str());
    			  alarm.month = atoi(month.c_str());
    			  alarm.year = atoi(year.c_str());
    			  alarm.gram = atoi(stringGramB.c_str()) - 10;


    			  if(firstAlarmStatus) {
    				  struct DateTime currentDateTime;
    				  realTimeClock.getDateTime(&currentDateTime);
    				  realTimeClock.setCurrentDateTime(currentDateTime);
    				  realTimeClock.sendAlarm(alarm);
    				  firstAlarmStatus = false;
    				  gramB = alarm.gram;
    				  }

    			 else {
    				  realTimeClock.alarms.push_back(alarm);
    			  }
              }

		  }
		  // Resetting System
		  else if(bluetoothModule.message[0] == 'R') {
			  if(mode == ModeA) {
				  NVIC_DisableIRQ(TIM7_IRQn);
				  timeThatPassed = 0;
				  mode = NONE;
			  }
			  else if((mode == ModeB)) {
				  NVIC_DisableIRQ(EXTI2_IRQn); // Disable interrupt
				  // Delete alarms
				  realTimeClock.deleteAlarms();
				  realTimeClock.disableAlarmMode();
 				  mode = NONE;
 				  firstAlarmStatus = true;
			  }
		  }
		   // Simple Mode
		  else if(bluetoothModule.message[0] == 'A') {
			  if(mode != ModeB) {
				  mode = ModeA;
				  NVIC_EnableIRQ(TIM7_IRQn);
	              int size = bluetoothModule.message.size();
				  string stringGramA = bluetoothModule.message.substr(2,size - 2);
				  gramA = atoi(stringGramA.c_str()) - 10;
			  }
		  }

		   bluetoothModule.message.clear();
		   bluetoothModule.message.shrink_to_fit();
	}

}

void initTimer7() {

	RCC->APB1ENR1 |= (1 << 5);
	TIM7->PSC = 39999;
	TIM7->ARR = 2000;
	TIM7->DIER |= (1 << 0);
	TIM7->CR1 |=  (1 << 0);

}

// Every second it enters interrupt.
void TIM7_IRQHandler(void) {

    TIM7->SR &= ~(1 << 0);
    // When distance between pet and sensor is lower than 15 cm , if meanwhile its food bowl is empty
    // It starts to count
	if(distanceSensor.getDistance() < 15 && weightSensor.getWeight() < 5) {
		timeThatPassed++;
		// If it waits 7 seconds in front of feeder under this circumstances , feeder gives food
		if(timeThatPassed >= 7) {
			while(weightSensor.getWeight() < gramA) {
				motor.startPouring();
			}
			motor.stopPouring();
		}
	}
	else {
		timeThatPassed = 0;
	}

}


