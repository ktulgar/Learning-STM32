/*
 * Speaker.h
 *
 *  Created on: 29 Ara 2021
 *      Author: ktulgar
 */

#ifndef SRC_SPEAKER_H_
#define SRC_SPEAKER_H_

#include "stm32l4xx.h"
#include <cmath>


class Speaker {

 //Notes and Their Frequencies
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988

public:
	Speaker();
	void play();


private:
	void initTimer();
	void initGPIO();
	void delayMS(int);
	int arrValues[203];
	int ccrValues[203];
	int notesPOTC[203] = {       // Notes Of The Theme Pirates Of The Caribbean
	   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
	   NOTE_A4, NOTE_G4, NOTE_A4, 0,
	   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
	   NOTE_A4, NOTE_G4, NOTE_A4, 0,
	   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	   NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
	   NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
	   NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,
	   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	   NOTE_D5, NOTE_E5, NOTE_A4, 0,
	   NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
	   NOTE_C5, NOTE_A4, NOTE_B4, 0,
	   NOTE_A4, NOTE_A4,
	   //Repeat of first part
	   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
	   NOTE_A4, NOTE_G4, NOTE_A4, 0,
	   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
	   NOTE_A4, NOTE_G4, NOTE_A4, 0,
	   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	   NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
	   NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
	   NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,
	   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	   NOTE_D5, NOTE_E5, NOTE_A4, 0,
	   NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
	   NOTE_C5, NOTE_A4, NOTE_B4, 0,
	   //End of Repeat
	   NOTE_E5, 0, 0, NOTE_F5, 0, 0,
	   NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
	   NOTE_D5, 0, 0, NOTE_C5, 0, 0,
	   NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4,
	   NOTE_E5, 0, 0, NOTE_F5, 0, 0,
	   NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
	   NOTE_D5, 0, 0, NOTE_C5, 0, 0,
	   NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4 };
	int durations[203] = {
	  //duration of each note (in ms) Quarter Note is set to 250 ms
	  125, 125, 250, 125, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 375, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 375, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 125, 250, 125,
	  125, 125, 250, 125, 125,
	  250, 125, 250, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 375, 375,
	  250, 125,
	  //Rpeat of First Part
	  125, 125, 250, 125, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 375, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 375, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 125, 250, 125,
	  125, 125, 250, 125, 125,
	  250, 125, 250, 125,
	  125, 125, 250, 125, 125,
	  125, 125, 375, 375,
	  //End of Repeat
	  250, 125, 375, 250, 125, 375,
	  125, 125, 125, 125, 125, 125, 125, 125, 375,
	  250, 125, 375, 250, 125, 375,
	  125, 125, 125, 125, 125, 500,
	  250, 125, 375, 250, 125, 375,
	  125, 125, 125, 125, 125, 125, 125, 125, 375,
	  250, 125, 375, 250, 125, 375,
	  125, 125, 125, 125, 125, 500
	};
};

#endif /* SRC_SPEAKER_H_ */
