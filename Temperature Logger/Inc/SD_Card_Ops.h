/*
 * SD_Card_Ops.h
 *
 *  Created on: Apr 30, 2023
 *      Author: Kazım Tulgaroğlu
 */

#ifndef INC_SD_CARD_OPS_H_
#define INC_SD_CARD_OPS_H_

#include "fatfs.h"
#include "rtc.h"
#include "nextion_uart.h"

struct SD_CARD{
	FATFS       FatFs;                //Fatfs handle
	FIL         fil;                  //File handle
	FRESULT     fres;                 //Result after operations
};


void init_FS(struct SD_CARD *);
void open_file_for_reading(struct SD_CARD *);
void open_file_for_writing(struct SD_CARD *);
void get_records(struct SD_CARD *);
void add_first_record(struct SD_CARD *);
void add_record(struct SD_CARD *,struct TimeDate *,float);
void erase_records(struct SD_CARD *);
void close_the_file(struct SD_CARD *);

#endif /* INC_SD_CARD_OPS_H_ */
