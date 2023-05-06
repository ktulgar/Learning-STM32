/*
 * SD_Card_Ops.c
 *
 *  Created on: Apr 30, 2023
 *      Author: Kazım Tulgaroğlu
 */


#include "fatfs.h"
#include "sd_card_ops.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"

void init_FS(struct SD_CARD *sd_card){
	f_mount(&sd_card->FatFs, "", 1);
}

void add_first_record(struct SD_CARD *sd_card) {
	char message[] = "xx.xx.xx xx:xx xx.xx\n";
	f_puts(message,&sd_card->fil);
}

void add_record(struct SD_CARD * sd_card,struct TimeDate *currentTimeDate,float temperature) {
	char message[22];
	sprintf(message,"%02d.%02d.%02d %02d:%02d %.2f\n",currentTimeDate->day,currentTimeDate->month,currentTimeDate->year,currentTimeDate->hour,currentTimeDate->minute,temperature);
	f_puts(message,&sd_card->fil);
}

void close_the_file(struct SD_CARD *sd_card) {
	f_close(&sd_card->fil);
}


void open_file_for_writing(struct SD_CARD *sd_card) {
	f_open(&sd_card->fil, "recordings.txt", FA_WRITE | FA_OPEN_APPEND);
}

void open_file_for_reading(struct SD_CARD *sd_card) {
    f_open(&sd_card->fil, "recordings.txt", FA_READ | FA_OPEN_ALWAYS);
}

void get_records(struct SD_CARD *sd_card){
	   while(!(f_eof(&sd_card->fil) != 0)) {

		   char record[22];
	       f_gets(record,22,&sd_card->fil);
	       char nextion_command[36];
	       char first_part[] = "t0.txt+=\"";
	       char last_part[] = "\r\n\"";

	       strcat(nextion_command,first_part);
	       strcat(nextion_command,record);
	       strcat(nextion_command,last_part);

	       nextion_command[33] = '\xff';
	       nextion_command[34] = '\xff';
	       nextion_command[35] = '\xff';

	       sendMessage(nextion_command,36);
	       memset(nextion_command,0,36);
	   }
}


void erase_records(struct SD_CARD *sd_card) {
	f_open(&sd_card->fil,"recordings.txt",FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
	f_truncate(&sd_card->fil);
	close_the_file(sd_card);
}
