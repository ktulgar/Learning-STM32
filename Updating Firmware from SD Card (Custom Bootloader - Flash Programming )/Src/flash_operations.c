/*
 * flash_operations.c
 *
 *  Created on: Aug 2, 2023
 *      Author: PC
 */

#include "flash_operations.h"


// Standard Programming
void write_into_flash(uint32_t starting_address,uint32_t *data,uint16_t len) {

	int index = 0;
	while(index < len) {
		while(FLASH->SR & (1 << 16)); // Check that no Flash main memory operation is ongoing
		FLASH->SR |= 0xC3FA;
		FLASH->CR |= (1 << 0);       // Flash programming enabled
		* ((__IO uint32_t *) starting_address) = data[index];
		* ((__IO uint32_t *) (starting_address + 4)) =  data[++index];
		starting_address += 4;
		while(FLASH->SR & (1 << 16)); // Check that no Flash main memory operation is ongoing
		FLASH->CR &= ~(1 << 0);         // Flash programming disabled
	  }

}

void lock_flash() {
	FLASH->CR |= (1 << 31);
}

void unlock_flash() {
	  FLASH->KEYR = 0x45670123;
	  FLASH->KEYR =	0xCDEF89AB;
}

void disable_flash() {
	RCC->AHB1ENR &= ~(1 << 8);           // Flash memory interface clock disable
}

void enable_flash() {
	  RCC->AHB1ENR |= (1 << 8);           // Flash memory interface clock enable
}

void erase_page(uint16_t pageNum) {

	while(FLASH->SR & (1 << 16));        // Check that no Flash memory operation is ongoing
	FLASH->SR |= 0xC3FA;                 // Check and clear all error programming flags
	FLASH->CR |= (1 << 1);               // page erase enabled


	if(pageNum < 256) {
		 FLASH->CR &= ~(1 << 11);             // Bank 1 is selected for page erase
		 FLASH->CR &= ~(0xFF << 3);
		 FLASH->CR |= (pageNum << 3);
	}
	else {
		 FLASH->CR |= (1 << 11);             // Bank 2 is selected for page erase
		 pageNum -= 256;
		 FLASH->CR &= ~(0xFF << 3);
		 FLASH->CR |= (pageNum << 3);

	}
	FLASH->CR |= (1 << 16);             // This bit triggers an erase operation when set.
	while(FLASH->SR & (1 << 16));       // Wait for the BSY bit to be cleared
	FLASH->CR &= ~(1 << 1);             // page erase disabled
}


void erase_bank(uint8_t bankNum) {

	while(FLASH->SR & (1 << 16));       // Check that no Flash memory operation is ongoing by checking the BSY bit
	FLASH->SR |= 0xC3FA;
	if(bankNum == 1) {
		FLASH->CR |= (1 << 2);          // Bank 1 Mass erase
	}
	else {
		FLASH->CR |= (1 << 15);         // Bank 2 Mass erase
	}
	FLASH->CR |= (1 << 16);             // This bit triggers an erase operation when set.
	while(FLASH->SR & (1 << 16));       // Wait for the BSY bit to be cleared
	FLASH->CR &=  ~(1 << 15);
	FLASH->CR &=  ~(1 << 2);
}
