/*
 * flash_operations.h
 *
 *  Created on: Aug 2, 2023
 *      Author: PC
 */

#ifndef INC_FLASH_OPERATIONS_H_
#define INC_FLASH_OPERATIONS_H_


#include "stm32l4xx.h"

void disable_flash();
void enable_flash();
void lock_flash();
void unlock_flash();
void erase_bank(uint8_t bankNum);
void erase_page(uint16_t pageNum);
void write_into_flash(uint32_t starting_address,uint32_t *data,uint16_t len);

#endif /* INC_FLASH_OPERATIONS_H_ */
