#ifndef __FLASH_H__
#define __FLASH_H__

#include "stdint.h"

#define FLASH_PACKET_SIZE 32
#define FLASH_LENGTH 0x755555

void FLASH_Init();
void FLASH_Push(uint8_t *payload);
uint8_t FLASH_Pop(uint8_t *payload);

#endif