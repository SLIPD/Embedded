#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "stdint.h"

#define DISPLAY_ADDRESS_READ 0x7D
#define DISPLAY_ADDRESS_WRITE 0x7C

void DISPLAY_Init();
void DISPLAY_SetText(char *text, uint8_t line);
void DISPALY_Clear();

#endif