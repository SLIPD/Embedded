#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "stdint.h"
#include "stdbool.h"

typedef struct
{
	
	char* message;
	uint8_t position, length;
	bool topLine;
	bool enabled;
	
} DISPLAY_Message;

void DISPLAY_Init();

void DISPLAY_InitMessage(DISPLAY_Message *msg);
void DISPLAY_SetMessage(DISPLAY_Message *msg);
void DISPLAY_ClearLine(bool topLine);

void DISPLAY_Clear();

void DISPLAY_Update();

#endif