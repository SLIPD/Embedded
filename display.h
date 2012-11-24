#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "stdint.h"
#include "stdbool.h"
#include "efm32_i2c.h"

#define HARD_LEFT       "<<<            "
#define SOFT_LEFT       "   <<<         "
#define STRAIGHT        "      ^^^      "
#define SOFT_RIGHT      "         >>>   "
#define HARD_RIGHT      "            >>>"

typedef struct
{
	
	char* message;
	uint8_t position, length;
	bool topLine;
	bool enabled;
        bool scroll;
	
} DISPLAY_Message;




/* prototypes */
void DISPLAY_dir(uint8_t dir);
void DISPLAY_clearScreen();
void DISPLAY_returnHome();
void DISPLAY_entryModeSet(uint8_t id, uint8_t s);

void DISPLAY_Clear();
void DISPLAY_ClearLine(bool topLine);

void DISPLAY_SetPosition(bool topLine, uint8_t pos);
void DISPLAY_InitMessage(DISPLAY_Message *msg);
void DISPLAY_SetMessage(DISPLAY_Message *msg);

void DISPLAY_updateLine(DISPLAY_Message *line);
void DISPLAY_Update();
void DISPLAY_Init();

void DISPLAY_sendByte(uint8_t control, uint8_t data);
void DISPLAY_sendPayload(uint8_t control, uint8_t *data, uint8_t len);
void DISPLAY_receiveByte(uint8_t control, uint8_t *data);
void DISPLAY_receivePayload(uint8_t control, uint8_t *data, uint8_t len);
I2C_TransferReturn_TypeDef DISPLAY_i2cTransfer(I2C_TransferSeq_TypeDef *seq);

#endif