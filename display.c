#include "display.h"

#include "efm32_cmu.h"
#include "efm32_gpio.h"
#include "efm32_i2c.h"
#include "efm32_timer.h"

#include "stdlib.h"
#include "string.h"
#include <stdarg.h>

#include "led.h"

/* prototypes */
I2C_TransferReturn_TypeDef DISPLAY_i2cTransfer(I2C_TransferSeq_TypeDef *seq);
void DISPLAY_sendByte(uint8_t control, uint8_t data);
void DISPLAY_sendPayload(uint8_t control, uint8_t *data, uint8_t len);
void DISPLAY_receiveByte(uint8_t control, uint8_t *data);
void DISPLAY_receivePayload(uint8_t control, uint8_t *data, uint8_t len);
void DISPLAY_SetPosition(bool topLine, uint8_t pos);
void DISPLAY_updateLine(DISPLAY_Message *line);

/* variables */
DISPLAY_Message 
line1 =
{
	.enabled = false,
	.position = 0,
	.message = NULL,
}, 
line2 = 
{
	.enabled = false,
	.position = 0,
	.message = NULL,
};
bool displayUpdateRequired = false;

/* functions */
void TIMER0_IRQHandler()
{

	// update screen
	displayUpdateRequired = true;
	DISPLAY_Update();

	TIMER_IntClear(TIMER0, TIMER_IF_OF);

}

void DISPLAY_updateLine(DISPLAY_Message *line)
{

	char blank[16];
	memset(blank,0,16);

	if (line->enabled == false)
	{

		DISPLAY_SetPosition(line->topLine,0);
		DISPLAY_sendPayload(0x40,(uint8_t*)blank,16);

		return;

	}

	if (line->length < 16)
	{

		DISPLAY_SetPosition(line->topLine,0);
		DISPLAY_sendPayload(0x40,(uint8_t*)line->message,line->length);

	}
	else
	{

		uint8_t charsRemaining = line->length - line->position;

	if (charsRemaining == 0)
	{
		line->position = 0;
		charsRemaining = line->length;
	}
	if (charsRemaining > 16)
	{
		charsRemaining = 16;
	}

	memcpy(blank,(uint8_t*)(line->message + line->position),charsRemaining);

	DISPLAY_SetPosition(line->topLine,0);
	DISPLAY_sendPayload(0x40,(uint8_t*)blank,16);

	line->position++;

	}
}

void DISPLAY_Update()
{

	if (displayUpdateRequired)
	{

		DISPLAY_updateLine(&line1);
		DISPLAY_updateLine(&line2);

	}

	displayUpdateRequired = false;

}

void DISPLAY_Init()
{
	// set up screen
	DISPLAY_sendByte(0x00,0x38);
	DISPLAY_sendByte(0x00,0x39);
	DISPLAY_sendByte(0x00,0x14);
	DISPLAY_sendByte(0x00,0x74);
	DISPLAY_sendByte(0x00,0x54);
	DISPLAY_sendByte(0x00,0x6F);
	DISPLAY_sendByte(0x00,0x0C);
	DISPLAY_sendByte(0x00,0x01);

	// init timer
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = true, 
		.debugRun   = true, 
		.prescale   = timerPrescale512, 
		.clkSel     = timerClkSelHFPerClk, 
		.fallAction = timerInputActionNone, 
		.riseAction = timerInputActionNone, 
		.mode       = timerModeUp, 
		.dmaClrAct  = false,
		.quadModeX4 = false, 
		.oneShot    = false, 
		.sync       = false, 
	};

	/* Enable overflow interrupt */
	TIMER_IntEnable(TIMER0, TIMER_IF_OF);

	/* Set TIMER Top value */
	// TIMER_TopSet(TIMER0, CMU_ClockFreqGet(cmuClock_TIMER0));
	TIMER_TopSet(TIMER0, 0xAAAA);

	/* Configure TIMER */
	TIMER_Init(TIMER0, &timerInit);

}

void DISPLAY_InitMessage(DISPLAY_Message *msg)
{
	msg->message = NULL;
	msg->position = 0;
	msg->topLine = true;
	msg->enabled = true;
}

void DISPLAY_SetMessage(DISPLAY_Message *msg)
{

	msg->length = strlen(msg->message);

	if (msg->topLine)
		memcpy(&line1,msg,sizeof(DISPLAY_Message));
	else
		memcpy(&line2,msg,sizeof(DISPLAY_Message));

	displayUpdateRequired = true;

}

void DISPLAY_ClearLine(bool topLine)
{
	if (topLine)
		line1.enabled = false;
	else
		line2.enabled = false;
}

void DISPLAY_Clear()
{
	DISPLAY_ClearLine(true);
	DISPLAY_ClearLine(false);
}

void DISPLAY_sendByte(uint8_t control, uint8_t data)
{

	I2C_TransferSeq_TypeDef seq;

	seq.addr = 0x7C;      // Parameter Address
	seq.flags = I2C_FLAG_WRITE_WRITE;     // Indicate combined write / read seq 

	seq.buf[0].data = &control;
	seq.buf[0].len = 1;
	seq.buf[1].data = &data;
	seq.buf[1].len = 1;

	DISPLAY_i2cTransfer(&seq);

}

void DISPLAY_sendPayload(uint8_t control, uint8_t *data, uint8_t len)
{

	I2C_TransferSeq_TypeDef seq;

	seq.addr = 0x7C;      // Parameter Address
	seq.flags = I2C_FLAG_WRITE_WRITE;     // Indicate combined write / read seq 

	seq.buf[0].data = &control;
	seq.buf[0].len = 1;
	seq.buf[1].data = data;
	seq.buf[1].len = len;

	DISPLAY_i2cTransfer(&seq);

}

void DISPLAY_receiveByte(uint8_t control, uint8_t *data)
{

	I2C_TransferSeq_TypeDef seq;

	seq.addr = 0x7D;      // Parameter Address
	seq.flags = I2C_FLAG_WRITE_READ;     // Indicate combined write / read seq 

	seq.buf[0].data = &control;
	seq.buf[0].len = 1;
	seq.buf[1].data = data;
	seq.buf[1].len = 1;

	DISPLAY_i2cTransfer(&seq);

}

void DISPLAY_receivePayload(uint8_t control, uint8_t *data, uint8_t len)
{

	I2C_TransferSeq_TypeDef seq;

	seq.addr = 0x7D;      // Parameter Address
	seq.flags = I2C_FLAG_WRITE_READ;     // Indicate combined write / read seq 

	seq.buf[0].data = &control;
	seq.buf[0].len = 1;
	seq.buf[1].data = data;
	seq.buf[1].len = len;

	DISPLAY_i2cTransfer(&seq);

}

void DISPLAY_SetPosition(bool topLine, uint8_t pos)
{

	pos |= 0x80;
	if (!topLine)
		pos |= 0x40;

	DISPLAY_sendByte(0x00,pos);

}

I2C_TransferReturn_TypeDef DISPLAY_i2cTransfer(I2C_TransferSeq_TypeDef *seq)
{
	I2C_TransferReturn_TypeDef ret;

	// Do a polled transfer 
	ret = I2C_TransferInit(I2C0, seq);
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C0);
	}

	if (ret == i2cTransferNack)
	{
		LED_On(RED);
	}
	else
	{
		LED_Off(RED);
	}

	return(ret);

}
