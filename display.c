#include "display.h"

#include "efm32_cmu.h"
#include "efm32_gpio.h"
#include "efm32_i2c.h"
#include "efm32_timer.h"
#include "efm32_int.h"

#include "stdlib.h"
#include "string.h"
#include <stdarg.h>
#include "trace.h"

#include "led.h"

/* variables */
DISPLAY_Message 
line1 =
{
	.enabled = false,
	.position = 0,
	.message = NULL,
        .scroll = true,
}, 
line2 = 
{
	.enabled = false,
	.position = 0,
	.message = NULL,
        .scroll = true,
};

bool displayUpdateRequired = false;

/* functions */

// Draw directions on top line of screen
// dir: 0 hard left
// dir: 1 soft left
// dir: 2 straight
// dir: 3 soft right
// dir: 4 hard right
void DISPLAY_dir(uint8_t dir)
{
    // Clear top line
    char blank[16];
    memset(blank,0,16);
    DISPLAY_SetPosition(true, 0);
    DISPLAY_sendPayload(0x40,(uint8_t*)blank,16);
    
    // Initialise and zero str
    char *str;
    
    // Set direction 
    switch(dir)
    {
        case 0:
            str = HARD_LEFT;
            break;
            
        case 1:
            str = SOFT_LEFT;
            break; 
            
        case 2:
            str = STRAIGHT;
            break;   
            
        case 3:
            str = SOFT_RIGHT;
            break;
            
        case 4:
            str = HARD_RIGHT;
            break;
            
        default:
            break;
    }
     
    // Set line1 attributes
    if (dir >= 0 && dir <= 5)
    {
            line1.enabled = true;
            line1.scroll = false;
            line1.message = str;
            line1.length = 15;
            displayUpdateRequired = true;
    }
}

// Screen built in functions

// Clear screen of characters
void DISPLAY_clearScreen()
{
    DISPLAY_sendByte(0x00,0x01);
}

// Return cursor to original status (left edge, top line)
void DISPLAY_returnHome()
{
    DISPLAY_sendByte(0x00,0x02);
}

// Sets moving direction of cursor and display
// id == 1: cursor moves to the right, DDRAM++
// id == 0: cursor moves to the left, DDRAM--
// s == 1 : shift entire display according to id value 
// s == 0 : no display shift 
// BROKEN!!!
void DISPLAY_entryModeSet(uint8_t id, uint8_t s)
{
    uint8_t sendByte = 0x00;
    
    if (id > 0x00)
    {
        TRACE("id true\n");
        id = 0x02;
    }
    
    if (s > 0x00)
    {
        TRACE("s true\n");
        s = 0x01;
    }
    
    sendByte =  (0x04 | id | s );
    DISPLAY_sendByte(0x00, sendByte);
}

// De enable lines
void DISPLAY_ClearLine(bool topLine)
{
	if (topLine)
		line1.enabled = false;
	else
		line2.enabled = false;
}


// Clear the screen
void DISPLAY_Clear()
{
	DISPLAY_ClearLine(true);
	DISPLAY_ClearLine(false);
}

// Set position of char on line (top or bottom)
void DISPLAY_SetPosition(bool topLine, uint8_t pos)
{
	pos |= 0x80;
	if (!topLine)
		pos |= 0x40;

	DISPLAY_sendByte(0x00,pos);
}

// Initialise DISPLAY_message
void DISPLAY_InitMessage(DISPLAY_Message *msg)
{
    
	msg->message = NULL;
	msg->position = 0;
	msg->topLine = true;
	msg->enabled = true;
        msg->scroll = true;
        
}

// Write message to screen
void DISPLAY_SetMessage(DISPLAY_Message *msg)
{
    
	msg->length = strlen(msg->message);
     
        INT_Disable();
        
        // Create and init new line (one line on screen is 16 chars)
	char blank[16];
	memset(blank,0,16);
        
	if (msg->topLine)
        {
                DISPLAY_SetPosition(msg->topLine,0);
		DISPLAY_sendPayload(0x40,(uint8_t*)blank,16);
		memcpy(&line1,msg,sizeof(DISPLAY_Message));
        }
	else
        {       DISPLAY_SetPosition(msg->topLine,0);
		DISPLAY_sendPayload(0x40,(uint8_t*)blank,16);
		memcpy(&line2,msg,sizeof(DISPLAY_Message));
        }

	displayUpdateRequired = true;
        
        INT_Enable();
        
}

void DISPLAY_updateLine(DISPLAY_Message *line)
{

        // Create and init new line (one line on screen is 16 chars)
	char blank[16];
	memset(blank,0,16);

        // Clear line
	if (line->enabled == false)
	{
                DISPLAY_SetPosition(line->topLine,0);
		DISPLAY_sendPayload(0x40,(uint8_t*)blank,16);
		return;
	}

        // Write line to screen
	if (line->length <= 16)
	{
		DISPLAY_SetPosition(line->topLine,0);
		DISPLAY_sendPayload(0x40, (uint8_t*)line->message, line->length);
	}
	else
	{
                // Get number of leftover chars to write to screen on line
		uint8_t charsRemaining = line->length - line->position;

                // Reset line
                if (charsRemaining == 0)
                {
                        line->position = 0;
                        charsRemaining = line->length;
                }
                // Only write 16 chars at a time
                if (charsRemaining > 16)
                {
                        charsRemaining = 16;
                }

                // Copy <=16 chars to blank
                memcpy(blank, (uint8_t*)(line->message + line->position), charsRemaining);

                // Set line
                DISPLAY_SetPosition(line->topLine,0);
                // Write line to screen
                DISPLAY_sendPayload(0x40,(uint8_t*)blank,16);
                
                if (line->scroll)
                {
                    line->position++;
                }
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

void DISPLAY_MessageWrite(DISPLAY_Message *msg)
{
    msg->enabled = true;
    DISPLAY_SetMessage(msg);
    DISPLAY_Update();
}

void DISPLAY_Init()
{
    
	// set up screen (in documentation, 3.3v)
	DISPLAY_sendByte(0x00,0x38);
	DISPLAY_sendByte(0x00,0x39);
	DISPLAY_sendByte(0x00,0x14);
	DISPLAY_sendByte(0x00,0x74);
	DISPLAY_sendByte(0x00,0x54);
	DISPLAY_sendByte(0x00,0x6F);
	DISPLAY_sendByte(0x00,0x0C);
	DISPLAY_sendByte(0x00,0x01);
	
	DISPLAY_clearScreen();
	DISPLAY_returnHome();

}

// Write data over I2C from screen as single byte
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

// Write data over I2C from screen as payload (multiple bytes)
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

// Read data over I2C from screen as single byte
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

// Read data over I2C from screen as payload (multiple bytes)
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
// Transfer bits via I2C to screen
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