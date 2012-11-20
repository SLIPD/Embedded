
#include "tasks.h"
#include "system.h"

#include <string.h>
#include <stdbool.h>

#include "efm32_gpio.h"

/* variables */
static volatile bool rdy = false;

/* prototypes */
bool display_send_byte(uint8_t byte1, uint8_t byte2);

/* functions */
void display_init_task_entrypoint()
{
	
	// init reset pin
	GPIO_PinModeSet(gpioPortD,10,gpioModePushPull,0);
	GPIO_PinModeSet(gpioPortD, 15, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(gpioPortD, 14, gpioModeWiredAnd, 1);
	
	TRACE("setting up screen\n");
	I2C0_Init();
	
	int i;
	for (i = 0; i < 100000; i++);
	GPIO->P[gpioPortD].DOUT &= (1 << 10);
	for (i = 0; i < 100000; i++);
	
	TRACE("screen init'd\n");
	
	display_send_byte(0x00,0x38);
	display_send_byte(0x00,0x39);
	display_send_byte(0x00,0x14);
	display_send_byte(0x00,0x74);
	display_send_byte(0x00,0x54);
	display_send_byte(0x00,0x6F);
	display_send_byte(0x00,0x0C);
	display_send_byte(0x00,0x01);
	
	TRACE("SCREEN SETUP OK\n");
	
	rdy = true;
	
}

void display_write(char *msg)
{
	while(!rdy);
	I2C0_Transfer(0x7C,0x40,true,msg,strlen(msg));
}

bool display_send_byte(uint8_t control, uint8_t data)
{
	return I2C0_Transfer(0x7C,control,true,&data,1);
}
