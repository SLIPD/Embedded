#ifndef __I2C_H__
#define __I2C_H__

#include <stdbool.h>
#include <stdint.h>

#include "efm32_i2c.h"

#include "led.h"

#define i2c_transfer_t I2C_TransferSeq_TypeDef

static inline void I2C0_Init()
{

	// init i2c
	I2C0->ROUTE |= I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | I2C_ROUTE_LOCATION_LOC3;

	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

	I2C_Init(I2C0, &i2cInit);
	
}

static inline bool I2C0_Transfer(uint8_t addr, uint8_t control, bool write, uint8_t *data, uint16_t size)
{
	
	i2c_transfer_t transfer;
	transfer.addr = addr;
	if (write)
		transfer.flags = I2C_FLAG_WRITE_WRITE;
	else
		transfer.flags = I2C_FLAG_WRITE_READ;
	
	transfer.buf[0].data = &control;
	transfer.buf[0].len = 1;
	transfer.buf[1].data = data;
	transfer.buf[1].len = size;
	
	I2C_TransferReturn_TypeDef ret;
	
  ret = I2C_TransferInit(I2C0, &transfer);
  while (ret == i2cTransferInProgress)
  {
    ret = I2C_Transfer(I2C0);
  }
	
	if (ret == i2cTransferNack)
	{
		TRACE("I2C0 NACK\n");
	}
	
	return (ret == i2cTransferDone);
	
}

#endif