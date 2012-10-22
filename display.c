#include "display.h"

#include "efm32.h"

#include "efm32_i2c.h"
#include "i2cdrv.h"

/* prototypes */
void read(uint8_t instruction, uint8_t *data, uint8_t len);
void write(uint8_t instruction, uint8_t *data, uint8_t len);

int showCursor()
{
	
	I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;
  
  uint8_t control = 0x00;
  uint8_t data = 0x0F;

  seq.addr = DISPLAY_ADDRESS_READ;      // Parameter Address
  seq.flags = I2C_FLAG_WRITE_READ;     // Indicate combined write / read seq 
  /* Select register to be read */
  // Format: Reg ID, Value
  // Length must be specified
  seq.buf[0].data = &control;
  seq.buf[0].len = 1;
  seq.buf[1].data = &data;
  seq.buf[1].len = 1;
 
  ret = I2CDRV_Transfer(&seq);
  if (ret != i2cTransferDone)
  {
    return((int)ret);
  }

  return(0);
	
}

void write(uint8_t instruction, uint8_t *data, uint8_t len)
{
	
}

void DISPLAY_Init()
{
	
	showCursor();
	
}

void DISPLAY_SetText(char *text, uint8_t line);
void DISPALY_Clear();
