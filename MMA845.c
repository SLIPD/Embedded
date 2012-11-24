#include <stddef.h>
#include "efm32.h"
#include "efm32_usart.h"
#include "efm32_emu.h"
#include "efm32_i2c.h"
#include "MMA845XQ.h"

int MMA845X_RegisterGet(I2C_TypeDef *i2c,
                         uint8_t addr,
                         uint8_t reg,
                         uint8_t *buf,
						 uint8_t len)
{
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t regid[1];
  //uint8_t data[1];

  seq.addr = addr;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select register to be read */
  regid[0] = reg;
  seq.buf[0].data = regid;
  seq.buf[0].len = 1;
  seq.buf[1].data = buf;
  seq.buf[1].len = len;
 
  ret = I2CDRV_Transfer(&seq);
  if (ret != i2cTransferDone)
  {
    return((int)ret);
  }

  return(0);
}

void MMARegWrite(uint8_t reg, uint8_t val) {
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t regid[1];
  //uint8_t data[1];

  seq.addr = MMA8451Q_ADDR;
  seq.flags = I2C_FLAG_WRITE_WRITE;
  /* Select register to be read */
  regid[0] = reg;
  seq.buf[0].data = regid;
  seq.buf[0].len = 1;
  seq.buf[1].data = &val;
  seq.buf[1].len = 1;
 
  ret = I2CDRV_Transfer(&seq);
}

uint8_t MMARegRead(uint8_t reg) {
  
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t regid[1];
  uint8_t data[1];

  seq.addr = MMA8451Q_ADDR;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select register to be read */
  regid[0] = reg;
  seq.buf[0].data = regid;
  seq.buf[0].len = 1;
  seq.buf[1].data = data;
  seq.buf[1].len = 1;
 
  ret = I2CDRV_Transfer(&seq);
  if (ret != i2cTransferDone)
  {
    return 0;
  }

  return data[0];
}

void MMARegReadN(uint8_t reg1, uint8_t n, uint8_t *array)
{
	MMA845X_RegisterGet(I2C0, MMA8451Q_ADDR, reg1, array, n);
}
        
int16_t MMAReadX_14()
{
    uint8_t MSB_buf, LSB_buf;
    
    MSB_buf = MMARegRead(OUT_X_MSB_REG);
    LSB_buf = MMARegRead(OUT_X_LSB_REG);
    
    uint16_t ret = ((MSB_buf << 8 | LSB_buf) >> 2);
    
    return (int16_t) ret;
}

int16_t MMAReadY_14()
{
    uint8_t MSB_buf, LSB_buf;
    
    MSB_buf = MMARegRead(OUT_Y_MSB_REG);
    LSB_buf = MMARegRead(OUT_Y_LSB_REG);
    
    uint16_t ret = ((MSB_buf << 8 | LSB_buf) >> 2);
    
    return (int16_t) ret;
}

int16_t MMAReadZ_14()
{
    uint8_t MSB_buf, LSB_buf;
    
    MSB_buf = MMARegRead(OUT_Z_MSB_REG);
    LSB_buf = MMARegRead(OUT_Z_LSB_REG);
    
    uint16_t ret = ((MSB_buf << 8 | LSB_buf) >> 2);
    
    return (int16_t) ret;
}

void MMAStandby(void) {
  uint8_t r;
  r = MMARegRead(CTRL_REG1);
  MMARegWrite(CTRL_REG1, r & ~ACTIVE_MASK);
}
void MMAActive(void) {
  uint8_t r;
  r = MMARegRead(CTRL_REG1);
  MMARegWrite(CTRL_REG1, r | ACTIVE_MASK);
}

void MMAInit(void) {
  MMAStandby();
  MMARegWrite(XYZ_DATA_CFG_REG, ((MMARegRead(XYZ_DATA_CFG_REG) & ~FS_MASK) | FULL_SCALE_2G)); //2G Fullscale
  MMARegWrite(CTRL_REG4, 0);
  MMARegWrite(CTRL_REG4, INT_EN_DRDY_MASK);
  MMARegWrite(0x2E, INT_EN_DRDY_MASK); //Set the interrupt to route to INT1
  //MMARegWrite(0x2E, INT_EN_DRDY_MASK); //Set the interrupt to route to INT1
  MMARegWrite(CTRL_REG1, (MMARegRead(CTRL_REG1) & ~DR_MASK) | DATA_RATE_80MS); // Active ODR
  MMARegWrite(CTRL_REG1, (MMARegRead(CTRL_REG1) & ~FREAD_MASK)); // 14-bit reads
  MMARegWrite(CTRL_REG1, (MMARegRead(CTRL_REG1) | LNOISE_MASK)); // Low Noise Mode
  MMARegWrite(CTRL_REG2, (MMARegRead(CTRL_REG2) & ~MODS_MASK) | 0x00); // Oversampling Mode in Active
  MMAActive();  
}              

