#include <stddef.h>
#include "efm32.h"
#include "efm32_usart.h"
#include "efm32_emu.h"
#include "efm32_i2c.h"
#include "MAG3110.h"

int MAG_RegisterGet(I2C_TypeDef *i2c,
                         uint8_t addr,
                         uint8_t reg,
                         uint8_t *buf,
			 uint8_t len)
{
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t regid[1];     // Register ID

  seq.addr = addr;      // Parameter Address
  seq.flags = I2C_FLAG_WRITE_READ;     // Indicate combined write / read seq 
  /* Select register to be read */
  // Format: Reg ID, Value
  // Length must be specified
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

/**
 * Write to register, use to write CTRL_REG1 and CTRL_REG2
 * @param reg Register ID to write to
 * @param val Value to write to register
 */
void MAGRegWrite(uint8_t reg, uint8_t val) 
{
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;
    uint8_t regid[1]; // REG ID

    seq.addr = MAG3110_ADDR_WRITE;        // MAG3110 write address
    seq.flags = I2C_FLAG_WRITE_WRITE;     // Indicate write sequence, 2 buffers
    /* Select register to be read */
    // Format: Reg ID, Value
    // Length must be specified
    regid[0] = reg;                       
    seq.buf[0].data = regid;              
    seq.buf[0].len = 1;
    seq.buf[1].data = &val;
    seq.buf[1].len = 1;

    // Transfer / Write to MAG Register
    ret = I2CDRV_Transfer(&seq);
}

/**
 * Write to register, use to read data registers
 * @param reg Register ID  to read from
 */
uint8_t MAGRegRead(uint8_t reg) 
{
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;
    uint8_t regid[1]; // REG ID
    uint8_t data[1]; // Data holder

    seq.addr = MAG3110_ADDR_READ;         // MAG3110 write address
    seq.flags = I2C_FLAG_WRITE_READ;      // Indicate combined write / read seq
    /* Select register to be read */  
    // Format: Reg ID, Value
    // Length must be specified
    regid[0] = reg;
    seq.buf[0].data = regid;
    seq.buf[0].len = 1;
    seq.buf[1].data = data;
    seq.buf[1].len = 1;

    ret = I2CDRV_Transfer(&seq);
    // While transfer is not done... return 0
    if (ret != i2cTransferDone)
    {
      return 0;
    }

    return data[0];
}

void MAGRegReadN(uint8_t reg1, uint8_t n, uint8_t *array)
{
    // I2CO                 - base pointer 
    // MAG3110_ADDR_READ    - MAG3110 read address
    // reg1                 - read this register on MAG
    // array                - Array to read data into (data holder)
    // n                    - Number of bytes to read

    MAG_RegisterGet(I2C0, MAG3110_ADDR_READ, reg1, array, n);
}

/**
 * Sets MAG3110 to STANDBY mode.
 */
void MAGStandby(void) 
{
    uint8_t r;
    r = MAGRegRead(CTRL_REG1);
     // Bit 0 of CTRL_REG1, 0 is standby
    MAGRegWrite(CTRL_REG1, r & ~ACTIVE_MASK);
}

/**
 * Sets MAG3110 to ACTIVE mode.
 * In ACTIVE mode, the MAG3110 will make periodic measurements based on values
 * in Data Rate (DR) and Over Sampling RAtio bits (OS).
 */
void MAGActive(void) 
{
    uint8_t r;
    r = MAGRegRead(CTRL_REG1);
     // Bit 0 of CTRL_REG1, 1 is active
    MAGRegWrite(CTRL_REG1, r | ACTIVE_MASK);
}

void MAGInit(void) 
{
    MAGStandby();
    // MAGRegWrite(0x2E, INT_EN_DRDY_MASK); //Set the interrupt to route to INT1
    // MAGRegWrite(0x2E, INT_EN_DRDY_MASK); //Set the interrupt to route to INT1
    MAGRegWrite(CTRL_REG1, (MAGRegRead(CTRL_REG2) | AUTO_MRST_MASK )); // Activate automatic magnetic sensor resets
    MAGRegWrite(CTRL_REG1, (MAGRegRead(CTRL_REG1) & ~ODR_MASK)); // Active ODR, 80Hz
    MAGRegWrite(CTRL_REG1, (MAGRegRead(CTRL_REG1) & ~OS_MASK)); // Active OS, 1
    MAGRegWrite(CTRL_REG1, (MAGRegRead(CTRL_REG1) & ~FR_MASK)); // Deactivate Fast Read
    MAGRegWrite(CTRL_REG1, (MAGRegRead(CTRL_REG1) & ~TM_MASK)); // Deactivate Trigger Measurement
    MAGRegWrite(CTRL_REG1, (MAGRegRead(CTRL_REG1) & ~ACTIVE_MASK) | 0x01); // Active Mode
    MAGActive();  
}              

