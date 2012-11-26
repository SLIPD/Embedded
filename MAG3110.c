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

int16_t MAGReadX_16()
{
    uint8_t MSB_buf, LSB_buf;
    
    MSB_buf = MAGRegRead(MAG_OUT_X_MSB_REG);
    LSB_buf = MAGRegRead(MAG_OUT_X_LSB_REG);
    
    uint16_t ret = (MSB_buf << 8 | LSB_buf);
    
    return (int16_t) ret;
}

int16_t MAGReadY_16()
{
    uint8_t MSB_buf, LSB_buf;
    
    MSB_buf = MAGRegRead(MAG_OUT_Y_MSB_REG);
    LSB_buf = MAGRegRead(MAG_OUT_Y_LSB_REG);
    
    uint16_t ret = (MSB_buf << 8 | LSB_buf);
    
    return (int16_t) ret;
}

int16_t MAGReadZ_16()
{
    uint8_t MSB_buf, LSB_buf;
    
    MSB_buf = MAGRegRead(MAG_OUT_Z_MSB_REG);
    LSB_buf = MAGRegRead(MAG_OUT_Z_LSB_REG);
    
    uint16_t ret = (MSB_buf << 8 | LSB_buf);
    
    return (int16_t) ret;
}

/**
 * Sets MAG3110 to STANDBY mode.
 */
void MAGStandby(void) 
{
    uint8_t r;
    r = MAGRegRead(MAG_CTRL_REG1);
     // Bit 0 of CTRL_REG1, 0 is standby
    MAGRegWrite(MAG_CTRL_REG1, r & ~MAG_ACTIVE_MASK);
}

/**
 * Sets MAG3110 to ACTIVE mode.
 * In ACTIVE mode, the MAG3110 will make periodic measurements based on values
 * in Data Rate (DR) and Over Sampling RAtio bits (OS).
 */
void MAGActive(void) 
{
    uint8_t r;
    r = MAGRegRead(MAG_CTRL_REG1);
     // Bit 0 of CTRL_REG1, 1 is active
    MAGRegWrite(MAG_CTRL_REG1, r | MAG_ACTIVE_MASK);
}

void MAGInit(void) 
{
    MAGStandby();
    // MAGRegWrite(0x2E, INT_EN_DRDY_MASK); //Set the interrupt to route to INT1
    // MAGRegWrite(0x2E, INT_EN_DRDY_MASK); //Set the interrupt to route to INT1
    
//    // Reset offset regs
//    MAGRegWrite(MAG_OFF_X_MSB, 0x00);
//    MAGRegWrite(MAG_OFF_X_LSB, 0x00);
//    MAGRegWrite(MAG_OFF_Y_MSB, 0x00);
//    MAGRegWrite(MAG_OFF_Y_LSB, 0x00);
//    MAGRegWrite(MAG_OFF_Z_MSB, 0x00);
//    MAGRegWrite(MAG_OFF_Z_LSB, 0x00);
//    
//    // CTRL_REG2
//    MAGRegWrite(MAG_CTRL_REG2, (MAGRegRead(MAG_CTRL_REG2) | MAG_AUTO_MRST_MASK )); // Activate automatic magnetic sensor resets
//    MAGRegWrite(MAG_CTRL_REG2, (MAGRegRead(MAG_CTRL_REG2) | MAG_RAW_MASK )); // Activate raw mode
//    
//    // CTRL_REG1
//    MAGRegWrite(MAG_CTRL_REG1, (MAGRegRead(MAG_CTRL_REG1) & ~MAG_ODR_MASK)); // Active ODR, 80Hz
//    MAGRegWrite(MAG_CTRL_REG1, (MAGRegRead(MAG_CTRL_REG1) & ~MAG_OS_MASK)); // Active OS, 1
//    MAGRegWrite(MAG_CTRL_REG1, (MAGRegRead(MAG_CTRL_REG1) & ~MAG_FR_MASK)); // Deactivate Fast Read
//    MAGRegWrite(MAG_CTRL_REG1, (MAGRegRead(MAG_CTRL_REG1) & ~MAG_TM_MASK)); // Deactivate Trigger Measurement
    
    MAGRegWrite(MAG_CTRL_REG2, 0xA0);
    MAGRegWrite(MAG_CTRL_REG1, 0x01);
    
    MAGActive();  
}              

Mag_Vector_Type getMAGReadings()
{
    Mag_Vector_Type magReading;
    magReading.x = MAGReadX_16();
    magReading.y = MAGReadY_16();
    magReading.z = MAGReadZ_16();
    return magReading;
}

void writeMagOffset(uint16_t xOff, uint16_t yOff, uint16_t zOff)
{
    MAGStandby();
    
    char str[32];
    
    uint16_t XMSB = (((int16_t) xOff )>> 7);
    if(XMSB & 0x8000)
    {
        XMSB = (uint8_t)(XMSB|0x80);
    }
    
    uint16_t YMSB = (((int16_t) yOff )>> 7);
    if(YMSB & 0x8000)
    {
        YMSB = (uint8_t)(YMSB|0x80);
    }
  
    uint16_t ZMSB = (((int16_t) zOff )>> 7);
    if(ZMSB & 0x8000)
    {
        ZMSB = (uint8_t)(ZMSB|0x80);
    }
    
    sprintf(str, "xOff 0x%4.4x XMSB 0x%4.4x\n", xOff, XMSB);
    TRACE(str);
    sprintf(str, "yOff 0x%4.4x YMSB 0x%4.4x\n", yOff, YMSB);
    TRACE(str);
    sprintf(str, "zOff 0x%4.4x ZMSB 0x%4.4x\n", zOff, ZMSB);
    TRACE(str);
    
    // Reset offset regs
    MAGRegWrite(MAG_OFF_X_MSB, (uint8_t) XMSB);
    MAGRegWrite(MAG_OFF_X_LSB, (uint8_t) ((xOff & 0xFF) << 1));
    MAGRegWrite(MAG_OFF_Y_MSB, (uint8_t) YMSB);
    MAGRegWrite(MAG_OFF_Y_LSB, (uint8_t) ((yOff & 0xFF) << 1));
    MAGRegWrite(MAG_OFF_Z_MSB, (uint8_t) ZMSB);
    MAGRegWrite(MAG_OFF_Z_LSB, (uint8_t) ((zOff & 0xFF) << 1));
    
    sprintf(str, "xOff 0x%4.4x, MSB 0x%2.2x, LSB 0x%2.2x\n", xOff, (uint8_t) XMSB, (uint8_t)((xOff & 0xFF) << 1));
    TRACE(str);
    sprintf(str, "yOff 0x%4.4x, MSB 0x%2.2x, LSB 0x%2.2x\n", yOff, (uint8_t) YMSB, (uint8_t) ((yOff & 0xFF) << 1));
    TRACE(str);
    sprintf(str, "zOff 0x%4.4x, MSB 0x%2.2x, LSB 0x%2.2x\n", zOff, (uint8_t) ZMSB, (uint8_t)((zOff & 0xFF) << 1));
    TRACE(str);
    
    MAGRegWrite(MAG_CTRL_REG2, (MAGRegRead(MAG_CTRL_REG2) | MAG_AUTO_MRST_MASK )); // Activate automatic magnetic sensor resets
    MAGRegWrite(MAG_CTRL_REG2, (MAGRegRead(MAG_CTRL_REG2) & ~MAG_RAW_MASK )); // Activate user offset mode
    
    
    MAGActive(); 
}