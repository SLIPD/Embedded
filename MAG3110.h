/* 
 * File:   MAG3110.h
 * Author: GCHAU
 *
 * Created on 13 October 2012, 17:05
 */

#ifndef MAG3110_H
#define	MAG3110_H

#include "efm32.h"

#ifdef	__cplusplus
extern "C" {
#endif
    
#define PI 3.141592654

#define MAG3110_ADDR 0x0E
#define MAG3110_ADDR_WRITE 0x1C
#define MAG3110_ADDR_READ 0x1D
    
    /*
     **  STATUS Registers
     * 
     * Note: ZYXOW & ZYXDR are logical OR operations.
     * ZYXDR bit is connected to INT1 pin (Pin Bank E, Pin 15)
     * 
     * Bit 7 - ZYXOW, if 1, prev X Y Z data was overwritten by new X Y Z data
     *         before it was completely read. 
     * Bit 6 - ZOW, if 1, prev Z data was overwritten by new Z data.
     * Bit 5 - YOW
     * Bit 4 - XOW
     * Bit 3 - ZYXDR, if 1, new set of data ready. (Not necessarily in all axis)
     * Bit 2 - ZDR, if 1, new Z data ready.
     * Bit 1 - YDR
     * Bit 0 - XDR
     */
#define DR_STATUS_REG         0x00      // Data Ready Status Reg

    /*
     * Data Overwrites Before Data Set Read Complete
     *
     * ZXYOW
     * Bit 7, X Y Z axis data overwrite, 0 no overwrite, 1 whenever new data is
     * acquired, before completing retrieval of the previous set. Triggered when
     * at least one data register has been overwritten. Cleared when high byte of 
     * data registers of all active channels are read.
     */
#define ZYXOW_MASK            0x80      // Bit 7, ZYX axis overwrite
#define ZOW_MASK              0x40      // Bit 6, Z axis overwrite
#define YOR_MASK              0x20      // Bit 5, Y axis overwrite
#define XOR_MASK              0x10      // Bit 4, X axis overwrite
    
    /*
     * New Data Available
     */
#define ZYXDR_MASK            0x08      // Bit 3, new set of data ready 
#define ZDR_MASK              0x04      // Bit 2, Z axis data ready
#define YDR_MASK              0x02      // Bit 1, Y axis data ready
#define XDR_MASK              0x01      // Bit 0, X axis data ready
    
    /*
     * XYZ Data Registers
     * 
     * Output sample data expressed as signed 2's complement numbers
     * When CTRL_REG2[RAW] = 1, output range: -20,000 to 20,000 bit counts
     * When CTRL_REG2[RAW] = 0, output range: -30,000 to 30,000 bit counts
     * With user offset included!
     * Data acquisition is a seq. read of 6 bytes.
     * If Fast Read (FR) is on, auto-increment will skip LSB.
     * Reg 0x01 must be read first to ensure latest acquisition data is being
     * read.
     * 
     * OUT_X_MSB should be read first so that the data in 
     * registers OUT_X_LSB through OUT_Z_LSB are updated.
     * 
     */
#define OUT_X_MSB_REG         0x01      // [15:8] of X measurement
#define OUT_X_LSB_REG         0x02      // [7:0] of X measurement
#define OUT_Y_MSB_REG         0x03      // [15:8] of Y measurement
#define OUT_Y_LSB_REG         0x04      // [7:0] of Y measurement       
#define OUT_Z_MSB_REG         0x05      // [15:8] of Z measurement
#define OUT_Z_LSB_REG         0x06      // [7:0] of Z measurement

    /*
     * MAG3110 Unit
     * SYSMOD modes:    00 - STANDBY MODE
     *                  01 - ACTIVE MODE, RAW DATA
     *                  10 - ACTIVE MODE, non-RAW user-corrected data
     * 
     * ACTIVE: The device will keep acquiring data continuously according to
     *         settings in CTRL_REG1 (CTRL_REG1 = 0bXXXXXX01)
     * 
     * STANDBY: The part is in a low-power state and no data acquisition 
     *          is taking place. The part defaults to Standby mode upon 
     *          power-up (AC = TM = 0).
     */
#define WHO_AM_I              0x07      // DEVICE ID NUMBER, 0xC4
#define SYSMOD                0x08      // Current System / Operating Mod
#define DIE_TEMP              0x0F      // Temperature, signed 8 bits in degrees

    /*
     * User offset 
     */
#define OFF_X_MSB             0x09      // [14:7] of user X offset 
#define OFF_X_LSB             0x0A      // [6:0] of user X offset, bit 0 not used
#define OFF_Y_MSB             0x0B      // [14:7] of user Y offset 
#define OFF_Y_LSB             0x0C      // [6:0] of user Y offset, bit 0 not used
#define OFF_Z_MSB             0x0D      // [14:7] of user Z offset 
#define OFF_Z_LSB             0x0E      // [6:0] of user Z offset, bit 0 not used
    
    /*
     * CTRL Registers
     * Except for STANDBY MODE(Bit 0 AC),
     * Must be in STANDBY MODE (SYSMOD) to change fields in CTRL_REG1
     * 
     * CTRL_REG1:       Bit 7 - ODR2, Data Rate Selection
     *                  Bit 6 - ODR1
     *                  Bit 5 - ODR0
     *                  Bit 4 - OS1, Over Sampling Ratio
     *                  Bit 3 - OS0
     *                  Bit 2 - FR, Fast Read
     *                  Bit 1 - TM, Trigger Immediate Measurement
     *                  Bit 0 - AC, Operating mode selection
     * 
     * CTRL_REG2:       Bit 7 - AUTO_MRST_EN, Automatic Magnetic Sensor Reset*
     *                  Bit 5 - RAW, Data output correction
     *                  Bit 4 - Mag_RST, Magnetic Sensor Reset
     * 
     * *Recommended that AUTO_MRST_E be set in
     *  the init routines before taking measurements.
     * 
     */
#define CTRL_REG1             0x10     
#define CTRL_REG2             0x11   
    
#define ODR2_MASK             0x80
#define ODR1_MASK             0x40
#define ODR0_MASK             0x20
#define OS1_MASK              0x10
#define OS0_MASK              0x08
#define FR_MASK               0x04
#define TM_MASK               0x02
#define ACTIVE_MASK           0x01
    
#define ODR_MASK              0xE0
#define OS_MASK               0x18
    
#define AUTO_MRST_MASK        0x80
#define RAW_MASK              0x20
#define MAG_RST_MASK          0x10
    
    /**
     * Data Rate Values (ODR)
     */
    
//#define ADC_RATE_1280         0x00
    
//prototypes   
    int MAG_RegisterGet(I2C_TypeDef *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
    
    void MAGRegWrite(uint8_t reg, uint8_t val);
    uint8_t MAGRegRead(uint8_t reg);
    void MAGRegReadN(uint8_t reg1, uint8_t n, uint8_t *array);
    
    void MAGStandby(void);
    void MAGActive(void);
    void MAGInit(void);
    
typedef struct 
{
    int16_t x;
    int16_t y;
    int16_t z;
} Mag_Vector_Type __attribute__ ((packed));
#define Mag_Vector_Len 6

#ifdef __cplusplus
}
#endif

#endif

