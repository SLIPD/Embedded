/***************************************************************************//**
 * @file
 * @brief I2C0 poll based driver for master mode operation on DVK.
 * @author Energy Micro AS
 * @version 1.6.1
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2010 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * This source code is the property of Energy Micro AS. The source and compiled
 * code may only be used on Energy Micro "EFM32" microcontrollers.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include <stddef.h>
//#include "dvk_boardcontrol.h"
#include "i2cdrv.h"
#include "efm32_cmu.h"
#include "efm32_gpio.h"

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

void I2C_Setup()
{
    // init reset pin
    GPIO_PinModeSet(gpioPortD, 10, gpioModePushPull, 1);
    // init i2c 14 data
    GPIO_PinModeSet(gpioPortD, 15, gpioModeWiredAnd, 1);
    // init i2c 15 clk
    GPIO_PinModeSet(gpioPortD, 14, gpioModeWiredAnd, 1);
    
    // init i2c
    I2C0->ROUTE |= I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | I2C_ROUTE_LOCATION_LOC3;

    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

    I2C_Init(I2C0, &i2cInit);
}

/***************************************************************************//**
 * @brief
 *   Perform I2C transfer.
 *
 * @details
 *   This driver only supports master mode, single bus-master. It does not
 *   return until the transfer is complete, polling for completion.
 *
 * @param[in] seq
 *   Pointer to sequence structure defining the I2C transfer to take place. The
 *   referenced structure must exist until the transfer has fully completed.
 ******************************************************************************/
I2C_TransferReturn_TypeDef I2CDRV_Transfer(I2C_TransferSeq_TypeDef *seq)
{
  I2C_TransferReturn_TypeDef ret;

  /* Do a polled transfer */
  ret = I2C_TransferInit(I2C0, seq);
  while (ret == i2cTransferInProgress)
  {
    ret = I2C_Transfer(I2C0);
  }

  return(ret);
}
