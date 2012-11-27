//#define PPS_MASTER

#include "efm32.h"

#include "efm32_chip.h"
#include "efm32_rtc.h"
#include "efm32_gpio.h"
#include "efm32_cmu.h"
#include "efm32_timer.h"
#include "efm32_int.h"
#include "efm32_i2c.h"
#include "efm32_usart.h"

#include <stdint.h>
#include <stdbool.h>

#include "led.h"
#include "trace.h"
#include "radio.h"
#include "config.h"
#include "packets.h"

#include "i2cdrv.h"
#include "display.h"
#include "MAG3110.h"
#include "MMA845XQ.h"
#include "eCompass.h"

#include "gps.h"

// Global variables
DISPLAY_Message displayMessageTop;
DISPLAY_Message displayMessageBottom;
Mag_Vector_Type magReading;
Accel_Vector_Type accelReading;
GPS_Vector_Type gpsReading;
//geralds crib
GPS_Vector_Type goTo ={
    .alt = 100,
    .lat = 55921691,       
    .lon = -4138082,
};
uint8_t buf[6];
char str[32];

// Prototypes
void initClocks();
void enableTimers();
void enableInterrupts();
void basestation_main();
bool basestation_handlePacket(Packet p);

void wait(uint32_t ms) {
    uint32_t time,
            clockFreq = CMU_ClockFreqGet(cmuClock_RTC);

    while (ms > 0) {

        time = RTC_CounterGet();

        if (16777215 - time < ((double) ms / 1000.0) * clockFreq) {
            ms -= (uint32_t) (1000.0 * ((16777215 - time) / (double) clockFreq));
            while (RTC_CounterGet() > time);
        } else {
            while (RTC_CounterGet() < time + ((double) ms / 1000.0) * clockFreq);
            break;
        }

    }
}

int main() {

    // Chip errata
    CHIP_Init();

    // ensure core frequency has been updated
    SystemCoreClockUpdate();

    // start clocks
    initClocks();

    // I2C setup
    I2C_Setup();

    TRACE_Init();
    TRACE("Trace started!\n");

    // init LEDs
    LED_Init();



    // init irqs
    enableInterrupts();

    // GPS Init
    GPS_Init();



    // display init
    DISPLAY_Init();

    // init top line
    DISPLAY_InitMessage(&displayMessageTop);

    // init bottom line
    DISPLAY_InitMessage(&displayMessageBottom);
    displayMessageBottom.topLine = false;

    // magnetometer init
    MAGInit();
    magReading = getMAGReadings();

    // eCompass init
    eCompassInit();





    // display getting fix message
    displayMessageTop.message = ("Getting Fix :)");
    DISPLAY_MessageWrite(&displayMessageTop);

    // wait for gps initial fix
    GPS_GetFix();

    // display getting fix message
    displayMessageTop.message = ("Fix Found :D");
    DISPLAY_MessageWrite(&displayMessageTop);

    LED_Off(RED);

    TRACE(":FIX FOUND\n");

    LED_On(RED);
    
    while (1) {
        
        GPS_Read(&gpsReading);
        sprintf(str," lat %i\n ", gpsReading.lat);
        TRACE(str);
        sprintf(str," lon %i\n ", gpsReading.lon);
        TRACE(str);
        float bearing = getBearing(gpsReading.lat , gpsReading.lon , goTo.lat , goTo.lon);
        sprintf(str, "bearing %.3f\n\n", bearing);
        TRACE(str);
           

    }

}

void enableInterrupts() {

    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    NVIC_SetPriority(GPIO_EVEN_IRQn, 6);

    NVIC_EnableIRQ(TIMER0_IRQn);
    NVIC_EnableIRQ(TIMER1_IRQn);
    NVIC_EnableIRQ(TIMER2_IRQn);
    NVIC_EnableIRQ(TIMER3_IRQn);
    NVIC_SetPriority(TIMER0_IRQn, 4);
    NVIC_SetPriority(TIMER1_IRQn, 4);
    NVIC_SetPriority(TIMER2_IRQn, 4);
    NVIC_SetPriority(TIMER3_IRQn, 4);

    NVIC_EnableIRQ(USART2_TX_IRQn);
    NVIC_EnableIRQ(USART2_RX_IRQn);
    NVIC_SetPriority(USART2_TX_IRQn, 5);
    NVIC_SetPriority(USART2_RX_IRQn, 5);

    NVIC_EnableIRQ(LEUART1_IRQn);
    NVIC_SetPriority(LEUART1_IRQn, 3);

#ifndef BASESTATION

    NVIC_EnableIRQ(UART1_TX_IRQn);
    NVIC_SetPriority(UART1_TX_IRQn, 5);

#endif

}

void initClocks() {

    /* Starting LFXO and waiting until it is stable */
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    // starting HFXO, wait till stable
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

    // route HFXO to CPU
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

    /* Routing the LFXO clock to the RTC */
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);

    // disabling the RCs
    CMU_ClockEnable(cmuSelect_HFRCO, false);
    CMU_ClockEnable(cmuSelect_LFRCO, false);

    /* Enabling clock to the interface of the low energy modules */
    CMU_ClockEnable(cmuClock_CORE, true);
    CMU_ClockEnable(cmuClock_CORELE, true);

    // enable clock to hf perfs
    CMU_ClockEnable(cmuClock_HFPER, true);

    // enable clock to GPIO
    CMU_ClockEnable(cmuClock_GPIO, true);

    // enable clock to RTC
    CMU_ClockEnable(cmuClock_RTC, true);
    RTC_Enable(true);

    // enable radio usart
    CMU_ClockEnable(cmuClock_USART2, true);

    // enable pc serial
    CMU_ClockEnable(cmuClock_UART1, true);

    // enable timers
    CMU_ClockEnable(cmuClock_TIMER0, true);
    CMU_ClockEnable(cmuClock_TIMER1, true);
    CMU_ClockEnable(cmuClock_TIMER2, true);
    CMU_ClockEnable(cmuClock_TIMER3, true);

    // i2c
    CMU_ClockEnable(cmuClock_I2C0, true);

    // LEUART for GPS
    CMU_ClockEnable(cmuClock_LEUART1, true);

}