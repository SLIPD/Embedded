
#include "gps.h"

#include "stdint.h"
#include "string.h"
#include "stdbool.h"
#include "efm32.h"

#include "efm32_gpio.h"
#include "efm32_leuart.h"
#include "efm32_rtc.h"
#include "efm32_cmu.h"
#include "efm32_int.h"
#include "trace.h"
#include "led.h"
#include "packets.h"
#include "radio.h"

const static uint8_t sirf_command_NMEA[] = {0xA0, 0xA2, 0x00, 0x18, 0x81, 0x02, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x12, 0xC0, 0x01, 0x60, 0xB0, 0xB3};
const static uint8_t sirf_command_MPM[] = {0xA0, 0xA2, 0x00, 0x06, 0xDA, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0xB0, 0xB3};
const static uint8_t sirf_command_PTF[] = {0xA0, 0xA2, 0x00, 0x0E, 0xDA, 0x04, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x0F, 0xA0, 0x00, 0x02, 0xBF, 0x20, 0x02, 0xE6, 0xB0, 0xB3};
const static uint8_t sirf_command_fullPowerMode = {0xA0, 0xA2, 0x00, 0x02, 0xDA, 0x00, 0x00, 0xDA, 0xB0, 0xB3};
// this returns a single binary byte that is the checksum

/* variables */
uint32_t last_mode_change = 0;
uint32_t fix = 0;
uint32_t nmea_len = 0;
uint32_t nmea_msg_rcvd = 0;
uint16_t seqNumber;
uint8_t nmea_buffer[256];
uint8_t localBuff[256];
GPS_Vector_Type lastRead;
GPS_Vector_Type toSend;
bool sendNow;
bool haveRead;
/* prototypes */
void switchMode();

/* functions */
void GPS_Init() {
    haveRead = false;
    lastRead.alt = 0;
    lastRead.lat = 0;
    lastRead.lon = 0;
    int nextRTC;
    sendNow = false;
    seqNumber = 0;
    fix = 0;
    LEUART_Init_TypeDef leuart1Init = {
        .enable = leuartEnable, /* Activate data reception on LEUn_TX pin. */
        .refFreq = CMU_ClockFreqGet(cmuClock_LEUART1), /* Inherit the clock frequenzy from the LEUART clock source */
        .baudrate = 4800, /* Baudrate = 9600 bps */
        .databits = leuartDatabits8, /* Each LEUART frame containes 8 databits */
        .parity = leuartNoParity, /* No parity bits in use */
        .stopbits = leuartStopbits2, /* Setting the number of stop bits in a frame to 2 bitperiods */
    };

    LEUART_Reset(LEUART1);
    LEUART_Init(LEUART1, &leuart1Init);
    //interrupt enable register to when receive
    LEUART1->IEN = LEUART_IEN_RXDATAV;




    //GPIO->P[0].DOUT |= (1 << 5);
    GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE5_MASK) | GPIO_P_MODEL_MODE5_PUSHPULL;
    GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE6_MASK) | GPIO_P_MODEL_MODE6_INPUT;


    //setting dc dc pin (janek suggested)
    GPIO->P[0].MODEH =
            GPIO_P_MODEH_MODE14_WIREDAND;
    GPIO->P[0].DOUT &= ~(1 << 14);


    LEUART1->ROUTE = LEUART_ROUTE_LOCATION_LOC1
            | LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN;

    // config wakeup pin
    GPIO->P[4].DOUT &= ~(1 << 12);
    GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE12_MASK) | GPIO_P_MODEH_MODE12_INPUT;

    // config on off pin
    GPIO->P[4].DOUT &= ~(1 << 13);
    GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE13_MASK) | GPIO_P_MODEH_MODE13_PUSHPULL;


    RTC_CounterReset();
    while (RTC->CNT < 65536);
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof (sirf_command_NMEA));
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof (sirf_command_NMEA));
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof (sirf_command_NMEA));
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof (sirf_command_NMEA));
    //uart_setbaud(LEUART1,4800);
    while (RTC->CNT < (65536 + 9 * (16000)));

    nmea_sendmessage(LEUART1, "PSRF117,16");



    while (RTC->CNT < (65536 + 10 * (16000)));

    /* Pulse GPS on/off signal for 500 ms */
    nextRTC = RTC->CNT + 8000;
    GPIO->P[4].DOUT = 1 << 13;
    while (RTC->CNT < nextRTC);
    GPIO->P[4].DOUT = 0;


    RTC_CounterReset();
    while (!(GPIO->P[4].DIN & (1 << 12))) {
        TRACE("SWITCHING MODE");
        switchMode();
    }
    //turn off all other messages
    nmea_sendmessage(LEUART1, "PSRF103,00,00,00,01");
    nmea_sendmessage(LEUART1, "PSRF103,03,00,00,01");
    nmea_sendmessage(LEUART1, "PSRF103,04,00,00,01");

    //gsa messages every second
    nmea_sendmessage(LEUART1, "PSRF103,02,00,01,01");
    TRACE("GPS INIT COMPLETE\n");


}

void switchMode() {
    //
    // wait at least 1 second since last change
    while (RTC_CounterGet() < last_mode_change + CMU_ClockFreqGet(cmuClock_RTC));

    // disable interrupts as this is time sensitive
    NVIC_DisableIRQ(GPIO_EVEN_IRQn);
    NVIC_DisableIRQ(GPIO_ODD_IRQn);
    NVIC_DisableIRQ(LEUART1_IRQn);

    GPIO->P[4].DOUT |= (1 << 13);

    uint32_t time = RTC_CounterGet(),
            wait = CMU_ClockFreqGet(cmuClock_RTC) / 10;

    while (RTC_CounterGet() < time + wait);

    GPIO->P[4].DOUT &= ~(1 << 13);

    time = RTC_CounterGet();
    wait = CMU_ClockFreqGet(cmuClock_RTC);
    while (RTC_CounterGet() < time + wait);

    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
    NVIC_EnableIRQ(LEUART1_IRQn);
    last_mode_change = RTC_CounterGet();

}

uint8_t nmea_generateChecksum(char *strPtr) {
    int p;
    char c;
    uint8_t chksum;

    c = strPtr[0]; // get first chr
    chksum = c;
    p = 1;
    while (c != 0x00) {
        c = strPtr[p]; // get next chr
        if (c != 0x00) {
            chksum = chksum ^ c;
        }
        p++;
    }

    return chksum;
}

void leuart_send_str(LEUART_TypeDef *uart, char *msg) {
    uint8_t c;
    while ((c = *(msg++)) != 0) {
        while (!(uart->STATUS & LEUART_STATUS_TXBL));
        uart->TXDATA = c;
    }
}


// send nmea message with checksum. 0 terminated string not including $,*,CHK,CRLF

void nmea_sendmessage(LEUART_TypeDef *uart, char *msg) {
    uint8_t chksum;
    uint8_t c;
    chksum = nmea_generateChecksum(msg);
    while (!(uart->STATUS & LEUART_STATUS_TXBL));
    uart->TXDATA = '$';
    leuart_send_str(uart, msg);
    while (!(uart->STATUS & LEUART_STATUS_TXBL));
    uart->TXDATA = '*';
    while (!(uart->STATUS & LEUART_STATUS_TXBL));
    if (((chksum & 0xF0) >> 4) > 9) {
        uart->TXDATA = ((chksum & 0xF0) >> 4) - 10 + 'A';
    } else {
        uart->TXDATA = ((chksum & 0xF0) >> 4) + '0';
    }
    while (!(uart->STATUS & LEUART_STATUS_TXBL));
    if (((chksum & 0x0F)) > 9) {
        uart->TXDATA = ((chksum & 0x0F)) - 10 + 'A';
    } else {
        uart->TXDATA = ((chksum & 0x0F)) + '0';
    }
    while (!(uart->STATUS & LEUART_STATUS_TXBL));
    uart->TXDATA = '\r';
    while (!(uart->STATUS & LEUART_STATUS_TXBL));
    uart->TXDATA = '\n';
}

void leuart_send_array(LEUART_TypeDef *uart, uint8_t *msg, uint32_t len) {
    for (int i = 0; i < len; i++) {
        while (!(uart->STATUS & LEUART_STATUS_TXBL));
        uart->TXDATA = msg[i];
    }
}

void LEUART1_IRQHandler(void) {
    uint8_t b;
    //LEUART1->IFC = ~_LEUART_IFC_RESETVALUE;
    while (!(UART1->STATUS & UART_STATUS_TXBL));
    b = LEUART1->RXDATA;
    //we have the start of a message
    if (b == '$') {
        nmea_len = 0;
    }
    if (nmea_len >= 253) {
        nmea_len = 0;
    }

    nmea_buffer[nmea_len++] = b;



    if (b == '\n') {
        nmea_buffer[nmea_len++] = '\0';
        //TRACE(nmea_buffer);
        nmea_msg_rcvd = 1;
    }
}

/**
 * busy waits until 3D fix is achieved
 */
bool GPS_GetFix() {

    if (fix == 1) {
        return true;
    }

    if (nmea_msg_rcvd) {
        INT_Disable();
        memcpy(localBuff, nmea_buffer, 256);
        INT_Enable();

        if ((localBuff[4] == 'S') && (localBuff[5] == 'A')) {

            switch (localBuff[9]) {
                case '1':
                    TRACE("no fix \n");
                    LED_Toggle(3);
                    return false;
                    break;
                case '2':
                    //TRACE("2d fix ");
                    return false;
                    break;
                case '3':
                    TRACE("FULL 3D FIX");
                    LED_Off(3);
                    LED_On(1);
                    fix = 1;
                    //gga every second, turn others off
                    nmea_sendmessage(LEUART1, "PSRF103,00,00,01,01");
                    nmea_sendmessage(LEUART1, "PSRF103,03,00,00,01");
                    nmea_sendmessage(LEUART1, "PSRF103,04,00,00,01");
#ifndef BASESTATION
                    //turn off gsa messages if we are not the basestation
                    nmea_sendmessage(LEUART1, "PSRF103,02,00,00,01");
#endif
                    return true;
            }

        }
        nmea_msg_rcvd = 0;

    }else {
    return false;
	}
}





void GPS_GetPrecision(uint8_t target_precision) {

    bool precision_reached = false;
    while (!precision_reached) {

        if (nmea_msg_rcvd) {
            INT_Disable();
            memcpy(localBuff, nmea_buffer, 256);
            INT_Enable();

            uint8_t precision = 0;
            uint8_t msg_type[6];

            memcpy(msg_type, &localBuff[1], 5);

            msg_type[5] = 0;

            char *msg_req_type = "GPGSA";

            if (strcmp((char*) msg_type, msg_req_type) == 0) {

                int commas = 0;
                int x = 0;
                uint8_t decimal_places = 0xFF;

                do {
                    if (localBuff[x] == ',') {
                        commas++;
                        x++;
                        continue;
                    }
                    switch (commas) {
                        case 15:

                            if (decimal_places != 0xFF) {
                                decimal_places += 1;
                                if (decimal_places > 1) {
                                    break;
                                }
                            }

                            if (localBuff[x] == '.') {
                                decimal_places = 0;

                            } else {
                                precision *= 10;
                                precision += (localBuff[x] - '0');

                            }

                            break;
                    }
                    x++;
                } while (localBuff[x] != '\n');

                if (precision < target_precision && precision > 0)
                    precision_reached = true;

            }

            nmea_msg_rcvd = 0;
        }

    }

}

/**
 * returns last position... can only be done after gps_read
 * @param vector
 */
void GPS_GetLastPosition(GPS_Vector_Type *vector) {
    if (!haveRead) {
        GPS_Read(vector);
    }
    vector->alt = lastRead.alt;
    vector->lat = lastRead.lat;
    vector->lon = lastRead.lon;

}

/**
 * reads in position data after we have got a fix
 * only call after calling GPS_GetFix
 * Don't call repeatedly outside of gps.c
 */
bool GPS_Read(GPS_Vector_Type *vector) {
    haveRead = true;
    bool updated = false;
    if (nmea_msg_rcvd) {
        if (fix == 1) {
            INT_Disable();
            memcpy(localBuff, nmea_buffer, 256);
            nmea_msg_rcvd = 0;
            INT_Enable();
            //we have a fix so get coords
            //parse data to get gps coords
            //checking gga messages
            if (localBuff[1] == 'G' && localBuff[3] == 'G' && localBuff[4] == 'G') {
                //counter for number of commas (used for parsing)
                int commas = 0;
                int x = 0;
                //north/south indicator
                int north = 1;
                //east/west indicator
                int east = 1;
                //
                int altCount = 0;
                uint8_t longBuff[11] = "dddmm.mmmm";
                uint8_t latBuff[10] = "ddmm.mmmm";
                uint8_t altBuffer[16] = "";

                //variables used for counting lat/long into buffer
                int start, bufCount;
                do {
                    //count which argument we are at
                    if (localBuff[x] == ',') {
                        commas++;
                        x++;
                        continue;
                    }
                    switch (commas) {
                        case 2:
                            //long
                            start = x;
                            bufCount = 0;
                            //read longitude into buffer
                            while (x < (start + 8)) {
                                latBuff[bufCount] = localBuff[x];
                                bufCount++;
                                x++;
                            }
                            latBuff[bufCount] = localBuff[x];
                            break;
                        case 3:
                            //n/s

                            if (localBuff[x] == 'S')
                                north = -1;
                            break;
                        case 4:
                            //long
                            start = x;
                            bufCount = 0;
                            //read longitude into buffer
                            while (x < (start + 9)) {
                                longBuff[bufCount] = localBuff[x];
                                bufCount++;
                                x++;
                            }
                            longBuff[bufCount] = localBuff[x];
                            break;
                        case 5:
                            //e/w
                            if (localBuff[x] == 'W')
                                east = -1;
                            break;
                        case 6:
                            //fix indicator
                            if ((localBuff[x]) == '0') {
                                TRACE("FIX LOST");
                                //need to do something here
                                fix = 0;
                            }
                            break;
                        case 9:
                            //altitude with 1 decimal place
                            if (localBuff[x] != '.') {
                                altBuffer[altCount] = localBuff[x];
                                altCount++;
                            }



                    }



                    x++;
                } while (localBuff[x] != '\n');


                //convert to integers
                longBuff[5] = "\0";
                latBuff[4] = "\0";
                altBuffer[altCount] = "\0";
                int16_t altit = atoi(altBuffer);

                int32_t longitude = 10000 * atoi(longBuff);
                longitude += atoi(&longBuff[6]);
                longitude = longitude*east;
                int32_t latitude = 10000 * atoi(latBuff);
                latitude += atoi(&latBuff[5]);
                latitude = latitude*north;

                lastRead.alt = altit;
                lastRead.lat = latitude;
                lastRead.lon = longitude;
                updated = true;
            }


        }
    }



    vector->alt = lastRead.alt;
    vector->lat = lastRead.lat;
    vector->lon = lastRead.lon;
    return updated;

}

//creates packets and sends on second iteration

void GPS_Main() {
	
    if (sendNow) {
        sendNow = false;
        MessageType_NodePosition nodePos;
        nodePos.elevation = toSend.alt;
        nodePos.latitude = toSend.lat;
        nodePos.longitude = toSend.lon;
        nodePos.last_seq_num = seqNumber;
        Packet pack;
        pack.destinationId = 0x00;
        pack.msgType = 0x01;
        pack.payload.nodePosition = nodePos;
        RADIO_Send((uint8_t*) & pack);
        seqNumber++;

    } else if (GPS_Read(&toSend)) {
        sendNow = true;
    }

}
