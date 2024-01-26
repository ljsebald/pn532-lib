/*
   pico-i2c-felica.c
   Copyright (C) 2024 Lawrence Sebald

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documnetation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to  whom the Software is
   furished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
*/

/*
    Example for reading the IDm/PMm/System Code from a FeliCa card.

    The default setup is for SDA to be on pin GP16 and SCL to be on pin GP17.
    This can be adjusted with the #define statements below.
*/

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/i2c.h"
#include "pn532_pico.h"

#define I2C_BUS i2c0
#define SDA_PIN 16
#define SCL_PIN 17

static PN532 nfc;

static int init_pn532(void) {
    uint8_t version[4];
    int err;

    /* Set up i2c for the NFC reader */
    i2c_init(I2C_BUS, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    PN532_I2C_Init(&nfc, I2C_BUS, PN532_NO_PIN, PN532_NO_PIN);
    err = PN532_GetFirmwareVersion(&nfc, version);

    if(err != PN532_STATUS_OK)
        return -1;

    printf("Firmware version: %02x%02x%02x%02x\n", version[0], version[1],
           version[2], version[3]);

    err = PN532_SamConfiguration(&nfc);
    if(err != PN532_STATUS_OK)
        return -2;

    return 0;
}

int main(void) {
    int err;
    uint16_t syscode;
    uint8_t idm[8], pmm[8];

    stdio_init_all();

    sleep_ms(5000);
    printf("Initializing PN532...\n");
    if((err = init_pn532())) {
        printf("Trapping due to earlier error (code %d)!\n", err);
        for(;;) ;
    }

    for(;;) {
        printf("Polling for a FeliCa card!\n");
        err = PN532_FelicaRead(&nfc, FELICA_POLL_SYSTEM_CODE_ANY,
                               FELICA_POLL_SYSTEM_CODE, idm, pmm, &syscode, 2);
        if(err == PN532_STATUS_OK) {
            printf("System Code: %hu\n", syscode);
            printf("IDm: %02x %02x %02x %02x %02x %02x %02x %02x\n", idm[0],
                   idm[1], idm[2], idm[3], idm[4], idm[5], idm[6], idm[7]);
            printf("PMm: %02x %02x %02x %02x %02x %02x %02x %02x\n", pmm[0],
                   pmm[1], pmm[2], pmm[3], pmm[4], pmm[5], pmm[6], pmm[7]);
            sleep_ms(2000);
        }
        else if(err != PN532_STATUS_TIMEOUT) {
            printf("Unknown error occurred!\n");
        }
    }

    return 0;
}
