/*
   pn532_pico.c
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "pico.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "pn532_pico.h"

static uint _req_pin = 0xff;
static uint _reset_pin = 0xff;
static i2c_inst_t *_i2c = NULL;

#define I2C_ADDR                (0x48 >> 1)

/* Currently, only I2C mode is supported here. */
static int PN532_Reset(void) {
    if(_reset_pin != PN532_NO_PIN) {
        gpio_put(_reset_pin, true);
        sleep_ms(100);
        gpio_put(_reset_pin, false);
        sleep_ms(100);
    }

    return PN532_STATUS_OK;
}

static void PN532_Log(const char *log) {
    printf("%s\n", log);
}

static int PN532_I2C_ReadData(uint8_t* data, uint16_t sz) {
    uint8_t frame[sz + 1];

    if(i2c_read_blocking(_i2c, I2C_ADDR, frame, 1, false) != 1)
        return PN532_STATUS_ERROR;

    if(frame[0] != 1)
        return PN532_STATUS_ERROR;

    if(i2c_read_blocking(_i2c, I2C_ADDR, frame, sz + 1, false) != sz + 1)
        return PN532_STATUS_ERROR;

    memcpy(data, frame + 1, sz);
    return PN532_STATUS_OK;
}

static int PN532_I2C_WriteData(uint8_t *data, uint16_t count) {
    if(i2c_write_blocking(_i2c, I2C_ADDR, data, count, false) != count)
        return PN532_STATUS_ERROR;

    return PN532_STATUS_OK;
}

static bool PN532_I2C_WaitReady(uint32_t timeout) {
    uint8_t status;
    uint32_t elapsed = 0;

    timeout *= 1000;

    for(;;) {
        if(i2c_read_blocking(_i2c, I2C_ADDR, &status, 1, false) == 1) {
            if(status == 1)
                return true;
        }

        if(elapsed + 5 > timeout)
            return false;

        sleep_ms(5);
        elapsed += 5;
    }
}

static int PN532_I2C_Wakeup(void) {
    if(_req_pin != PN532_NO_PIN) {
        gpio_put(_req_pin, false);
        sleep_ms(100);
        gpio_put(_req_pin, true);
        sleep_ms(100);
    }

    return PN532_STATUS_OK;
}

void PN532_I2C_Init(PN532 *pn532, i2c_inst_t *i2c, uint reset_pin,
                    uint req_pin) {
    pn532->reset = &PN532_Reset;
    pn532->read_data = &PN532_I2C_ReadData;
    pn532->write_data = &PN532_I2C_WriteData;
    pn532->wait_ready = &PN532_I2C_WaitReady;
    pn532->wakeup = &PN532_I2C_Wakeup;
    pn532->log = &PN532_Log;

    _req_pin = req_pin;
    _reset_pin = reset_pin;
    _i2c = i2c;

    if(_req_pin != PN532_NO_PIN)
        gpio_set_dir(_req_pin, true);
    if(_reset_pin != PN532_NO_PIN)
        gpio_set_dir(_reset_pin, true);

    pn532->reset();
    pn532->wakeup();
}
