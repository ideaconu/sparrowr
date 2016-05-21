/*
 * Copyright (C) 2013 Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_at86rf2xx
 * @{
 *
 * @file
 * @brief       Implementation of driver internal functions
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Mark Solters <msolters@gmail.com>
 *
 * @}
 */

#include "RF.h"

void RF::reg_write(const uint8_t addr,
                         const uint8_t value)
{
    byte writeCommand = addr | RF_ACCESS_REG | RF_ACCESS_WRITE;
    digitalWrite(cs_pin, LOW);
    SPI_TYPE.transfer(writeCommand);
    SPI_TYPE.transfer(value);
    digitalWrite(cs_pin, HIGH);
}

uint8_t RF::reg_read(const uint8_t addr)
{
    byte value;
    byte readCommand = addr | RF_ACCESS_REG | RF_ACCESS_READ;
    digitalWrite(cs_pin, LOW);
    SPI_TYPE.transfer(readCommand);
    value = SPI_TYPE.transfer(0x00);
    digitalWrite(cs_pin, HIGH);

    return (uint8_t)value;
}

void RF::sram_read(const uint8_t offset,
                         uint8_t *data,
                         const size_t len)
{
    byte readCommand = RF_ACCESS_SRAM | RF_ACCESS_READ;
    digitalWrite(cs_pin, LOW);
    SPI_TYPE.transfer(readCommand);
    SPI_TYPE.transfer((char)offset);
    for (int b=0; b<len; b++) {
      data[b] = SPI_TYPE.transfer(0x00);
    }
    digitalWrite(cs_pin, HIGH);
}

void RF::sram_write(const uint8_t offset,
                          const uint8_t *data,
                          const size_t len)
{
    byte writeCommand = RF_ACCESS_SRAM | RF_ACCESS_WRITE;
    digitalWrite(cs_pin, LOW);
    SPI_TYPE.transfer(writeCommand);
    SPI_TYPE.transfer((char)offset);
    for (int b=0; b<len; b++) {
      SPI_TYPE.transfer(data[b]);
    }
    digitalWrite(cs_pin, HIGH);
}

void RF::fb_read(uint8_t *data,
                       const size_t len)
{
    byte readCommand = RF_ACCESS_FB | RF_ACCESS_READ;
    digitalWrite(cs_pin, LOW);
    SPI_TYPE.transfer(readCommand);
    for (int b=0; b<len; b++) {
      data[b] = SPI_TYPE.transfer(0x00);
    }
    digitalWrite(cs_pin, HIGH);
}

uint8_t RF::get_state()
{
    /* if sleeping immediately return state */
    if(state == RF_STATE_SLEEP ||
            state == RF_STATE_DEEP_SLEEP)
        return state;

    return reg_read(RF_REG__TRX_STATUS) & RF_TRX_STATUS_MASK__TRX_STATUS;
}

void RF::assert_awake()
{
    int state_ = get_state();
    if(state_ == RF_STATE_SLEEP ||
            state_ == RF_STATE_DEEP_SLEEP) {
        /* wake up and wait for transition to TRX_OFF */
        digitalWrite(sleep_pin, LOW);
        delayMicroseconds(RF_WAKEUP_DELAY);

        if (state_ == RF_STATE_DEEP_SLEEP)
        {
            initDefaults();
        }
        /* update state */
        state = reg_read(RF_REG__TRX_STATUS) & RF_TRX_STATUS_MASK__TRX_STATUS;
    }
}

void RF::hardware_reset()
{
    /* wake up from sleep in case radio is sleeping */
    //delayMicroseconds(50); // Arduino seems to hang without some minimum pause here
    assert_awake();

    /* trigger hardware reset */

    digitalWrite(reset_pin, LOW);
    delayMicroseconds(RF_RESET_PULSE_WIDTH);
    digitalWrite(reset_pin, HIGH);
    delayMicroseconds(RF_RESET_DELAY);
}

void RF::force_trx_off()
{
    reg_write(RF_REG__TRX_STATE, RF_TRX_STATE__FORCE_TRX_OFF);
    while (get_state() != RF_STATE_TRX_OFF);
}
