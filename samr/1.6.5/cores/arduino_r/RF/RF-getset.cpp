/*
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
 * @brief       Getter and setter functions for the AT86RF2xx drivers
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Baptiste Clenet <bapclenet@gmail.com>
 * @author      Daniel Krebs <github@daniel-krebs.net>
 *
 * @}
 */

#include "RF.h"


static const int16_t tx_pow_to_dbm[] = {4, 4, 3, 3, 2, 2, 1,
                                        0, -1, -2, -3, -4, -6, -8, -12, -17};
static const uint8_t dbm_to_tx_pow[] = {0x0f, 0x0f, 0x0f, 0x0e, 0x0e, 0x0e,
                                        0x0e, 0x0d, 0x0d, 0x0d, 0x0c, 0x0c,
                                        0x0b, 0x0b, 0x0a, 0x09, 0x08, 0x07,
                                        0x06, 0x05, 0x03,0x00};

uint16_t RF::get_addr_short()
{
    return (addr_short[0] << 8) | addr_short[1];
}

void RF::set_addr_short(uint16_t addr)
{
    addr_short[0] = addr >> 8;
    addr_short[1] = addr & 0xff;
    reg_write(RF_REG__SHORT_ADDR_0,
                        addr_short[0]);
    reg_write(RF_REG__SHORT_ADDR_1,
                        addr_short[1]);
}

uint64_t RF::get_addr_long()
{
    uint64_t addr;
    uint8_t *ap = (uint8_t *)(&addr);
    for (int i = 0; i < 8; i++) {
        ap[i] = addr_long[7 - i];
    }
    return addr;
}

void RF::set_addr_long(uint64_t addr)
{
    for (int i = 0; i < 8; i++) {
        addr_long[i] = (addr >> ((7 - i) * 8));
        reg_write((RF_REG__IEEE_ADDR_0 + i), addr_long[i]);
    }
}

uint8_t RF::get_chan()
{
    return chan;
}

void RF::set_chan(uint8_t channel)
{
    uint8_t tmp;

    if (channel < RF_MIN_CHANNEL
        || channel > RF_MAX_CHANNEL) {
        return;
    }
    chan = channel;
    tmp = reg_read(RF_REG__PHY_CC_CCA);
    tmp &= ~(RF_PHY_CC_CCA_MASK__CHANNEL);
    tmp |= (channel & RF_PHY_CC_CCA_MASK__CHANNEL);
    reg_write(RF_REG__PHY_CC_CCA, tmp);
}

uint16_t RF::get_pan()
{
    return pan;
}

void RF::set_pan(uint16_t pan_)
{
    pan = pan_;
    //DEBUG("pan0: %u, pan1: %u\n", (uint8_t)pan, pan >> 8);
    reg_write(RF_REG__PAN_ID_0, (uint8_t)pan);
    reg_write(RF_REG__PAN_ID_1, (pan >> 8));
}

int16_t RF::get_txpower()
{
    uint8_t txpower = reg_read(RF_REG__PHY_TX_PWR) & RF_PHY_TX_PWR_MASK__TX_PWR;
    return tx_pow_to_dbm[txpower];
}

void RF::set_txpower(int16_t txpower)
{
    txpower += 17;
    if (txpower < 0) {
        txpower = 0;
    }
    else if (txpower > 21) {
        txpower = 21;
    }

    reg_write(RF_REG__PHY_TX_PWR, dbm_to_tx_pow[txpower]);
}

uint8_t RF::get_max_retries()
{
    return (reg_read(RF_REG__XAH_CTRL_0) >> 4);
}

void RF::set_max_retries(uint8_t max)
{
    max = (max > 7) ? 7 : max;
    uint8_t tmp = reg_read(RF_REG__XAH_CTRL_0);
    tmp &= ~(RF_XAH_CTRL_0__MAX_FRAME_RETRIES);
    tmp |= (max << 4);
    reg_write(RF_REG__XAH_CTRL_0, tmp);
}

uint8_t RF::get_csma_max_retries()
{
    uint8_t tmp;
    tmp  = reg_read(RF_REG__XAH_CTRL_0);
    tmp &= RF_XAH_CTRL_0__MAX_CSMA_RETRIES;
    tmp >>= 1;
    return tmp;
}

void RF::set_csma_max_retries(int8_t retries)
{
    retries = (retries > 5) ? 5 : retries; /* valid values: 0-5 */
    retries = (retries < 0) ? 7 : retries; /* max < 0 => disable CSMA (set to 7) */
    //DEBUG("[at86rf2xx] opt: Set CSMA retries to %u\n", retries);

    uint8_t tmp = reg_read(RF_REG__XAH_CTRL_0);
    tmp &= ~(RF_XAH_CTRL_0__MAX_CSMA_RETRIES);
    tmp |= (retries << 1);
    reg_write(RF_REG__XAH_CTRL_0, tmp);
}

void RF::set_csma_backoff_exp(uint8_t min, uint8_t max)
{
    max = (max > 8) ? 8 : max;
    min = (min > max) ? max : min;
    //DEBUG("[at86rf2xx] opt: Set min BE=%u, max BE=%u\n", min, max);

    reg_write(RF_REG__CSMA_BE, (max << 4) | (min));
}

void RF::set_csma_seed(uint8_t entropy[2])
{
    if(entropy == NULL) {
        //DEBUG("[at86rf2xx] opt: CSMA seed entropy is nullpointer\n");
        return;
    }
    //DEBUG("[at86rf2xx] opt: Set CSMA seed to 0x%x 0x%x\n", entropy[0], entropy[1]);

    reg_write(RF_REG__CSMA_SEED_0, entropy[0]);

    uint8_t tmp = reg_read(RF_REG__CSMA_SEED_1);
    tmp &= ~(RF_CSMA_SEED_1__CSMA_SEED_1);
    tmp |= entropy[1] & RF_CSMA_SEED_1__CSMA_SEED_1;
    reg_write(RF_REG__CSMA_SEED_1, tmp);
}

void RF::set_option(uint16_t option, bool state)
{
    uint8_t tmp;

    //DEBUG("set option %i to %i\n", option, state);

    /* set option field */
    if (state) {
        options |= option;
        /* trigger option specific actions */
        switch (option) {
            case RF_OPT_CSMA:
                //DEBUG("[at86rf2xx] opt: enabling CSMA mode" \
                      "(4 retries, min BE: 3 max BE: 5)\n");
                /* Initialize CSMA seed with hardware address */
                set_csma_seed(addr_long);
                set_csma_max_retries(4);
                set_csma_backoff_exp(3, 5);
                break;
            case RF_OPT_PROMISCUOUS:
                //DEBUG("[at86rf2xx] opt: enabling PROMISCUOUS mode\n");
                /* disable auto ACKs in promiscuous mode */
                tmp = reg_read(RF_REG__CSMA_SEED_1);
                tmp |= RF_CSMA_SEED_1__AACK_DIS_ACK;
                reg_write(RF_REG__CSMA_SEED_1, tmp);
                /* enable promiscuous mode */
                tmp = reg_read(RF_REG__XAH_CTRL_1);
                tmp |= RF_XAH_CTRL_1__AACK_PROM_MODE;
                reg_write(RF_REG__XAH_CTRL_1, tmp);
                break;
            case RF_OPT_AUTOACK:
                //DEBUG("[at86rf2xx] opt: enabling auto ACKs\n");
                tmp = reg_read(RF_REG__CSMA_SEED_1);
                tmp &= ~(RF_CSMA_SEED_1__AACK_DIS_ACK);
                reg_write(RF_REG__CSMA_SEED_1, tmp);
                break;
            case RF_OPT_TELL_RX_START:
                //DEBUG("[at86rf2xx] opt: enabling SFD IRQ\n");
                tmp = reg_read(RF_REG__IRQ_MASK);
                tmp |= RF_IRQ_STATUS_MASK__RX_START;
                reg_write(RF_REG__IRQ_MASK, tmp);
                break;
            default:
                /* do nothing */
                break;
        }
    }
    else {
        options &= ~(option);
        /* trigger option specific actions */
        switch (option) {
            case RF_OPT_CSMA:
                //DEBUG("[at86rf2xx] opt: disabling CSMA mode\n");
                /* setting retries to -1 means CSMA disabled */
                set_csma_max_retries(-1);
                break;
            case RF_OPT_PROMISCUOUS:
                //DEBUG("[at86rf2xx] opt: disabling PROMISCUOUS mode\n");
                /* disable promiscuous mode */
                tmp = reg_read(RF_REG__XAH_CTRL_1);
                tmp &= ~(RF_XAH_CTRL_1__AACK_PROM_MODE);
                reg_write(RF_REG__XAH_CTRL_1, tmp);
                /* re-enable AUTOACK only if the option is set */
                if (options & RF_OPT_AUTOACK) {
                    tmp = reg_read(RF_REG__CSMA_SEED_1);
                    tmp &= ~(RF_CSMA_SEED_1__AACK_DIS_ACK);
                    reg_write(RF_REG__CSMA_SEED_1,
                                        tmp);
                }
                break;
            case RF_OPT_AUTOACK:
                //DEBUG("[at86rf2xx] opt: disabling auto ACKs\n");
                tmp = reg_read(RF_REG__CSMA_SEED_1);
                tmp |= RF_CSMA_SEED_1__AACK_DIS_ACK;
                reg_write(RF_REG__CSMA_SEED_1, tmp);
                break;
            case RF_OPT_TELL_RX_START:
                //DEBUG("[at86rf2xx] opt: disabling SFD IRQ\n");
                tmp = reg_read(RF_REG__IRQ_MASK);
                tmp &= ~RF_IRQ_STATUS_MASK__RX_START;
                reg_write(RF_REG__IRQ_MASK, tmp);
                break;
            default:
                /* do nothing */
                break;
        }
    }
}

inline void RF::_set_state(uint8_t state_)
{
    reg_write(RF_REG__TRX_STATE, state_);
    while (get_state() != state_);
    state = state_;
}

void RF::set_state(uint8_t state_)
{
    uint8_t old_state = get_state();

    if (state_ == old_state) {
        return;
    }
    /* make sure there is no ongoing transmission, or state transition already
     * in progress */
    while (old_state == RF_STATE_BUSY_RX_AACK ||
           old_state == RF_STATE_BUSY_TX_ARET ||
           old_state == RF_STATE_IN_PROGRESS) {
        old_state = get_state();
    }

    /* check if we need to wake up from sleep mode */
    if (old_state == RF_STATE_SLEEP ||
            old_state == RF_STATE_DEEP_SLEEP) {
        //DEBUG("at86rf2xx: waking up from sleep mode\n");
        assert_awake();
    }

    if (state_ == RF_STATE_SLEEP ||
            state_ == RF_STATE_DEEP_SLEEP) {
        /* First go to TRX_OFF */
        force_trx_off();

        if (state_ == RF_STATE_DEEP_SLEEP)
        {
            _set_state(RF_STATE_DEEP_SLEEP);
        }

        //_set_state(RF_STATE_TRX_OFF);
        /* Discard all IRQ flags, framebuffer is lost anyway */
        reg_read(RF_REG__IRQ_STATUS);
        /* Go to SLEEP mode from TRX_OFF */
        digitalWrite(sleep_pin, HIGH);
        state = state_;
    } else {
        _set_state(state_);
    }
}

void RF::reset_state_machine()
{
    uint8_t old_state;

    assert_awake();

    /* Wait for any state transitions to complete before forcing TRX_OFF */
    do {
        old_state = get_state();
    } while (old_state == RF_STATE_IN_PROGRESS);

    force_trx_off();
}
