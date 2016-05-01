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
 * @brief       Implementation of public functions for AT86RF2xx drivers
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Mark Solters <msolters@gmail.com>
 *
 * @}
 */

#include <Arduino.h>
#include "RF.h"

#define USB_PRINT 0

/*  Declare radio device as globally scoped struct  */
RF RFDevice = RF();

/**
 * @brief   Increments events count by  1.
 */
static void rf_irq_handler()
{
    RFDevice.events++;
    return;
}

RF::RF() {}

int RF::init()
{

    #if USB_PRINT
    SerialUSB.println("[at86rf2xx] Booting radio device.");
    #endif

    /* initialize device descriptor */
    cs_pin = RF_SEL;
    int_pin = RF_IRQ;
    sleep_pin = RF_SLP_TR;
    reset_pin = RF_RESET;
    idle_state = RF_STATE_TRX_OFF;
    state = RF_STATE_SLEEP;

    /* setup GPIOs */
    pinMode(reset_pin, OUTPUT);
    pinMode(sleep_pin, OUTPUT);
    pinMode(int_pin, INPUT_PULLDOWN);
    pinMode(cs_pin, OUTPUT);
    //pinMode(RF_DIG1, INPUT_PULLDOWN);
    //pinMode(RF_DIG2, INPUT_PULLDOWN);
    //pinMode(RF_DIG3, INPUT_PULLDOWN);
    //pinMode(RF_DIG4, INPUT_PULLDOWN);
    //pinMode(RF_CLKM, INPUT_PULLDOWN);

    /* initialise SPI */
    //  Set up SPI
    SPI_TYPE.begin();
    //  Data is transmitted and received MSB first
    //SPI.setBitOrder(MSBFIRST);
    //  SPI interface will run at 1MHz if 8MHz chip or 2Mhz if 16Mhz
    //SPI.setClockDivider(SPI_CLOCK_DIV8);
    //  Data is clocked on the rising edge and clock is low when inactive
    //SPI.setDataMode(SPI_MODE0);
    SPI_TYPE.beginTransaction(SPISettings(7500000, MSBFIRST, SPI_MODE0));

    /*  wait for SPI to be ready  */
    delay(10);

    /*  initialize GPIOs */
    digitalWrite(sleep_pin, LOW);
    digitalWrite(reset_pin, HIGH);
    digitalWrite(cs_pin, HIGH);
    attachInterrupt(digitalPinToInterrupt(int_pin), rf_irq_handler, RISING);

    /* make sure device is not sleeping, so we can query part number */
    assert_awake();

    /* test if the SPI is set up correctly and the device is responding */
    byte part_num = reg_read(RF_REG__PART_NUM);
    if (part_num != RF_PARTNUM) {
        #if USB_PRINT
        SerialUSB.print("[at86rf2xx] Error: unable to read correct part number. ");
        SerialUSB.println(part_num);
        #endif
        return -1;
    }

    #if USB_PRINT
    SerialUSB.print("[at86rf2xx] Detected part #: 0x");
    SerialUSB.println(part_num, HEX);
    SerialUSB.print("[at86rf2xx] Version: 0x");
    SerialUSB.println(reg_read(RF_REG__VERSION_NUM), HEX);
    #endif

    /* reset device to default values and put it into RX state */
    reset();

    force_trx_off();

    //Put RF to sleep for low power consumption
    set_state(RF_STATE_SLEEP);

    return 0;
}

void RF::reset()
{
    hardware_reset();

    /* Reset state machine to ensure a known state */
    reset_state_machine();

    /* reset options and sequence number */
    seq_nr = 0;
    options = 0;

    /* set short and long address */
    set_addr_long(RF_DEFAULT_ADDR_LONG);
    set_addr_short(RF_DEFAULT_ADDR_SHORT);

    /* set default PAN id */
    set_pan(RF_DEFAULT_PANID);

    /* set default channel */
    set_chan(RF_DEFAULT_CHANNEL);

    /* set default TX power */
    set_txpower(RF_DEFAULT_TXPOWER);

    /* set default options */
    set_option(RF_OPT_PROMISCUOUS, true);
    set_option(RF_OPT_AUTOACK, true);
    set_option(RF_OPT_CSMA, true);
    set_option(RF_OPT_TELL_RX_START, true);
    set_option(RF_OPT_TELL_RX_END, true);

    /* enable safe mode (protect RX FIFO until reading data starts) */
    reg_write(RF_REG__TRX_CTRL_2, RF_TRX_CTRL_2_MASK__RX_SAFE_MODE);

//#ifdef MODULE_AT86RF212B
//    at86rf2xx_set_freq(dev, RF_FREQ_915MHZ);
//#endif

    /* don't populate masked interrupt flags to IRQ_STATUS register */
    /*uint8_t tmp = at86rf2xx_reg_read(RF_REG__TRX_CTRL_1);
    tmp &= ~(RF_TRX_CTRL_1_MASK__IRQ_MASK_MODE);
    at86rf2xx_reg_write(RF_REG__TRX_CTRL_1, tmp);*/

    /* disable clock output to save power */
    byte tmp = reg_read(RF_REG__TRX_CTRL_0);
    tmp &= ~(RF_TRX_CTRL_0_MASK__CLKM_CTRL);
    tmp &= ~(RF_TRX_CTRL_0_MASK__CLKM_SHA_SEL);
    tmp |= (RF_TRX_CTRL_0_CLKM_CTRL__OFF);
    reg_write(RF_REG__TRX_CTRL_0, tmp);

    /* enable interrupts */
    reg_write(RF_REG__IRQ_MASK, RF_IRQ_STATUS_MASK__TRX_END);

    /* clear interrupt flags */
    reg_read(RF_REG__IRQ_STATUS);

    /* go into RX state */
    set_state(RF_STATE_RX_AACK_ON);

    #if USB_PRINT
    SerialUSB.println("[at86rf2xx] Reset complete.");
    #endif
}

bool RF::cca()
{
    uint8_t tmp;
    uint8_t status;

    assert_awake();

    /* trigger CCA measurment */
    tmp = reg_read(RF_REG__PHY_CC_CCA);
    tmp &= RF_PHY_CC_CCA_MASK__CCA_REQUEST;
    reg_write(RF_REG__PHY_CC_CCA, tmp);

    /* wait for result to be ready */
    do {
        status = reg_read(RF_REG__TRX_STATUS);
    } while (!(status & RF_TRX_STATUS_MASK__CCA_DONE));

    /* return according to measurement */
    if (status & RF_TRX_STATUS_MASK__CCA_STATUS) {
        return true;
    }
    else {
        return false;
    }
}

size_t RF::send(uint8_t *data, size_t len)
{
    /* check data length */
    if (len > RF_MAX_PKT_LENGTH) {

        #if USB_PRINT
        SerialUSB.println("[at86rf2xx] Error: Data to send exceeds max packet size.");
        #endif
        return 0;
    }
    RF::tx_prepare();
    RF::tx_load(data, len, 0);
    RF::tx_exec();
    return len;
}

void RF::tx_prepare()
{
    uint8_t state;

    /* make sure ongoing transmissions are finished */
    do {
        state = get_status();
    }
    while (state == RF_STATE_BUSY_TX_ARET);

    /* if receiving cancel */
    if(state == RF_STATE_BUSY_RX_AACK) {
        force_trx_off();
        idle_state = RF_STATE_RX_AACK_ON;
    } else if (state != RF_STATE_TX_ARET_ON) {
        idle_state = state;
    }
    set_state(RF_STATE_TX_ARET_ON);
    frame_len = IEEE802154_FCS_LEN;
}

size_t RF::tx_load(uint8_t *data,
                         size_t len, size_t offset)
{
    frame_len += (uint8_t)len;
    sram_write(offset + 1, data, len);
    return offset + len;
}

void RF::tx_exec()
{
    /* write frame length field in FIFO */
    sram_write(0, &(frame_len), 1);
    /* trigger sending of pre-loaded frame */
    reg_write(RF_REG__TRX_STATE, RF_TRX_STATE__TX_START);
    /*if (at86rf2xx.event_cb && (at86rf2xx.options & RF_OPT_TELL_TX_START)) {
        at86rf2xx.event_cb(NETDEV_EVENT_TX_STARTED, NULL);
    }*/
}

size_t RF::rx_len()
{
    uint8_t phr;
    fb_read(&phr, 1);

    /* ignore MSB (refer p.80) and substract length of FCS field */
    return (size_t)((phr & 0x7f) - 2);
}

void RF::rx_read(uint8_t *data, size_t len, size_t offset)
{
    /* when reading from SRAM, the different chips from the AT86RF2xx family
     * behave differently: the AT86F233, the AT86RF232 and the ATRF86212B return
     * frame length field (PHR) at position 0 and the first data byte at
     * position 1.
     * The AT86RF231 does not return the PHR field and return
     * the first data byte at position 0.
     */
    sram_read(offset + 1, data, len);
}
