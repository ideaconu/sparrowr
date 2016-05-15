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

static void rf_receive_data() {
  size_t pkt_len = RFDevice.rx_len();
#if USB_PRINT
  SerialUSB.print("Frame length: ");
  SerialUSB.print(pkt_len);
  SerialUSB.println(" bytes");

  /*  Print the frame, byte for byte  */
  SerialUSB.println("Frame dump (ASCII):");
#endif

  uint8_t data[pkt_len];
  RFDevice.rx_read(data, pkt_len, 0);
  RFDevice.put(data,pkt_len);
#if USB_PRINT
  for (int d=0; d<pkt_len; d++)
    SerialUSB.print((char)data[d]);
  SerialUSB.println();

  /* How many frames is this so far?  */
  SerialUSB.print("[[Total frames received: ");
  SerialUSB.print(++received);
  SerialUSB.println("]]\n");
#endif
}

static void rf_eventHandler() {

  /* If transceiver is sleeping register access is impossible and frames are
   * lost anyway, so return immediately.
   */

  byte state = RFDevice.get_status();
  if(state == RF_STATE_SLEEP)
    return;

  /* read (consume) device status */
  byte irq_mask = RFDevice.reg_read(RF_REG__IRQ_STATUS);

#if USB_PRINT
  /*  Incoming radio frame! */
  if (irq_mask & RF_IRQ_STATUS_MASK__RX_START)
    SerialUSB.println("[at86rf2xx] EVT - RX_START");
#endif
  /*  Done receiving radio frame; call our receive_data function.
   */
  if (irq_mask & RF_IRQ_STATUS_MASK__TRX_END)
  {
      if(state == RF_STATE_RX_AACK_ON || state == RF_STATE_BUSY_RX_AACK) {
#if USB_PRINT
      SerialUSB.println("[at86rf2xx] EVT - RX_END");
#endif
      rf_receive_data();
    }
    else
    {
#if USB_PRINT
      SerialUSB.println("[at86rf2xx] EVT - TX_END");
#endif
    }
  }
}

/**
 * @brief   Increments events count by  1.
 */
static void rf_irq_handler()
{
    RFDevice.events ++;
    SerialUSB.println("RF int 1");
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
    state = RF_STATE_SLEEP;

    /* setup GPIOs */
    pinMode(reset_pin, OUTPUT);
    pinMode(sleep_pin, OUTPUT);
    pinMode(int_pin, INPUT_PULLDOWN);
    pinMode(cs_pin, OUTPUT);
    pinMode(RF_DIG1, INPUT_PULLDOWN);
    pinMode(RF_DIG2, INPUT_PULLDOWN);
    pinMode(RF_DIG3, INPUT_PULLDOWN);
    pinMode(RF_DIG4, INPUT_PULLDOWN);
    pinMode(RF_CLKM, INPUT_PULLDOWN);

    /* initialise SPI */
    //  Set up SPI
    SPI_TYPE.begin();
    //  Data is transmitted and received MSB first
    //  SPI interface will run at 7.5 max, and SamR21 will allow 6 MHz or 8MHz
    //  Data is clocked on the rising edge and clock is low when inactive
    SPI_TYPE.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));

    /*  wait for SPI to be ready  */
    delay(10);

    /*  initialize GPIOs */
    digitalWrite(sleep_pin, LOW);
    digitalWrite(reset_pin, HIGH);
    digitalWrite(cs_pin, HIGH);
    attachInterrupt(int_pin, rf_irq_handler, RISING);

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

    //Put RF to sleep for low power consumption
    set_state(RF_STATE_SLEEP);

    return 0;
}

void RF::reset()
{
    rx_new = NULL;
    rx_old = &rx_data[0];
    for(int i =0; i < RX_BUFF_NUM - 1;i++)
    {
        rx_data[i].next = &rx_data[i+1];
    }

    rx_data[RX_BUFF_NUM - 1].next = &rx_data[0];

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
    /* this disabled did not block the TX */
    set_option(RF_OPT_PROMISCUOUS, true);
    set_option(RF_OPT_AUTOACK, true);
    set_option(RF_OPT_CSMA, true);

    //RF_TRX_CTRL_1_MASK__TX_AUTO_CRC_ON

    /* enable safe mode (protect RX FIFO until reading data starts) */
    reg_write(RF_REG__TRX_CTRL_2, RF_TRX_CTRL_2_MASK__RX_SAFE_MODE);

    /* don't populate masked interrupt flags to IRQ_STATUS register */
    /*uint8_t tmp = at86rf2xx_reg_read(RF_REG__TRX_CTRL_1);
    tmp &= ~(RF_TRX_CTRL_1_MASK__IRQ_MASK_MODE);
    at86rf2xx_reg_write(RF_REG__TRX_CTRL_1, tmp);*/

    /* Auto FCS generation */
    //reg_write(RF_REG__TRX_CTRL_1, RF_TRX_CTRL_1_MASK__TX_AUTO_CRC_ON);


    /* disable clock output to save power */
    byte tmp = reg_read(RF_REG__TRX_CTRL_0);
    tmp &= ~(RF_TRX_CTRL_0_MASK__CLKM_CTRL);
    tmp &= ~(RF_TRX_CTRL_0_MASK__CLKM_SHA_SEL);
    tmp |= (RF_TRX_CTRL_0_CLKM_CTRL__OFF);
    reg_write(RF_REG__TRX_CTRL_0, tmp);

    /* enable interrupts */
    reg_write(RF_REG__IRQ_MASK, RF_IRQ_STATUS_MASK__TRX_END | RF_IRQ_STATUS_MASK__RX_START);

    /* clear interrupt flags */
    reg_read(RF_REG__IRQ_STATUS);

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

size_t RF::send(uint8_t *data, size_t len, size_t sleep_now)
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
    //This was commented when TX worked
    //RFDevice.reg_read(RF_REG__IRQ_STATUS);
    RF::tx_exec(sleep_now);
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

void RF::tx_exec(size_t sleepNow)
{
    /* write frame length field in FIFO */
    sram_write(0, &(frame_len), 1);
    /* trigger sending of pre-loaded frame */
    //reg_write(RF_REG__TRX_STATE, RF_TRX_STATE__TX_START);
    digitalWrite(sleep_pin, HIGH);
    delayMicroseconds(4);
    digitalWrite(sleep_pin, LOW);
    if(sleepNow)
    {
        sleep();
    }

        uint16_t timeout = 50*10;
        while(events == 0 && timeout > 0)
        {
            timeout --;
            delayMicroseconds(100);
        }
    events = 0;
    rf_eventHandler();
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

int RF::available()
{
    return rx_new != NULL;
}

void RF::read_data(radio_buffer_t *rf)
{
    __disable_irq();
    pop(rf);
    __enable_irq();
}

void RF::pop(radio_buffer_t *rf)
{
    if (rx_new == NULL)
    {
        rf->len = 0;
    }
    else
    {
        rf->len = rx_old->len;
        memcpy(rf->data,rx_old->data,rf->len);
        if (rx_old == rx_new)
        {
            rx_new = NULL;
        }
        else
        {
            rx_old = rx_old->next;
        }
    }
}

void RF::put(uint8_t *data, size_t len)
{
    if (rx_new == NULL)
    {
        rx_new = rx_old;
    }
    else
    {
        if(rx_new->next == rx_old)
        {
            rx_old = rx_old->next;
        }
        rx_new = rx_new->next;
    }
    rx_new->len = len;
    memcpy(rx_new->data, data, len);
}

void RF::handleEvents()
{
    rf_eventHandler();
}


