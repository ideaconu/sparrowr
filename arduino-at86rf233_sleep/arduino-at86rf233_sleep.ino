#include <SPI.h>
#include "at86rf2xx.h"
#include <RTCZero.h>


int received = 0;
 
RTCZero rtc; 
 
void setup() {
  //USBDevice.detach();
  //sleep();
  //SerialUSB.begin(9600);
  //SerialUSB.println("test"); 
  //delay(10000);
  delay(12);
  //rtc.begin();
  
  at86rf2xx.init(RF_SEL, RF_IRQ, RF_SLP_TR, RF_RESET);
  //at86rf2xx.set_chan(11); // set channel to 26
  at86rf2xx.set_state(AT86RF2XX_STATE_SLEEP); 
  //at86rf2xx.set_state(AT86RF2XX_STATE_TRX_OFF);   
  //sleep();
}


void loop() {
  //sleep();
  //rtc.standbyMode(); 
  
  return;
  
  if (at86rf2xx.events)
    at86rf2xx_eventHandler();
  delay(1000);
  uint8_t c[5] = {0,1,2,3,4};
  at86rf2xx.send(c,5);
  return;
}

void sleep()
{  
  //USBDevice.detach(); 
  //SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
  //NVMCTRL->CTRLB.bit.SLEEPPRM = 3;
  __DMB();
  __enable_irq();
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __DSB();
  __WFI();
  //USBDevice.attach();
}

void at86rf2xx_eventHandler() {
  /* One less event to handle! */
  at86rf2xx.events--;

  /* If transceiver is sleeping register access is impossible and frames are
   * lost anyway, so return immediately.
   */
  byte state = at86rf2xx.get_status();
  if(state == AT86RF2XX_STATE_SLEEP)
    return;

  /* read (consume) device status */
  byte irq_mask = at86rf2xx.reg_read(AT86RF2XX_REG__IRQ_STATUS);

  /*  Incoming radio frame! */
  if (irq_mask & AT86RF2XX_IRQ_STATUS_MASK__RX_START)
    SerialUSB.println("[at86rf2xx] EVT - RX_START");

  /*  Done receiving radio frame; call our receive_data function.
   */
  if (irq_mask & AT86RF2XX_IRQ_STATUS_MASK__TRX_END)
  {
    if(state == AT86RF2XX_STATE_RX_AACK_ON || state == AT86RF2XX_STATE_BUSY_RX_AACK) {
      SerialUSB.println("[at86rf2xx] EVT - RX_END");
      at86rf2xx_receive_data();
    }
  }
}

void at86rf2xx_receive_data() {
  /*  print the length of the frame
   *  (including the header)
   */
  size_t pkt_len = at86rf2xx.rx_len();
  SerialUSB.print("Frame length: ");
  SerialUSB.print(pkt_len);
  SerialUSB.println(" bytes");

  /*  Print the frame, byte for byte  */
  SerialUSB.println("Frame dump (ASCII):");
  uint8_t data[pkt_len];
  at86rf2xx.rx_read(data, pkt_len, 0);
  for (int d=0; d<pkt_len; d++)
    SerialUSB.print((char)data[d]);
  SerialUSB.println();

  /* How many frames is this so far?  */
  SerialUSB.print("[[Total frames received: ");
  SerialUSB.print(++received);
  SerialUSB.println("]]\n");
}
