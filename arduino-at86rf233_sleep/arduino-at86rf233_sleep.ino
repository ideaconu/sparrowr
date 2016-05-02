//#include <RTCZero.h>

#include <RF.h>
int received = 0;

//RTCZero rtc;

void setup() {

  //USBDevice.init();

  //pmSetVoltage(3200);
  //USBDevice.attach();

  //USBDevice.detach(); 
  //USBDevice.detach();
  //sleep();
  //SerialUSB.begin(9600);
  //SerialUSB.println("test");
  //delay(10);
  //rtc.begin();


  RFDevice.init();
  //sleep();

  //RFDevice.set_chan(11); // set channel to 26
  //RFDevice.set_state(RF_STATE_SLEEP);
  //RFDevice.set_state(RF_STATE_TRX_OFF);
  sleep();
}


void loop() {
  //sleep();
  //rtc.standbyMode();
  return;

  if (RFDevice.events)
    rf_eventHandler();
  delay(1000);
  uint8_t c[5] = {0,1,2,3,4};
  RFDevice.send(c,5);
  return;
}

void rf_eventHandler() {
  /* One less event to handle! */
  RFDevice.events--;

  /* If transceiver is sleeping register access is impossible and frames are
   * lost anyway, so return immediately.
   */
  byte state = RFDevice.get_status();
  if(state == RF_STATE_SLEEP)
    return;

  /* read (consume) device status */
  byte irq_mask = RFDevice.reg_read(RF_REG__IRQ_STATUS);

  /*  Incoming radio frame! */
  if (irq_mask & RF_IRQ_STATUS_MASK__RX_START)
    SerialUSB.println("[at86rf2xx] EVT - RX_START");

  /*  Done receiving radio frame; call our receive_data function.
   */
  if (irq_mask & RF_IRQ_STATUS_MASK__TRX_END)
  {
    if(state == RF_STATE_RX_AACK_ON || state == RF_STATE_BUSY_RX_AACK) {
      SerialUSB.println("[at86rf2xx] EVT - RX_END");
      at86rf2xx_receive_data();
    }
  }
}

void at86rf2xx_receive_data() {
  /*  print the length of the frame
   *  (including the header)
   */
  size_t pkt_len = RFDevice.rx_len();
  SerialUSB.print("Frame length: ");
  SerialUSB.print(pkt_len);
  SerialUSB.println(" bytes");

  /*  Print the frame, byte for byte  */
  SerialUSB.println("Frame dump (ASCII):");
  uint8_t data[pkt_len];
  RFDevice.rx_read(data, pkt_len, 0);
  for (int d=0; d<pkt_len; d++)
    SerialUSB.print((char)data[d]);
  SerialUSB.println();

  /* How many frames is this so far?  */
  SerialUSB.print("[[Total frames received: ");
  SerialUSB.print(++received);
  SerialUSB.println("]]\n");
}
