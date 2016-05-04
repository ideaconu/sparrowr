#include <RF.h>
int received = 0;
 
void setup() {

  USBDevice.init();
  pmSetVoltage(3200);
  USBDevice.attach();
 
  SerialUSB.begin(9600);
  SerialUSB.println("test");
   
  RFDevice.init();
  RFDevice.set_state(RF_STATE_RX_AACK_ON);
  RFDevice.set_chan(11);
}


void loop() {
  if (RFDevice.events || digitalRead(RF_IRQ) == 1)
    rf_eventHandler();
  delay(1000);
  SerialUSB.println(digitalRead(RF_IRQ));
  SerialUSB.println(RFDevice.events);
  SerialUSB.println("Waiting for data");
  return;
}

void rf_eventHandler() {
  /* One less event to handle! */
 // RFDevice.events--;

  SerialUSB.println("Event");
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
    SerialUSB.println("[RF] EVT - RX_START");

  /*  Done receiving radio frame; call our receive_data function.
   */
  if (irq_mask & RF_IRQ_STATUS_MASK__TRX_END)
  {
    if(state == RF_STATE_RX_AACK_ON || state == RF_STATE_BUSY_RX_AACK) {
      SerialUSB.println("[RF] EVT - RX_END");
      rf_receive_data();
    }
  }
}

void rf_receive_data() {
  /*  print the length of the frame
   *  (including the header)
   */
  SerialUSB.print("Reading frame length -- ");
  size_t pkt_len = RFDevice.rx_len();
  SerialUSB.print("Frame length: ");
  SerialUSB.print(pkt_len);
  SerialUSB.println(" bytes");

  /*  Print the frame, byte for byte  */
  SerialUSB.println("Frame dump (ASCII):");
  uint8_t data[pkt_len];
  RFDevice.rx_read(data, pkt_len, 0);
  for (int d=0; d<pkt_len; d++) 
    SerialUSB.print(data[d]); 
  SerialUSB.println();

  /* How many frames is this so far?  */
  SerialUSB.print("[[Total frames received: ");
  SerialUSB.print(++received);
  SerialUSB.println("]]\n");
}
