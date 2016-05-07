#include <RF.h>
int received = 0;
void setup() {

  pmSetVoltage(3200);
  USBDevice.init();
  USBDevice.attach();

  SerialUSB.begin(9600);

  RFDevice.init();
  RFDevice.set_state(RF_STATE_RX_AACK_ON);
  RFDevice.set_chan(11);
}

void loop() {

  RFDevice.handleEvents();
  SerialUSB.println("Waiting for data");

  while (RFDevice.available())
  {
    radio_buffer_t data;
    RFDevice.read_data(&data);

    size_t pkt_len = RFDevice.rx_len();
    SerialUSB.print("Frame length: ");
    SerialUSB.print(pkt_len);
    SerialUSB.print(" bytes  ");

    for (int d = 0; d < data.len; d++)
    {
      SerialUSB.print(data.data[d]);
      SerialUSB.print(" ");
    }
    SerialUSB.println();
  }
  delay(1001);
  return;
}
