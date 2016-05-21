int received = 0;
volatile int perEvent = 0;
void setup() {

  pmSetVoltage(3200);
  USBDevice.init();
  USBDevice.attach();

  SerialUSB.begin(9600);
 
  RFDevice.set_state(RF_STATE_RX_AACK_ON);
  RFDevice.set_chan(11);

  rtc.enablePeriodicInterrupt(RTC_PER_1);
  rtc.attachPeriodicInterrupt(rtcPer);
}

void loop() {

  if(perEvent >= 1)
  {
    SerialUSB.println("----WAITING FOR DATA----");
    perEvent = -1;
  }
  while (RFDevice.available())
  {
    perEvent = -1;
    received ++;
    radio_buffer_t data;
    RFDevice.read_data(&data);

    SerialUSB.print(received);
    SerialUSB.print(" - Frame length: ");
    SerialUSB.print(data.len);
    SerialUSB.print(" - RSSI: ");
    SerialUSB.print(data.rssi);
    SerialUSB.print("dB, - LQI: ");
    SerialUSB.print(data.lqi);
    SerialUSB.print(", bytes  ---  ");

    for (int d = 0; d < data.len; d++)
    {
      SerialUSB.print(data.data[d]);
      SerialUSB.print(" ");
    }
    SerialUSB.println();

    perEvent = -4;
  }

}
void rtcPer()
{
  perEvent ++;
}
