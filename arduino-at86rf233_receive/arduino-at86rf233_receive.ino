#include <RF.h>
int received = 0;
volatile int perEvent = 0;
void setup() {

  pmSetVoltage(3200);
  USBDevice.init();
  USBDevice.attach();

  SerialUSB.begin(9600);

  RFDevice.init();
  RFDevice.set_state(RF_STATE_RX_AACK_ON);
  RFDevice.set_chan(11);
  rtc.begin();
  rtc.enablePeriodicInterrupt(RTC_PER_1);
  rtc.attachPeriodicInterrupt(rtcPer);
  sleepMode(SLEEP_IDLE_2);
}

void loop() {
  RFDevice.handleEvents();
  if(perEvent >= 1)
  {
    SerialUSB.println("----WAITING FOR DATA---");
    perEvent = -1;
  }
  while (RFDevice.available())
  {
    perEvent = -1;
    received ++;
    radio_buffer_t data;
    RFDevice.read_data(&data);

    size_t pkt_len = RFDevice.rx_len();
    SerialUSB.print(received);
    SerialUSB.print(" - Frame length: ");
    SerialUSB.print(pkt_len);
    SerialUSB.print(" bytes, buffer len:  ");
    SerialUSB.print(data.len);
    SerialUSB.print(" bytes  ---  ");

    for (int d = 0; d < data.len; d++)
    {
      SerialUSB.print(data.data[d]);
      SerialUSB.print(" ");
    }
    SerialUSB.println();
    perEvent = -4;
  }
  return;
}
void rtcPer()
{
  perEvent ++;
}
