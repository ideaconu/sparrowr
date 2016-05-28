int received = 0;
volatile int perEvent = 0;

typedef struct {
  uint16_t supply_voltage;
  uint16_t solar_voltage;
  uint16_t send_freq;
  calendar_struct calendar;
} solar_t;

void setup() {

  pmSetVoltage(3200);
  USBDevice.init();
  USBDevice.attach();

  SerialUSB.begin(9600);
 
  RFDevice.set_state(RF_STATE_RX_AACK_ON);
  RFDevice.set_chan(20); 
  
  rtc.enablePeriodicInterrupt(RTC_PER_1);
  rtc.attachPeriodicInterrupt(rtcPer);
}

void loop() {
  // for some reason, a delay is needed in order to automaticaly reset the de board
  //delay(10);
  if(perEvent >= 1)
  {
    //SerialUSB.println("----WAITING FOR DATA----");
    perEvent = -1;
  }
  while (RFDevice.available())
  {
    solar_t *solar;
    perEvent = -1;
    received ++;
    radio_buffer_t data;
    RFDevice.read_data(&data);
    #if 0
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
#endif
    if (data.len == sizeof(solar_t))
    {
      solar = (solar_t*)data.data;
      SerialUSB.print(received);
      SerialUSB.print(", ");
      SerialUSB.print(solar->calendar.FIELD);
      SerialUSB.print(", ");
      SerialUSB.print(solar->calendar.year);
      SerialUSB.print(" ");
      SerialUSB.print(solar->calendar.month);
      SerialUSB.print(" ");
      SerialUSB.print(solar->calendar.day);
      SerialUSB.print(" ");
      SerialUSB.print(solar->calendar.hour);
      SerialUSB.print(":");
      SerialUSB.print(solar->calendar.minute);
      SerialUSB.print(":");
      SerialUSB.print(solar->calendar.second);
      SerialUSB.print(", supply mv: ");
      SerialUSB.print(solar->supply_voltage);
      SerialUSB.print(", solare mv: ");
      SerialUSB.print(solar->solar_voltage);
      SerialUSB.print(", send freq: ");
      SerialUSB.print(solar->send_freq);
      SerialUSB.println();
    }
    perEvent = -4;
  }

}
void rtcPer()
{
  perEvent ++;
}
