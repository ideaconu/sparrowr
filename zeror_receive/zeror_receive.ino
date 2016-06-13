int received = 0;
volatile int perEvent = 0;

typedef struct __attribute__((packed)){
  uint32_t nothing; //4
  uint32_t total_sent; //4
  uint32_t start_timestamp; //4
  uint32_t end_timestamp; //4
  
  uint32_t target_time; //4
  
  uint32_t current_timestamp; //4
  uint16_t current_voltage; //2
  
  uint32_t time_changed; //4
  uint16_t voltage_changed; //2
  uint16_t send_freq; //2
  uint16_t actual_freq; //2
  uint8_t state; //1
  uint32_t estimated; //4
  uint32_t remaining; //4
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
      SerialUSB.print(solar->current_timestamp);
      SerialUSB.print(", supply mv: ");
      SerialUSB.print(solar->current_voltage);
      SerialUSB.print(",");
      SerialUSB.print(solar->state);
      SerialUSB.print(", send freq: ");
      SerialUSB.print(solar->send_freq);
      SerialUSB.print(", actual_freq: ");
      SerialUSB.print(solar->actual_freq);
      SerialUSB.print(", voltage changed ");
      SerialUSB.print(solar->voltage_changed);
      SerialUSB.print(", ");
      SerialUSB.print(solar->target_time);
      SerialUSB.print(", ");
      SerialUSB.print(solar->start_timestamp);
      SerialUSB.print(", time_changed ");
      SerialUSB.print(solar->time_changed);
      SerialUSB.print(", end time ");
      SerialUSB.print(solar->end_timestamp);
      SerialUSB.print(", est ");
      SerialUSB.print(solar->estimated);
      SerialUSB.print(", remain ");
      SerialUSB.print(solar->remaining);
      SerialUSB.print(", ");
      SerialUSB.print(solar->total_sent);
      
    int delta_time = solar->current_timestamp - solar->time_changed;
    int absolute_delta_time = solar->end_timestamp - solar->start_timestamp;
      SerialUSB.print(", delta_time ");
      SerialUSB.print(delta_time);
      SerialUSB.print(", absolute_delta_time ");
      SerialUSB.print(absolute_delta_time);
      SerialUSB.println();
    }
    perEvent = -4;
  }

}
void rtcPer()
{
  perEvent ++;
}
