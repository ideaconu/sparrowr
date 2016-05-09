#include <RF.h>

int received = 0;


 
void setup() {


  pmSetVoltage(3200);
  USBDevice.init();
  USBDevice.attach();

  //USBDevice.detach(); 
  //USBDevice.detach();
  //sleep();
  SerialUSB.begin(9600);

  RFDevice.init();
  //sleep();

  //RFDevice.set_chan(11); // set channel to 26
  //RFDevice.set_state(RF_STATE_SLEEP);
  //RFDevice.set_state(RF_STATE_TRX_OFF);
  
  rtc.setAlarmSeconds(0);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachAlarmInterrupt(rtcAlarm);
}

uint32_t old_ms, new_ms;
void loop() {
  //sleep();
  //SerialUSB.println("delay 1s");
  
  SerialUSB.println(new_ms - old_ms);
  old_ms = millis();
  delay(1000);
  new_ms = millis();
  return;

}

void rtcAlarm()
{
  SerialUSB.println("RTC ALARM");
}

