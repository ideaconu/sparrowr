#include <RF.h>
#include <WRTC.h>

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

  rtc.begin();
  rtc.setAlarmSeconds(0);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(rtcAlarm);
}


void loop() {
  //sleep();
  return;

}

void rtcAlarm()
{
  SerialUSB.println("RTC ALARM");
}

