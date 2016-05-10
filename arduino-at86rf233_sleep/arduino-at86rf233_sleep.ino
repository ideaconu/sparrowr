#include <RF.h>
#include <events.h>

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
  
  
  events_init(EVSYS_ID_GEN_RTC_PER_5,EVSYS_ID_USER_DMAC_CH_0);
  events_attach_interrupt(EVSYS_ID_GEN_RTC_PER_5,evtPer);
  
  rtc.begin();
  rtc.setAlarmSeconds(0);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachAlarmInterrupt(rtcAlarm);
  
}

uint32_t old_ms, new_ms;
void loop() {
  //sleep();
  SerialUSB.println(EVSYS->INTFLAG.reg); 
  delay(1000); 
  return;

}

void evtPer()
{
  SerialUSB.println("per int");
}

void rtcAlarm()
{
  SerialUSB.println("RTC ALARM");
}

