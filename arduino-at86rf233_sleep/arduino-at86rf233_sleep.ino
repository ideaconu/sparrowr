#include <RF.h> 

int received = 0;
 

void setup() {


  pmSetVoltage(1800);
  //USBDevice.init();
  //USBDevice.attach();

  //USBDevice.detach();
  //USBDevice.detach();
  //sleep();
  //SerialUSB.begin(9600);
  //Serial1.begin(115200);
  RFDevice.init();
  //sleep();

  //RFDevice.set_chan(11); // set channel to 26
  //RFDevice.set_state(RF_STATE_SLEEP);
  //RFDevice.set_state(RF_STATE_TRX_OFF);
  
  pinMode(0,OUTPUT);
  pinMode(1,INPUT_PULLUP);
  attachInterrupt(1,pinPer, FALLING);
  rtc.begin();
  rtc.setAlarmSeconds(15);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachAlarmInterrupt(rtcAlarm);
  rtc.enablePeriodicInterrupt(RTC_PER_1);
  rtc.attachPeriodicInterrupt(evtPer);
  sleepMode(SLEEP_STANDBY);
  digitalWrite(0,HIGH);
  //sleep();
  //delay(5000);
  
  //USBDevice.init();
  //USBDevice.attach();
  //SerialUSB.begin(9600);
}

uint32_t old_ms, new_ms;
void loop() {
  sleep();
  //SerialUSB.println("0");
  //Serial1.println("0");
  //SerialUSB.println(rtc.getSeconds());
}

void pinPer()
{ 
  //SerialUSB.println("btn int");
  digitalWrite(0,LOW);
}
void evtPer()
{
  //SerialUSB.println("per int");
  //Serial1.println("per int");
  if( received == 0)
  {
    digitalWrite(0,HIGH);
    received = 1;
  }
  else
  {
    digitalWrite(0,LOW);
    received = 0;
  }
}

void rtcAlarm()
{
    digitalWrite(0,LOW);
  //SerialUSB.println("RTC ALARM");
  //Serial1.println("RTC ALARM");
}

