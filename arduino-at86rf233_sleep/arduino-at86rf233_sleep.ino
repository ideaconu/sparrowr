#include <RF.h> 
#include <SPI.h>
int received = 0;
 

void setup() {


  pmSetVoltage(1800);
 // SPI.begin();
 // SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
  //Serial.begin(115200);
  
  //USBDevice.init();
  //USBDevice.attach();
  //SerialUSB.begin(9600);
  //Serial1.begin(115200);
  RFDevice.init();
  
  //RFDevice.set_chan(11); // set channel to 26
  //RFDevice.set_state(RF_STATE_SLEEP);
  //RFDevice.set_state(RF_STATE_TRX_OFF);
   
  rtc.begin();
  rtc.setAlarmSeconds(15);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachAlarmInterrupt(rtcAlarm);
  rtc.enablePeriodicInterrupt(RTC_PER_1);
  rtc.attachPeriodicInterrupt(evtPer);
  pinMode(2,OUTPUT); 
  //analogWrite(3,100);
}

uint32_t old_ms, new_ms;
void loop() {
  //sleep();
  return;
  SPI.transfer(0xaa);
  SPI.transfer(0xaa);
  SPI.transfer(0xaa);
  SPI.transfer(0xaa);
  SPI.transfer(0xaa);
  SPI.transfer(0xaa);
  SPI.transfer(0xaa);
  SPI.transfer(0xaa);
  SPI.transfer(0xaa);
  Serial.println("a1");
  //delay(100);
  //sleep();
  //SerialUSB.println("0");
  //SerialUSB.println(((SystemCoreClock >> (PM->APBCSEL.reg) ) * 8) / (16 * 115200));
  //SerialUSB.println((SystemCoreClock >> (PM->APBCSEL.reg)));
  //SerialUSB.println(PM->APBASEL.reg);
  //SerialUSB.println(PM->APBBSEL.reg);
  //SerialUSB.println(PM->APBCSEL.reg);
  
}
 
void evtPer()
{
  //SerialUSB.println("per int");
  //Serial1.println("per int");
  if( received == 0)
  {
    digitalWrite(2,HIGH);
    received = 1;
  }
  else
  {
    digitalWrite(2,LOW);
    received = 0;
  }
}

void rtcAlarm()
{
    digitalWrite(2,LOW);
  //SerialUSB.println("RTC ALARM");
  //Serial1.println("RTC ALARM");
}

