#include <EEPROM.h>
#include <Wire.h>

int received = 0;
 

void setup() {


  pmSetVoltage(1800);
 // SPI.begin();
 // SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
  //Serial.begin(115200);

  //Wire.begin();
  
  pmSetVoltage(3200);
  USBDevice.init();
  USBDevice.attach();
  SerialUSB.begin(9600); 
   
  //RFDevice.set_state(RF_STATE_SLEEP);
  //RFDevice.set_state(RF_STATE_TRX_OFF);
    
  //rtc.setAlarmSeconds(0);
  //rtc.enableAlarm(RTC_MATCH_SS);
  //rtc.attachAlarmInterrupt(rtcAlarm);
  
  rtc.enablePeriodicInterrupt(RTC_PER_1);
  rtc.attachPeriodicInterrupt(evtPer);
  pinMode(2,OUTPUT); 
  //analogWrite(3,100);
  PM->RCAUSE;
}

uint32_t old_ms, new_ms;
byte x;
void loop() {
  //Wire.beginTransmission(4); // transmit to device #4
  //Wire.write("x is ");        // sends five bytes
  //Wire.write(x);              // sends one byte  
  //Wire.endTransmission();    // stop transmitting
  //sleep();
  //eeprom_test();
  //sleep();
  SerialUSB.println();
  return;
//  SPI.transfer(0xaa);
//  SPI.transfer(0xaa);
//  SPI.transfer(0xaa);
//  SPI.transfer(0xaa);
//  SPI.transfer(0xaa);
//  SPI.transfer(0xaa);
//  SPI.transfer(0xaa);
//  SPI.transfer(0xaa);
//  SPI.transfer(0xaa);
//  Serial.println("a1");
  delay(1000);
  //sleep();
}
 
void evtPer()
{ 
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
}

void eeprom_test()
{
  SerialUSB.println("test");
  uint8_t page_data[EEPROM_PAGE_SIZE];
  eeprom_emulator_read_page(0, page_data);
  SerialUSB.println(page_data[0]);
  
  page_data[0] ++;
  eeprom_emulator_write_page(0, page_data);
  eeprom_emulator_commit_page_buffer();
  
  eeprom_emulator_read_page(0, page_data);
  SerialUSB.println(page_data[0]);
  delay(4000);
}

