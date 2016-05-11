#include <RF.h>
int sent = 0;

void setup() {

  pmSetVoltage(3200);
  //enableUSB();
  //SerialUSB.println("test"); 
 
  RFDevice.init(); 
  RFDevice.set_state(RF_STATE_TRX_OFF);
  RFDevice.set_chan(11); // set channel to 26
  pinMode(0,OUTPUT);
  rtc.begin();
  rtc.enablePeriodicInterrupt(RTC_PER_1);
}


void loop() {
  //delay(252);
  uint8_t c[6] = {0,1,2,3,4};
  c[5] = sent++;
 
  RFDevice.set_state(RF_STATE_TRX_OFF);
  RFDevice.send(c,6,1);   
  RFDevice.set_state(RF_STATE_SLEEP);
    
  if(sent %2 == 0)
  {
    digitalWrite(0,HIGH);
  }
  else
  {
    digitalWrite(0,LOW);
  }
  sleep();
}

void enableUSB()
{
  USBDevice.init();
  USBDevice.attach();

  SerialUSB.begin(9600);
}

