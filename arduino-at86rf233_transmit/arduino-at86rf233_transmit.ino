#include <RF.h>
int sent = 0;

void setup() {

  pmSetVoltage(3200);
  //USBDevice.init();
  //USBDevice.attach();
 
  //SerialUSB.begin(9600);
  //SerialUSB.println("test"); 
 
  RFDevice.init(); 
  RFDevice.set_state(RF_STATE_TRX_OFF);
  RFDevice.set_chan(11); // set channel to 26
}


void loop() {
 
  delay(250);
  uint8_t c[6] = {0,1,2,3,4};
  c[5] = sent++;
  //RFDevice.set_state(RF_STATE_TRX_OFF);
  RFDevice.send(c,6); 
  //delay(20);
  //RFDevice.set_state(RF_STATE_SLEEP);
  //SerialUSB.println("Sending data"); 
}
