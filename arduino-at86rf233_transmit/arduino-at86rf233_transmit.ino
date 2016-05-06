#include <RF.h>
int sent = 0;

void setup() {

  USBDevice.init();
  pmSetVoltage(3200);
  USBDevice.attach();
 
  SerialUSB.begin(9600);
  SerialUSB.println("test"); 
 
  RFDevice.init(); 
  RFDevice.set_state(RF_STATE_TRX_OFF);
  RFDevice.set_chan(11); // set channel to 26
}


void loop() {
 
  delay(1000);
  uint8_t c[6] = {0,1,2,3,4};
  c[5] = sent++;
  RFDevice.send(c,6); 
  SerialUSB.println("Sending data"); 
}
