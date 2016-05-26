#include <EEPROM.h>
#include <Wire.h>

int received = 0;
 

void setup() {


  pmSetVoltage(3200); 
  
  USBDevice.init();
  USBDevice.attach();
  SerialUSB.begin(9600); 
  pinMode(0,INPUT);
}

int prev_pin;
void loop() {
  int pin = digitalRead(0);
  if (pin != prev_pin)
  {
    SerialUSB.println(pin);
    prev_pin = pin;
  }
  delay(10); 
}
 
