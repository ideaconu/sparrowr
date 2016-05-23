int sent = 0;

volatile int rtcPeriodic = 0;

void setup() {

  pmSetVoltage(1800);
  //enableUSB();
 
  RFDevice.set_state(RF_STATE_TRX_OFF);
  RFDevice.set_chan(20); // set channel to 26
  pinMode(0,OUTPUT);
  rtc.enablePeriodicInterrupt(RTC_PER_1);
  rtc.attachPeriodicInterrupt(perInt);
}


void loop() {
  if (rtcPeriodic != 0)
  {
    uint8_t c[6] = {0,1,2,3,4};
    c[5] = sent++;
  
    RFDevice.set_state(RF_STATE_TRX_OFF);
    SerialUSB.println("Sending Data");
    RFDevice.send(c,6);
    SerialUSB.println("Sleep");
    RFDevice.set_state(RF_STATE_SLEEP);
  
    if(sent %2 == 0)
    {
      digitalWrite(0,HIGH);
    }
    else
    {
      digitalWrite(0,LOW);
    }
    rtcPeriodic = 0;
  }
  sleep();
}

void perInt(void)
{
  SerialUSB.println("perInt");
  rtcPeriodic = 1;
}

void enableUSB(void)
{
  USBDevice.init();
  USBDevice.attach();
  SerialUSB.begin(9600);
}

