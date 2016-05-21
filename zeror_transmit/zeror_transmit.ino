int sent = 0;

volatile int rtcPeriodic = 0;

void setup() {

  pmSetVoltage(3200);
  //enableUSB();
 
  RFDevice.set_chan(11); // set channel to 26
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
    RFDevice.send(c,6);
    RFDevice.set_state(RF_STATE_DEEP_SLEEP);
  
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
  rtcPeriodic = 1;
}

void enableUSB(void)
{
  USBDevice.init();
  USBDevice.attach();
  SerialUSB.begin(9600);
}
