int sent = 0;

volatile int rtcPeriodic = 0;

int received = 0;
void setup() {

  pmSetVoltage(1800);
  //enableUSB();
  RFDevice.set_state(RF_STATE_TRX_OFF);
  RFDevice.set_chan(20); // set channel to 26
 
  rtc.enablePeriodicInterrupt(RTC_PER_1);
  rtc.attachPeriodicInterrupt(perInt);
  //sleepMode(SLEEP_STANDBY);
}


void loop() {
  if (rtcPeriodic != 0)
  {
    uint8_t c[6] = {0, 1, 2, 3, 4};
    c[5] = sent++;

    RFDevice.send(c, 6);
    RFDevice.set_state(RF_STATE_SLEEP);
    rtcPeriodic = 0;
  }
  sleep();
}
 
void perInt(void)
{ 
  rtcPeriodic = 1;
  
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

void enableUSB(void)
{
  USBDevice.init();
  USBDevice.attach();
  SerialUSB.begin(9600);
}

