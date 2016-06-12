#define TOTAL_DAYS          (3)
#define TOTAL_HOURS         (TOTAL_DAYS*24)
#define SECONDS_PER_HOUR    (60*60)

#define DEFAULT_TARGET_TIME (2*60*60)
#define DECREASE_EFFICIENY  (80)

#define MIN_SEND_FREQ       (1)
#define MAX_SEND_FREQ       (3600)
#define DEFAULT_SEND_FREQ   (3600)

#define MIN_WAIT_TIME       (119)
#define MIN_COMPUTE_TIME    (MIN_WAIT_TIME+119)
#define MIN_VOLTAGE         (2050)

#define CHARGING            (0)
#define DISCHARGING         (1)
#define CHARGING_DELTA_START  (40)
#define DISCHARGE_DELTA_START (-45)

typedef struct {

  uint16_t start_voltage; //2
  uint16_t end_voltage; //2
  uint32_t total_sent; //4
  uint32_t start_timestamp; //4
  uint32_t end_timestamp; //4

  uint32_t target_time; //4

  uint32_t current_timestamp; //4
  uint16_t current_voltage; //2

  uint32_t time_changed; //4
  uint16_t voltage_changed; //2
  uint16_t send_freq; //2
  uint8_t state; //1
  uint32_t estimated; //4
  uint32_t remaining; //4
} solar_t; //35 bytes

static uint8_t buffer_solar[60];
solar_t *solar = (solar_t *)buffer_solar;

volatile int wdtClear = 1, period, sendData, ticks_per_send;
volatile int previous_voltage, times_discharged;

void setup() {

  pmSetVoltage(1800);
  //enableUSB();

  RFDevice.set_state(RF_STATE_TRX_OFF);
  RFDevice.set_chan(20); // set channel to 20
  RFDevice.set_state(RF_STATE_SLEEP);

  rtc.setAlarmSeconds(0);
  rtc.enableAlarm(RTC_MATCH_SS);
  rtc.attachAlarmInterrupt(rtcAlarm);

  rtc.enablePeriodicInterrupt(RTC_PER_1);
  rtc.attachPeriodicInterrupt(perInt);

  initSolar();
  wdt_init();
  wdt_reset_count();
  //PM->RCAUSE;
}


void loop() {
  if (wdtClear == 1)
  {
    wdt_reset_count();
    wdtClear = 0;
  }

  if (sendData != 0)
  {
    delay(100);
    RFDevice.send( (uint8_t*) solar, sizeof(solar_t));
    RFDevice.set_state(RF_STATE_SLEEP);

    sendData = 0;
  }

  sleep();
}

void rtcAlarm(void)
{
  //calculateSolar();
}

void perInt(void)
{
  solar->current_timestamp ++;

  if (solar->current_timestamp % 6 == 0)
  {
    wdtClear = 1;
  }

  if (solar->current_timestamp % ticks_per_send == 0 ||
      solar->current_timestamp % 10 == 0)
  {
    sendData = 1;
    if (solar->state == DISCHARGING)
    {
      //solar->total_sent ++;
    }
    solar->current_voltage = readVoltage();
  }

  if (solar->current_timestamp % 60 == 0)
  {
    calculateSolar();
  }
  
#if 0
  if (period == 0)
  {
    period = 1;
    digitalWrite(2, HIGH);
  }
  else
  {
    period = 0;
    digitalWrite(2, LOW);
  }
#endif
}

int readVoltage()
{
  digitalWrite(0, HIGH);
  int value = analogRead(0);
  digitalWrite(0, LOW);
  return value * 1000 * 77 / 40960;
}

void initSolar()
{

  EEPROM.read(0, buffer_solar, 60);
  if (solar->state == 0xFF)
  {
    memset(buffer_solar, 0, 60);

    solar->current_timestamp = 0;
    solar->current_voltage = readVoltage();
    solar->send_freq = DEFAULT_SEND_FREQ;
    solar->target_time = DEFAULT_TARGET_TIME;

    EEPROM.write(0, buffer_solar, 60);
  }
  
  previous_voltage = readVoltage();
  solar->current_voltage = previous_voltage;
  solar->state = DISCHARGING;
  ticks_per_send = SECONDS_PER_HOUR / solar->send_freq;
  times_discharged = 0;
  sendData = 1;
}

void calculateSolar()
{
  static uint8_t prev_hour = 0;

  int voltage = readVoltage();
  int delta_voltage = voltage - previous_voltage;
  int data_changed = 0;

  solar->current_voltage = voltage;
  previous_voltage = voltage;
  
  if (delta_voltage > CHARGING_DELTA_START)
  {
    solar->state = CHARGING;
    solar->send_freq = DEFAULT_SEND_FREQ;

    data_changed = 1;
    times_discharged = 0;
  }
  else
  {
    if (delta_voltage <= 0)
    {
      if (times_discharged < 2)
      {
        times_discharged ++;
      }
      else
      {
        
        times_discharged = 3;
      }
      
      if (delta_voltage < DISCHARGE_DELTA_START || times_discharged > 1)
      {
        if (solar->state == CHARGING)
        {
          if (solar->end_timestamp != 0 &&
              solar->total_sent != 0 &&
              solar->start_voltage != 0 &&
              solar->end_voltage != 0)
          {
            solar->target_time = solar->end_timestamp - solar->start_timestamp;
            solar->send_freq = (solar->target_time * solar->total_sent/ SECONDS_PER_HOUR);
            if (solar->send_freq < MIN_SEND_FREQ)
            {
              solar->send_freq = MIN_SEND_FREQ;
            }

            if (solar->send_freq > MAX_SEND_FREQ)
            {
              solar->send_freq = MAX_SEND_FREQ;
            }
            data_changed = 1;
          }
          solar->start_timestamp = solar->current_timestamp;
          solar->time_changed = solar->current_timestamp;
          solar->total_sent = 0;
          solar->start_voltage = 0;
          solar->voltage_changed = 0;
        }
        solar->state = DISCHARGING;
      }
    }
    else
    {
      times_discharged = 0;
    }
  }

  if (solar->state == DISCHARGING)
  {
    solar->end_timestamp = solar->current_timestamp;
    solar->end_voltage = voltage;
    int delta_time = solar->current_timestamp - solar->time_changed;
    int absolute_delta_time = solar->current_timestamp - solar->start_timestamp;
     
    if (absolute_delta_time >= MIN_WAIT_TIME &&
        solar->voltage_changed == 0)
    {
      data_changed = 1;
      solar->start_voltage = voltage;
      solar->end_voltage = voltage;
      solar->voltage_changed = voltage;
      solar->total_sent ++;
    }
    
    if (delta_time >= MIN_COMPUTE_TIME && 
        solar->voltage_changed > voltage)
    {
      if ( voltage > MIN_VOLTAGE) {
        solar->estimated = (voltage - MIN_VOLTAGE) * delta_time / ( solar->voltage_changed - voltage);
        //if we do not reach our quota
        solar->remaining = 0;
        if (solar->estimated + absolute_delta_time < solar->target_time)
        {
          solar->remaining = solar->target_time - absolute_delta_time;
        }
        else
        {
          solar->remaining = 2* solar->estimated + absolute_delta_time - solar-> target_time;
        }
        
        if (solar->remaining != 0)
        {
          int send_freq = SECONDS_PER_HOUR / (SECONDS_PER_HOUR / (solar->send_freq * solar->estimated / solar->remaining));
          
          if (send_freq > MAX_SEND_FREQ)
          {
              send_freq = MAX_SEND_FREQ;
          }
          if (send_freq < MIN_SEND_FREQ)
          {
            send_freq = MIN_SEND_FREQ;
          }
          
          if (send_freq != solar->send_freq)
          {
            solar->time_changed = solar->current_timestamp;
            //solar->voltage_changed = voltage;
            data_changed = 1;
            solar->send_freq = send_freq;
          }
        }
      }
      else
      {
        if (solar->send_freq != MIN_SEND_FREQ)
        {
          data_changed = 1;
          //solar->voltage_changed = voltage;
          solar->send_freq = MIN_SEND_FREQ;
        }
      }
    }
  }

  ticks_per_send = SECONDS_PER_HOUR / solar->send_freq;
    
  if (solar->current_timestamp % SECONDS_PER_HOUR == 0 ||
      data_changed == 1)
  {
    EEPROM.write(0, buffer_solar, 60);
  }
}

void enableUSB(void)
{
  pmSetVoltage(3200);
  USBDevice.init();
  USBDevice.attach();
  SerialUSB.begin(9600);
}

