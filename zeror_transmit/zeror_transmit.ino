#include <math.h>


#define SECONDS_PER_MINUTE  (60)
#define SECONDS_PER_HOUR    (60*SECONDS_PER_MINUTE)
#define WDT_SAVE_PERIOD     (10*SECONDS_PER_MINUTE)

#define DEFAULT_TARGET_TIME (12*60*60)
#define MIN_ESTIMATED_TIME  (10*60)

//#define TIME_GRACE_PERIOD   (30*60)


#define TIME_EXTRA_REMAINING    (10*60)
#define TIME_PERCENTAGE_PENALTY (80)

#define SPEED_DECREASE_PENALTY  (75)

#define MIN_SEND_FREQ       (1)
#define MAX_SEND_FREQ       (3600)
#define DEFAULT_SEND_FREQ   (600)

#define MIN_WAIT_TIME       (119)
#define MIN_COMPUTE_TIME    (MIN_WAIT_TIME+119)
#define MIN_VOLTAGE_COMPUTE (2200)
#define MIN_VOLTAGE         (2100)
#define MIN_VOLTAGE_DELTA   (40)

#define CHARGING            (0)
#define DISCHARGING         (1)
#define CHARGING_DELTA_START  (40)
#define DISCHARGE_DELTA_START (-45)

typedef struct __attribute__((packed)){
  uint32_t nothing; //4
  uint32_t total_sent; //4
  uint32_t start_timestamp; //4
  uint32_t end_timestamp; //4

  uint32_t target_time; //4

  uint32_t current_timestamp; //4
  uint16_t current_voltage; //2

  uint32_t time_changed; //4
  uint16_t voltage_changed; //2
  int16_t send_freq; //2
  uint16_t actual_freq; //2
  uint8_t state; //1
  int32_t estimated; //4
  int32_t remaining; //4
} solar_t; //35 bytes

static uint8_t buffer_solar[EEPROM_PAGE_SIZE+4];
solar_t *solar = (solar_t *)buffer_solar;

volatile int wdtClear = 1, period, sendData, ticks_per_send;
volatile int previous_voltage, previous_voltage_minutes, times_discharged;
volatile int wdt_last_saved_time;
void setup() {

  pmSetVoltage(1800);
  //enableUSB();

  RFDevice.set_state(RF_STATE_TRX_OFF);
  RFDevice.set_chan(20); // set channel to 20
  RFDevice.set_state(RF_STATE_SLEEP);

  initSolar();

  rtc.setAlarmSeconds(0);
  rtc.enableAlarm(RTC_MATCH_SS);
  rtc.attachAlarmInterrupt(rtcAlarm);

  rtc.enablePeriodicInterrupt(RTC_PER_1);
  rtc.attachPeriodicInterrupt(perInt);

  wdt_init();
  wdt_reset_count();
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

  if (solar->current_timestamp % ticks_per_send == 0)
  {
    sendData = 1;
    if (solar->state == DISCHARGING)
    {
      solar->total_sent ++;
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

  EEPROM.read(0, buffer_solar, EEPROM_PAGE_SIZE);
  int changed = 0;
  if (solar->state == 0xFF)
  {
    memset(buffer_solar, 0, EEPROM_PAGE_SIZE);

    solar->current_timestamp = 0;
    solar->send_freq = DEFAULT_SEND_FREQ;
    solar->target_time = DEFAULT_TARGET_TIME;
    changed = 1;
  }

  //This in case a wdt reset happens, recover some of time lost.
  if (pmWDTReset() == 1)
  {
    solar->current_timestamp += WDT_SAVE_PERIOD / 2;  
    changed = 1;
  }
  previous_voltage = readVoltage(); 
  previous_voltage_minutes = previous_voltage;
  ticks_per_send = SECONDS_PER_HOUR / solar->send_freq;
  times_discharged = 0;
  sendData = 1;
  solar->current_voltage = previous_voltage;
  wdt_last_saved_time = solar->current_timestamp;
  
  if (changed == 1)
  {
    EEPROM.write(0, buffer_solar, EEPROM_PAGE_SIZE);
  }
}

void calculateSolar()
{
  int voltage = readVoltage();
  int delta_voltage = voltage - previous_voltage;
  int delta_voltage_minutes = voltage - previous_voltage_minutes;
  int data_changed = 0;

  solar->current_voltage = voltage;
  previous_voltage = voltage;
  
  if (solar->current_timestamp % (4 * SECONDS_PER_MINUTE) == 0)
  {
    previous_voltage_minutes = voltage;
  }
  
  if (delta_voltage > CHARGING_DELTA_START || delta_voltage_minutes > CHARGING_DELTA_START)
  {
    if (solar->state == DISCHARGING)
    {
      solar->state = CHARGING;
      solar->send_freq = DEFAULT_SEND_FREQ;
      data_changed = 1;
    }
    times_discharged = 0;
  }
  else
  {
    if (delta_voltage <= 0)
    { 
      if (times_discharged < 1000)
      {
        times_discharged ++;
      }
      if (delta_voltage_minutes < DISCHARGE_DELTA_START || delta_voltage < DISCHARGE_DELTA_START || times_discharged > 12)
      {
        if (solar->state == CHARGING)
        {
          if (solar->end_timestamp != 0 &&
              solar->total_sent != 0 &&
              solar->end_timestamp > solar->start_timestamp)
          {
            solar->target_time = solar->end_timestamp - solar->start_timestamp;
            solar->send_freq = (SECONDS_PER_HOUR * solar->total_sent / solar->target_time);
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
    int delta_time = solar->current_timestamp - solar->time_changed;
    int absolute_delta_time = solar->current_timestamp - solar->start_timestamp;
     
    if (delta_time >= MIN_WAIT_TIME &&
        solar->voltage_changed == 0)
    {
      data_changed = 1;
      solar->voltage_changed = voltage; 
    }
    
     
    if (voltage > MIN_VOLTAGE_COMPUTE &&
        delta_time >= MIN_COMPUTE_TIME && 
        solar->voltage_changed > voltage + MIN_VOLTAGE_DELTA)
    {
      //solar->estimated = TIME_PERCENTAGE_PENALTY *(voltage - MIN_VOLTAGE) * delta_time / ( solar->voltage_changed - voltage) - TIME_GRACE_PERIOD;
      
      solar->estimated = (TIME_PERCENTAGE_PENALTY *(voltage - MIN_VOLTAGE) * delta_time) / (( solar->voltage_changed - voltage) * 100);
      if (solar->estimated < MIN_ESTIMATED_TIME)
      {
        solar->estimated = MIN_ESTIMATED_TIME;
      }
      //if we do not reach our quota
      solar->remaining = 0;
      
      if (solar->target_time > absolute_delta_time)
      {
        solar->remaining = solar->target_time - absolute_delta_time;
      }

      if (solar->remaining > 0)
      {
        solar->actual_freq = (solar->send_freq * solar->estimated) / (solar->remaining + TIME_EXTRA_REMAINING);
        //if we change down the speed, then we do not scale linear, so we incure a penalty.
        if (solar->actual_freq < solar->send_freq)
        {
          solar->actual_freq = solar->actual_freq * SPEED_DECREASE_PENALTY / 100;
        }
        int send_freq = SECONDS_PER_HOUR / ceil(SECONDS_PER_HOUR *1.0f /  solar->actual_freq);
        
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
          solar->voltage_changed = 0;
          data_changed = 1;
          solar->send_freq = send_freq;
        }
      }
    }
    
    if (voltage < MIN_VOLTAGE_COMPUTE)
    {
        int send_freq = SECONDS_PER_HOUR / ceil(SECONDS_PER_HOUR *2.0f /  solar->actual_freq);
        
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
          data_changed = 1;
          solar->send_freq = send_freq;
        }
    }
  }

  ticks_per_send = SECONDS_PER_HOUR / solar->send_freq;
    
  if (solar->current_timestamp - wdt_last_saved_time >= WDT_SAVE_PERIOD ||
      data_changed == 1)
  {
    wdt_last_saved_time = solar->current_timestamp;
    EEPROM.write(0, buffer_solar, EEPROM_PAGE_SIZE);
  }
}

void enableUSB(void)
{
  pmSetVoltage(3200);
  USBDevice.init();
  USBDevice.attach();
  SerialUSB.begin(9600);
}

