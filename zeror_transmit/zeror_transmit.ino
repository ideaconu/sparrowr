#define TOTAL_DAYS  (3)
#define TOTAL_HOURS (TOTAL_DAYS*24)

#define WDT_USER

int sent = 0;

volatile int rtcPeriodic = 0, period;

typedef struct {
  uint16_t supply_voltage;
  uint16_t solar_voltage;
  uint16_t send_freq;
  calendar_struct calendar;
} solar_t;

solar_t current_solar;

uint32_t perTime = 0;
uint8_t current_index;

void setup() {

  pmSetVoltage(1800);
  enableUSB();

  RFDevice.set_state(RF_STATE_TRX_OFF);
  RFDevice.set_chan(20); // set channel to 20
 
  rtc.setAlarmSeconds(0);
  rtc.enableAlarm(RTC_MATCH_SS);
  //rtc.attachAlarmInterrupt(rtcAlarm);
  
  rtc.enablePeriodicInterrupt(RTC_PER_1);
  rtc.attachPeriodicInterrupt(perInt);
  //initSolar(); 
}


void loop() {
  if (rtcPeriodic != 0)
  {
    RFDevice.send( (uint8_t*) &current_solar, sizeof(solar_t));
    RFDevice.set_state(RF_STATE_SLEEP);
  
    rtcPeriodic = 0;
  }
  
  sleep();
}

void rtcAlarm(void)
{
  calculateSolar();
}
 
void perInt(void)
{ 
  perTime ++;
  wdt_reset_count();
  
  if (perTime % 60 == 0)
  {
  }
  
  rtcPeriodic = 1;
  if (period == 0)
  {
    period = 1;
    digitalWrite(2,HIGH);
  }
  else
  {
    period = 0;
    digitalWrite(2,LOW);
  }
  
  digitalWrite(0,HIGH); 
  int solar = analogRead(0);
  digitalWrite(0,LOW); 
   
  SerialUSB.println(solar*1000*77/40960);
  //SerialUSB.println(1000*solar/4096);
}

void initSolar()
{
  solar_t solar_data[24];
  
  current_solar.calendar.FIELD = 0xFFFFFFFF;
  for (uint8_t days = 0; days < TOTAL_DAYS; days ++)
  {
    EEPROM.read(days,(uint8_t*)solar_data,24 * sizeof(solar_t));
    for( uint8_t hours = 0; hours < 24; hours ++)
    {
      if (solar_data[hours].calendar.FIELD < current_solar.calendar.FIELD)
      {
         current_solar = solar_data[hours];
         current_index = hours + days*24;
      }
    }
  }

  if (current_solar.calendar.FIELD != 0xFFFFFFFF)
  {
    rtc.setTime(current_solar.calendar.hour, current_solar.calendar.minute, current_solar.calendar.second);
    rtc.setDate(current_solar.calendar.year, current_solar.calendar.month, current_solar.calendar.day);
  }
  else
  {
    current_solar.calendar.year = rtc.getYear();
    current_solar.calendar.month = rtc.getMonth();
    current_solar.calendar.day = rtc.getDay();
    current_solar.calendar.hour = rtc.getHours();
    current_solar.calendar.minute = rtc.getMinutes();
    current_solar.calendar.second = rtc.getSeconds();
    current_solar.supply_voltage = 1800;
    current_solar.solar_voltage = 1800;
    current_solar.send_freq = 3600;
  }
}

void calculateSolar()
{
  static uint8_t prev_hour = 0; 
  uint8_t current_hour = rtc.getHours();
  uint8_t current_minute = rtc.getMinutes();
  current_solar.calendar.second = rtc.getSeconds();
 
  current_solar.calendar.minute = current_minute;
  current_solar.supply_voltage = 1800;
  current_solar.solar_voltage = 1800;
  current_solar.send_freq = 3600; 
  
  if (current_hour != prev_hour)
  {
    current_solar.calendar.year = rtc.getYear();
    current_solar.calendar.month = rtc.getMonth();
    current_solar.calendar.day = rtc.getDay();
    current_solar.calendar.hour = current_hour;
    prev_hour = current_hour;
    
    //current_index = (current_index + 1 ) % TOTAL_HOURS;
    solar_t solar_data[24];
    EEPROM.read(current_index/24,(uint8_t*)solar_data,24 * sizeof(solar_t));
    solar_data[current_index%24] = current_solar;
    EEPROM.write(current_index/24,(uint8_t*)solar_data,24 * sizeof(solar_t)); 
    //EEPROM.read(current_index/24,(uint8_t*)solar_data,24 * sizeof(solar_t));
   //current_solar = solar_data[current_index%24];
    
  }
}

void enableUSB(void)
{
  pmSetVoltage(3200);
  USBDevice.init();
  USBDevice.attach();
  SerialUSB.begin(9600);
}

