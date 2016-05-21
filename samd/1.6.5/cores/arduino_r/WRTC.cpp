/*
  RTC library for Arduino Zero.
  Copyright (c) 2015 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <time.h>
#include "WRTC.h"
#include "events.h"

#define EPOCH_TIME_OFF      946684800  // This is 1st January 2000, 00:00:00 in epoch time
#define EPOCH_TIME_YEAR_OFF 100        // years since 1900

voidFuncPtr RTC_callBack = NULL;

WRTC rtc = WRTC();

void WRTC::begin()
{
  uint16_t tmp_reg = 0;

  PM->APBAMASK.reg |= PM_APBAMASK_RTC; // turn on digital interface clock

  // Setup clock GCLK2 with OSC32K divided by 32
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2)|GCLK_GENDIV_DIV(4);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;
  GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_DIVSEL | GCLK_GENCTRL_RUNSTDBY);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  GCLK->CLKCTRL.reg = (uint32_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2 |  GCLK_CLKCTRL_ID(RTC_GCLK_ID));
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  RTCdisable();

  RTCreset();

  tmp_reg |= RTC_MODE2_CTRL_MODE_CLOCK; // set clock operating mode
  tmp_reg |= RTC_MODE2_CTRL_PRESCALER_DIV1024; // set prescaler to 1024 for MODE2
  tmp_reg &= ~RTC_MODE2_CTRL_MATCHCLR; // disable clear on match

  //According to the datasheet RTC_MODE2_CTRL_CLKREP = 0 for 24h
  tmp_reg &= ~RTC_MODE2_CTRL_CLKREP; // 24h time representation

  RTC->MODE2.READREQ.reg &= ~RTC_READREQ_RCONT; // disable continuously mode

  RTC->MODE2.CTRL.reg = tmp_reg;
  while (RTCisSyncing())
    ;

  NVIC_EnableIRQ(RTC_IRQn); // enable RTC interrupt
  NVIC_SetPriority(RTC_IRQn, 0x00);

  RTC->MODE2.INTENSET.reg |= RTC_MODE2_INTENSET_ALARM0; // enable alarm interrupt
  RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = RTC_MATCH_OFF; // default alarm match is off (disabled)

  while (RTCisSyncing())
    ;

  RTCenable();
  RTCresetRemove();
}

void RTC_Handler(void)
{
  if (RTC_callBack != NULL) {
    RTC_callBack();
  }

  RTC->MODE2.INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0; // must clear flag at end
}

void WRTC::enableAlarm(Alarm_Match match)
{
  RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = match;
  while (RTCisSyncing())
    ;
}

void WRTC::disableAlarm()
{
  RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = 0x00;
  while (RTCisSyncing())
    ;
}

void WRTC::attachAlarmInterrupt(voidFuncPtr callback)
{
  RTC_callBack = callback;
}

void WRTC::detachAlarmInterrupt()
{
  RTC_callBack = NULL;
}


void WRTC::enablePeriodicInterrupt(uint8_t per)
{
    RTCdisable();

    uint32_t event_mask =0;

    event_mask |= RTC_MODE2_EVCTRL_PEREO(1 << (per - RTC_PER_128));

    RTC->MODE2.EVCTRL.reg = event_mask;

    RTCenable();

    events_init(per,EVSYS_ID_USER_DMAC_CH_0);
}

void WRTC::disablePeriodicInterrupt()
{
    events_reset();
}

void WRTC::attachPeriodicInterrupt(voidFuncPtr callback)
{
    events_attach_interrupt(0,callback);
}

void WRTC::detachPeriodicInterrupt()
{
    events_detach_interrupt(0);
}

/*
 * Get Functions
 */

uint8_t WRTC::getSeconds()
{
  return RTC->MODE2.CLOCK.bit.SECOND;
}

uint8_t WRTC::getMinutes()
{
  return RTC->MODE2.CLOCK.bit.MINUTE;
}

uint8_t WRTC::getHours()
{
  return RTC->MODE2.CLOCK.bit.HOUR;
}

uint8_t WRTC::getDay()
{
  return RTC->MODE2.CLOCK.bit.DAY;
}

uint8_t WRTC::getMonth()
{
  return RTC->MODE2.CLOCK.bit.MONTH;
}

uint8_t WRTC::getYear()
{
  return RTC->MODE2.CLOCK.bit.YEAR;
}

uint8_t WRTC::getAlarmSeconds()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND;
}

uint8_t WRTC::getAlarmMinutes()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE;
}

uint8_t WRTC::getAlarmHours()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR;
}

uint8_t WRTC::getAlarmDay()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY;
}

uint8_t WRTC::getAlarmMonth()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH;
}

uint8_t WRTC::getAlarmYear()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR;
}

/*
 * Set Functions
 */

void WRTC::setSeconds(uint8_t seconds)
{
  RTC->MODE2.CLOCK.bit.SECOND = seconds;
  while (RTCisSyncing())
    ;
}

void WRTC::setMinutes(uint8_t minutes)
{
  RTC->MODE2.CLOCK.bit.MINUTE = minutes;
  while (RTCisSyncing())
    ;
}

void WRTC::setHours(uint8_t hours)
{
  RTC->MODE2.CLOCK.bit.HOUR = hours;
  while (RTCisSyncing())
    ;
}

void WRTC::setTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  setSeconds(seconds);
  setMinutes(minutes);
  setHours(hours);
}

void WRTC::setDay(uint8_t day)
{
  RTC->MODE2.CLOCK.bit.DAY = day;
  while (RTCisSyncing())
    ;
}

void WRTC::setMonth(uint8_t month)
{
  RTC->MODE2.CLOCK.bit.MONTH = month;
  while (RTCisSyncing())
    ;
}

void WRTC::setYear(uint8_t year)
{
  RTC->MODE2.CLOCK.bit.YEAR = year;
  while (RTCisSyncing())
    ;
}

void WRTC::setDate(uint8_t day, uint8_t month, uint8_t year)
{
  setDay(day);
  setMonth(month);
  setYear(year);
}

void WRTC::setAlarmSeconds(uint8_t seconds)
{
  RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND = seconds;
  while (RTCisSyncing())
    ;
}

void WRTC::setAlarmMinutes(uint8_t minutes)
{
  RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE = minutes;
  while (RTCisSyncing())
    ;
}

void WRTC::setAlarmHours(uint8_t hours)
{
  RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR = hours;
  while (RTCisSyncing())
    ;
}

void WRTC::setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  setAlarmSeconds(seconds);
  setAlarmMinutes(minutes);
  setAlarmHours(hours);
}

void WRTC::setAlarmDay(uint8_t day)
{
  RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY = day;
  while (RTCisSyncing())
    ;
}

void WRTC::setAlarmMonth(uint8_t month)
{
  RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH = month;
  while (RTCisSyncing())
    ;
}

void WRTC::setAlarmYear(uint8_t year)
{
  RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR = year;
  while (RTCisSyncing())
    ;
}

void WRTC::setAlarmDate(uint8_t day, uint8_t month, uint8_t year)
{
  setAlarmDay(day);
  setAlarmMonth(month);
  setAlarmYear(year);
}

uint32_t WRTC::getEpoch()
{
  struct tm tm;

  tm.tm_isdst = -1;
  tm.tm_yday = 0;
  tm.tm_wday = 0;
  tm.tm_year = getYear() + EPOCH_TIME_YEAR_OFF;
  tm.tm_mon = getMonth() - 1;
  tm.tm_mday = getDay();
  tm.tm_hour = getHours();
  tm.tm_min = getMinutes();
  tm.tm_sec = getSeconds();

  return mktime(&tm);
}

uint32_t WRTC::getY2kEpoch()
{
  return (getEpoch() - EPOCH_TIME_OFF);
}

void WRTC::setEpoch(uint32_t ts)
{
  if (ts < EPOCH_TIME_OFF) {
    ts = EPOCH_TIME_OFF;
  }

  time_t t = ts;
  struct tm* tmp = gmtime(&t);

  RTC->MODE2.CLOCK.bit.YEAR = tmp->tm_year - EPOCH_TIME_YEAR_OFF;
  RTC->MODE2.CLOCK.bit.MONTH = tmp->tm_mon + 1;
  RTC->MODE2.CLOCK.bit.DAY = tmp->tm_mday;
  RTC->MODE2.CLOCK.bit.HOUR = tmp->tm_hour;
  RTC->MODE2.CLOCK.bit.MINUTE = tmp->tm_min;
  RTC->MODE2.CLOCK.bit.SECOND = tmp->tm_sec;

  while (RTCisSyncing())
    ;
}

void WRTC::setY2kEpoch(uint32_t ts)
{
  setEpoch(ts + EPOCH_TIME_OFF);
}

/*
 * Private Utility Functions
 */

/* Wait for sync in write operations */
bool WRTC::RTCisSyncing()
{
  return (RTC->MODE2.STATUS.bit.SYNCBUSY);
}

void WRTC::RTCdisable()
{
  RTC->MODE2.CTRL.reg &= ~RTC_MODE2_CTRL_ENABLE; // disable RTC
  while (RTCisSyncing())
    ;
}

void WRTC::RTCenable()
{
  RTC->MODE2.CTRL.reg |= RTC_MODE2_CTRL_ENABLE; // enable RTC
  while (RTCisSyncing())
    ;
}

void WRTC::RTCreset()
{
  RTC->MODE2.CTRL.reg |= RTC_MODE2_CTRL_SWRST; // software reset
  while (RTCisSyncing())
    ;
}

void WRTC::RTCresetRemove()
{
  RTC->MODE2.CTRL.reg &= ~RTC_MODE2_CTRL_SWRST; // software reset remove
  while (RTCisSyncing())
    ;
}
