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

#ifdef __cplusplus

#ifndef _WRTC_H
#define _WRTC_H

#include "Arduino.h"
#include <stdint.h>


typedef void(*voidFuncPtr)(void);

  enum Per_Int: uint8_t
  {
    RTC_PER_1             = EVSYS_ID_GEN_RTC_PER_7,
    RTC_PER_2             = EVSYS_ID_GEN_RTC_PER_6,
    RTC_PER_4             = EVSYS_ID_GEN_RTC_PER_5,
    RTC_PER_8             = EVSYS_ID_GEN_RTC_PER_4,
    RTC_PER_16            = EVSYS_ID_GEN_RTC_PER_3,
    RTC_PER_32            = EVSYS_ID_GEN_RTC_PER_2,
    RTC_PER_64            = EVSYS_ID_GEN_RTC_PER_1,
    RTC_PER_128           = EVSYS_ID_GEN_RTC_PER_0,
  };

  enum Alarm_Match: uint8_t // Should we have this enum or just use the identifiers from /component/rtc.h ?
  {
    RTC_MATCH_OFF          = RTC_MODE2_MASK_SEL_OFF_Val,          // Never
    RTC_MATCH_SS           = RTC_MODE2_MASK_SEL_SS_Val,           // Every Minute
    RTC_MATCH_MMSS         = RTC_MODE2_MASK_SEL_MMSS_Val,         // Every Hour
    RTC_MATCH_HHMMSS       = RTC_MODE2_MASK_SEL_HHMMSS_Val,       // Every Day
    RTC_MATCH_DHHMMSS      = RTC_MODE2_MASK_SEL_DDHHMMSS_Val,     // Every Month
    RTC_MATCH_MMDDHHMMSS   = RTC_MODE2_MASK_SEL_MMDDHHMMSS_Val,   // Every Year
    RTC_MATCH_YYMMDDHHMMSS = RTC_MODE2_MASK_SEL_YYMMDDHHMMSS_Val  // Once, on a specific date and a specific time
  };


class WRTC {
public:


  WRTC() {};
  void begin();

  void enableAlarm(Alarm_Match match);
  void disableAlarm();

  void attachAlarmInterrupt(voidFuncPtr callback);
  void detachAlarmInterrupt();

  /* Get Functions */

  uint8_t getSeconds();
  uint8_t getMinutes();
  uint8_t getHours();
  uint8_t getAM_PM();

  uint8_t getDay();
  uint8_t getMonth();
  uint8_t getYear();

  uint8_t getAlarmSeconds();
  uint8_t getAlarmMinutes();
  uint8_t getAlarmHours();
  uint8_t getAlarmAM_PM();

  uint8_t getAlarmDay();
  uint8_t getAlarmMonth();
  uint8_t getAlarmYear();

  /* Set Functions */

  void setSeconds(uint8_t seconds);
  void setMinutes(uint8_t minutes);
  void setHours(uint8_t hours);
  void setTime(uint8_t hours, uint8_t minutes, uint8_t seconds);

  void setDay(uint8_t day);
  void setMonth(uint8_t month);
  void setYear(uint8_t year);
  void setDate(uint8_t day, uint8_t month, uint8_t year);

  void setAlarmSeconds(uint8_t seconds);
  void setAlarmMinutes(uint8_t minutes);
  void setAlarmHours(uint8_t hours);
  void setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds);

  void setAlarmDay(uint8_t day);
  void setAlarmMonth(uint8_t month);
  void setAlarmYear(uint8_t year);
  void setAlarmDate(uint8_t day, uint8_t month, uint8_t year);

  /* Epoch Functions */

  uint32_t getEpoch();
  uint32_t getY2kEpoch();
  void setEpoch(uint32_t ts);
  void setY2kEpoch(uint32_t ts);

  void enablePeriodicInterrupt(uint8_t per);
  void disablePeriodicInterrupt();
  void attachPeriodicInterrupt(voidFuncPtr);
  void detachPeriodicInterrupt();

private:
  bool RTCisSyncing(void);
  void RTCdisable();
  void RTCenable();
  void RTCreset();
  void RTCresetRemove();
};

extern WRTC rtc;

#endif /* _WRTC_H */

#endif /* CPLUS_PLUS */
