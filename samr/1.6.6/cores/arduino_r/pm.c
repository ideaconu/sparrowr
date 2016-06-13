#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

static uint16_t g_current_mv;

void pmInit()
{

    pinMode(VOLTAGE_VSEL2, OUTPUT);
    pinMode(VOLTAGE_VSEL3, OUTPUT);
    pinMode(VOLTAGE_VSEL4, OUTPUT);
    pinMode(VOLTAGE_CTRL, OUTPUT);

    pmSetVoltage(1800);
    pmPowerOffSensors();
}

void pmSetVoltage(uint16_t mv)
{
    if (mv <1800)
    {
        g_current_mv = 1800;
    }
    else if (mv > 3200)
    {
        g_current_mv = 3200;
    }
    else
    {
        g_current_mv = mv/100;
        g_current_mv -= g_current_mv%2;
        g_current_mv *= 100;
    }

    if ( (g_current_mv /200) % 2 == 1)
    {
        digitalWrite(VOLTAGE_VSEL2, LOW);
    }
    else
    {
        digitalWrite(VOLTAGE_VSEL2, HIGH);
    }


    if ( ((g_current_mv +200)/400) % 2 == 1)
    {
        digitalWrite(VOLTAGE_VSEL3, LOW);
    }
    else
    {
        digitalWrite(VOLTAGE_VSEL3, HIGH);
    }

    if (g_current_mv < 2600)
    {
        digitalWrite(VOLTAGE_VSEL4, LOW);
    }
    else
    {
        digitalWrite(VOLTAGE_VSEL4, HIGH);
    }

}


void pmPowerOnSensors()
{
    digitalWrite(VOLTAGE_CTRL,HIGH);
}


void pmPowerOffSensors()
{
    digitalWrite(VOLTAGE_CTRL,LOW);
}

uint16_t pmGetVoltage()
{
    return g_current_mv;
}

uint8_t pmWDTReset()
{
    return (PM->RCAUSE.reg & PM_RCAUSE_WDT) != 0;
}

#ifdef __cplusplus
}
#endif


