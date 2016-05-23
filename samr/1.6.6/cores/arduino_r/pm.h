#ifndef _PM_
#define _PM_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Initializes the external voltage regulator.
 */
void pmInit();

/**
 * \brief Set the current voltage for VDD.
 *
 * \param mv The voltage, between 1800 and 3300 mV.
 */
void pmSetVoltage(uint16_t mv);

/**
 * \brief Get the current running voltage.
 */
uint16_t pmGetVoltage();

/**
 * \brief Power on the line for the sensors. Maximum load of 100 mA.
 */
void pmPowerOnSensors();

/**
 * \brief Power off the external line for the sensors.
 */
void pmPowerOffSensors();

#ifdef __cplusplus
}
#endif
#endif
