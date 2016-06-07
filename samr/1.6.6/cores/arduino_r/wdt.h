/**
 * \file
 *
 * \brief SAM Watchdog Driver
 *
 * Copyright (c) 2012-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#ifndef WDT_H_INCLUDED
#define WDT_H_INCLUDED

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Watchdog Timer period configuration enum.
 *
 * Enum for the possible period settings of the Watchdog timer module, for
 * values requiring a period as a number of Watchdog timer clock ticks.
 */
enum wdt_period {
	/** No Watchdog period. This value can only be used when setting the
	 *  Window and Early Warning periods; its use as the Watchdog Reset
	 *  Period is invalid. */
	WDT_PERIOD_NONE     = 0,
	/** Watchdog period of 8 clocks of the Watchdog Timer Generic Clock */
	WDT_PERIOD_8CLK     = 1,
	/** Watchdog period of 16 clocks of the Watchdog Timer Generic Clock */
	WDT_PERIOD_16CLK    = 2,
	/** Watchdog period of 32 clocks of the Watchdog Timer Generic Clock */
	WDT_PERIOD_32CLK    = 3,
	/** Watchdog period of 64 clocks of the Watchdog Timer Generic Clock */
	WDT_PERIOD_64CLK    = 4,
	/** Watchdog period of 128 clocks of the Watchdog Timer Generic Clock */
	WDT_PERIOD_128CLK   = 5,
	/** Watchdog period of 256 clocks of the Watchdog Timer Generic Clock */
	WDT_PERIOD_256CLK   = 6,
	/** Watchdog period of 512 clocks of the Watchdog Timer Generic Clock */
	WDT_PERIOD_512CLK   = 7,
	/** Watchdog period of 1024 clocks of the Watchdog Timer Generic Clock */
	WDT_PERIOD_1024CLK  = 8,
	/** Watchdog period of 2048 clocks of the Watchdog Timer Generic Clock */
	WDT_PERIOD_2048CLK  = 9,
	/** Watchdog period of 4096 clocks of the Watchdog Timer Generic Clock */
	WDT_PERIOD_4096CLK  = 10,
	/** Watchdog period of 8192 clocks of the Watchdog Timer Generic Clock */
	WDT_PERIOD_8192CLK  = 11,
	/** Watchdog period of 16384 clocks of the Watchdog Timer Generic Clock */
	WDT_PERIOD_16384CLK = 12,
};


/**
 * \brief Determines if the hardware module(s) are currently synchronizing to the bus.
 *
 * Checks to see if the underlying hardware peripheral module(s) are currently
 * synchronizing across multiple clock domains to the hardware bus. This
 * function can be used to delay further operations on a module until such time
 * that it is ready, to prevent blocking delays for synchronization in the
 * user application.
 *
 * \return Synchronization status of the underlying hardware module(s).
 *
 * \retval false If the module has completed synchronization
 * \retval true If the module synchronization is ongoing
 */
static inline bool wdt_is_syncing(void)
{
	Wdt *const WDT_module = WDT;

	if (WDT_module->STATUS.reg & WDT_STATUS_SYNCBUSY) {
		return true;
	}

	return false;
}

/**
 * \brief Initializes a Watchdog Timer configuration structure to defaults.
 *
 *  Initializes a given Watchdog Timer configuration structure to a set of
 *  known default values. This function should be called on all new
 *  instances of these configuration structures before being modified by the
 *  user application.
 *
 *  The default configuration is as follows:
 *   \li Not locked, to allow for further (re-)configuration
 *   \li Enable WDT
 *   \li Watchdog timer sourced from Generic Clock Channel 4
 *   \li A timeout period of 16384 clocks of the Watchdog module clock
 *   \li No window period, so that the Watchdog count can be reset at any time
 *   \li No early warning period to indicate the Watchdog will soon expire
 *
 *  \param[out] config  Configuration structure to initialize to default values
 */

int wdt_init();

/** \brief Determines if the Watchdog timer is currently locked in an enabled state.
 *
 *  Determines if the Watchdog timer is currently enabled and locked, so that
 *  it cannot be disabled or otherwise reconfigured.
 *
 *  \return Current Watchdog lock state.
 */
static inline bool wdt_is_locked(void)
{
	Wdt *const WDT_module = WDT;
	return (WDT_module->CTRL.reg & WDT_CTRL_ALWAYSON);
}

/** @} */

/** \name Timeout and Early Warning Management
 * @{
 */

/** \brief Clears the Watchdog timer early warning period elapsed flag.
 *
 *  Clears the Watchdog timer early warning period elapsed flag, so that a new
 *  early warning period can be detected.
 */
static inline void wdt_clear_early_warning(void)
{
	Wdt *const WDT_module = WDT;

	WDT_module->INTFLAG.reg = WDT_INTFLAG_EW;
}

/** \brief Determines if the Watchdog timer early warning period has elapsed.
 *
 *  Determines if the Watchdog timer early warning period has elapsed.
 *
 *  \note If no early warning period was configured, the value returned by this
 *        function is invalid.
 *
 *  \return Current Watchdog Early Warning state.
 */
static inline bool wdt_is_early_warning(void)
{
	Wdt *const WDT_module = WDT;

	return (WDT_module->INTFLAG.reg & WDT_INTFLAG_EW);
}

void wdt_reset_count(void);

void wdt_clear_counter(int minutes);


/** @} */

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* WDT_H_INCLUDED */
