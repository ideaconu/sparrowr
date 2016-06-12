/**
 * \file
 *
 * \brief SAM Watchdog Driver
 *
 * Copyright (C) 2012-2015 Atmel Corporation. All rights reserved.
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
#include "wdt.h"
#include "WRTC.h"
/**
 * \brief Sets up the WDT hardware module based on the configuration.
 *
 * Writes a given configuration of a WDT configuration to the
 * hardware module, and initializes the internal device struct.
 *
 * \param[in]  config  Pointer to the configuration struct
 *
 * \return Status of the configuration procedure.
 *
 * \retval STATUS_OK     If the module was configured correctly
 * \retval STATUS_ERR_INVALID_ARG   If invalid argument(s) were supplied
 * \retval STATUS_ERR_IO  If the Watchdog module is locked to be always on
 */


static int last_minutes;

int wdt_init()
{
    last_minutes = -1;
    Wdt *const WDT_module = WDT;

	/* Turn on the digital interface clock */
    PM->APBAMASK.reg |= PM_APBAMASK_WDT;

	/* Check of the Watchdog has been locked to be always on, if so, abort */
	if (wdt_is_locked()) {
		return -1;
	}


	/* Disable the Watchdog module */
	WDT_module->CTRL.reg &= ~WDT_CTRL_ENABLE;

	while (wdt_is_syncing()) {
		/* Wait for all hardware modules to complete synchronization */
	}


	/* Configure GCLK channel and enable clock */
#if 0
    struct system_gclk_chan_config gclk_chan_conf;
	gclk_chan_conf.source_generator = config->clock_source;
	system_gclk_chan_set_config(WDT_GCLK_ID, &gclk_chan_conf);
	system_gclk_chan_enable(WDT_GCLK_ID);
#endif
  // Setup clock GCLK4 with OSC32K divided by 128 for 64s period.
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(4)|GCLK_GENDIV_DIV(3);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;
  GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(4) | GCLK_GENCTRL_DIVSEL | GCLK_GENCTRL_RUNSTDBY);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;


    GCLK->CLKCTRL.reg = (uint32_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 |  GCLK_CLKCTRL_ID(WDT_GCLK_ID));
    while (GCLK->STATUS.bit.SYNCBUSY);

	uint32_t new_config = 0;

    /* Set to 4 seconds, for low power when getting stuck */
	/* Update the timeout period value with the requested period */
	new_config |= (WDT_PERIOD_16384CLK - 1) << WDT_CONFIG_PER_Pos;

#if 0
	/* Check if the user has requested a reset window period */
	if (config->window_period != WDT_PERIOD_NONE) {
		WDT_module->CTRL.reg |= WDT_CTRL_WEN;

		/* Update and enable the timeout period value */
		new_config |= (config->window_period - 1) << WDT_CONFIG_WINDOW_Pos;
	} else {
		/* Ensure the window enable control flag is cleared */
		WDT_module->CTRL.reg &= ~WDT_CTRL_WEN;
	}
#endif

    /* Ensure the window enable control flag is cleared */
    WDT_module->CTRL.reg &= ~WDT_CTRL_WEN;

    while (wdt_is_syncing()) {
		/* Wait for all hardware modules to complete synchronization */
	}

	/* Write the new Watchdog configuration */
	WDT_module->CONFIG.reg = new_config;

#if 0
	/* Check if the user has requested an early warning period */
	if (config->early_warning_period != WDT_PERIOD_NONE) {
		while (wdt_is_syncing()) {
			/* Wait for all hardware modules to complete synchronization */
		}

		/* Set the Early Warning period */
		WDT_module->EWCTRL.reg
			= (config->early_warning_period - 1) << WDT_EWCTRL_EWOFFSET_Pos;
	}
#endif
	/* Either enable or lock-enable the Watchdog timer depending on the user
	 * settings */
    WDT_module->CTRL.reg |= WDT_CTRL_ENABLE;

	while (wdt_is_syncing()) {
		/* Wait for all hardware modules to complete synchronization */
	}

	return 0;
}

/**
 * \brief Resets the count of the running Watchdog Timer that was previously enabled.
 *
 * Resets the current count of the Watchdog Timer, restarting the timeout
 * period count elapsed. This function should be called after the window
 * period (if one was set in the module configuration) but before the timeout
 * period to prevent a reset of the system.
 */
void wdt_reset_count(void)
{

    while (wdt_is_syncing()) {
		/* Wait for all hardware modules to complete synchronization */
	}

	/* Disable the Watchdog module */
	WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;

}

void wdt_clear_counter(int minutes)
{

    if (minutes != last_minutes)
    {
        last_minutes = minutes;
        wdt_reset_count();
    }
}


