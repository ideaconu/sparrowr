
#include "Arduino.h"
#include "events.h"

#define EVENTS_INVALID_CHANNEL  0xFF
#  define _EVENTS_START_OFFSET_BUSY_BITS           8
#  define _EVENTS_START_OFFSET_USER_READY_BIT      0
#  define _EVENTS_START_OFFSET_DETECTION_BIT       8
#  define _EVENTS_START_OFFSET_OVERRUN_BIT         0


#  define _EVENTS_INTFLAGS_DETECT  0x0f00ff00
#  define _EVENTS_INTFLAGS_OVERRUN 0x000f00ff

#define _EVENTS_INTFLAGS_MASK (_EVENTS_INTFLAGS_DETECT | _EVENTS_INTFLAGS_OVERRUN)

enum events_edge_detect {
	/** No event output */
	EVENTS_EDGE_DETECT_NONE,
	/** Event on rising edge */
	EVENTS_EDGE_DETECT_RISING,
	/** Event on falling edge */
	EVENTS_EDGE_DETECT_FALLING,
	/** Event on both edges */
	EVENTS_EDGE_DETECT_BOTH,
};

/**
 * \brief Path selection enum.
 *
 * Event channel path selection.
 *
 */
enum events_path_selection {
	/** Select the synchronous path for this event channel */
	EVENTS_PATH_SYNCHRONOUS,
	/** Select the resynchronizer path for this event channel */
	EVENTS_PATH_RESYNCHRONIZED,
	/** Select the asynchronous path for this event channel */
	EVENTS_PATH_ASYNCHRONOUS,
};


static uint8_t allocated_channels = 0;

static uint8_t _event_find_first_free_channel_and_allocate(void);

static void _events_release_channel(uint8_t channel);

uint32_t _events_find_bit_position(uint8_t channel, uint8_t start_offset);

static void _events_reset(void);


voidFuncPtr EVENTS_callBack = NULL;

uint8_t events_init(uint8_t generator, uint8_t user)
{
    PM->APBCMASK.reg |=  PM_APBCMASK_EVSYS;

    _events_reset();

    uint8_t new_channel = _event_find_first_free_channel_and_allocate();
    uint8_t edge_detect = EVENTS_EDGE_DETECT_RISING;
    uint8_t path = EVENTS_PATH_SYNCHRONOUS;

    uint32_t source_generator = GCLK_CLKCTRL_GEN_GCLK1;

    if (new_channel == EVENTS_INVALID_CHANNEL)
    {
        return -1;
    }
    uint8_t channel = EVSYS_GCLK_ID_0 + new_channel;
/* Cache the new config to reduce sync requirements */
	uint32_t new_clkctrl_config = (channel << GCLK_CLKCTRL_ID_Pos);

	/* Select the desired generic clock generator */
	new_clkctrl_config |= source_generator << GCLK_CLKCTRL_GEN_Pos;

	/* Disable generic clock channel */

    *((uint8_t*)&GCLK->CLKCTRL.reg) = channel;

	/* Switch to known-working source so that the channel can be disabled */
	uint32_t prev_gen_id = GCLK->CLKCTRL.bit.GEN;
	GCLK->CLKCTRL.bit.GEN = 0;

	/* Disable the generic clock */
	GCLK->CLKCTRL.reg &= ~GCLK_CLKCTRL_CLKEN;
	while (GCLK->CLKCTRL.reg & GCLK_CLKCTRL_CLKEN) {
		/* Wait for clock to become disabled */
	}

	/* Restore previous configured clock generator */
	GCLK->CLKCTRL.bit.GEN = prev_gen_id;

	/* Write the new configuration */
	GCLK->CLKCTRL.reg = new_clkctrl_config;

    *((uint8_t*)&GCLK->CLKCTRL.reg) = channel;

	/* Enable the generic clock */
	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN;



//

    uint32_t channel_reg = EVSYS_CHANNEL_CHANNEL(new_channel)       |
			     EVSYS_CHANNEL_EVGEN(generator)   |
			     EVSYS_CHANNEL_PATH(path)         |
			     EVSYS_CHANNEL_EDGSEL(edge_detect);

                 /* -- attach user */
    EVSYS->USER.reg = EVSYS_USER_CHANNEL(new_channel + 1) |
        EVSYS_USER_USER(user);

	/* Then configure the channel */
	EVSYS->CHANNEL.reg = channel_reg;

                 /* --- enable interrupt*/

                 //add hock
    NVIC_EnableIRQ(EVSYS_IRQn); // enable RTC interrupt
    NVIC_SetPriority(EVSYS_IRQn, 0x00);

                 /* --- enable interrupt */
    //INTERRUPT DETECT
    EVSYS->INTENSET.reg = _events_find_bit_position(new_channel,
            _EVENTS_START_OFFSET_DETECTION_BIT);

    while(EVSYS->CHSTATUS.reg & (_events_find_bit_position(new_channel,
			_EVENTS_START_OFFSET_BUSY_BITS)));

    return 0;
}

void events_attach_interrupt(uint8_t resource, voidFuncPtr callback)
{
    EVENTS_callBack = callback;
}


void events_detach_interrupt(uint8_t resource)
{
    EVENTS_callBack = NULL;
}

void events_reset(void)
{
    _events_reset();
}

static uint8_t _event_find_first_free_channel_and_allocate(void)
{

    uint8_t count;
	uint32_t tmp;
	bool allocated = false;

    //TODO better way for interrupts
    //__disable_irq();

	tmp = allocated_channels;

	for(count = 0; count < EVSYS_CHANNELS; ++count) {

		if(!(tmp & 0x00000001)) {
			/* If free channel found, set as allocated and return number */

			allocated_channels |= 1 << count;
			allocated = true;
			break;

		}

		tmp = tmp >> 1;
	}

    //TODO
    //__enable_irq();

	if(!allocated) {
		return EVENTS_INVALID_CHANNEL;
	} else {
		return count;
	}
}

static void _events_release_channel(uint8_t channel)
{
    //TODO
   // _disable_irq();

    allocated_channels &= ~(1 << channel);
   //TODO - interrupts
    //__enable_irq();
}

uint32_t _events_find_bit_position(uint8_t channel, uint8_t start_offset)
{
	uint32_t pos;

	if (channel < _EVENTS_START_OFFSET_BUSY_BITS) {
		pos = 0x01UL << (start_offset + channel);
	} else {
		pos = 0x01UL << (start_offset + channel + _EVENTS_START_OFFSET_BUSY_BITS);
	}

	return pos;
}

void EVSYS_Handler(void)
{
    EVSYS->INTFLAG.reg = _EVENTS_INTFLAGS_MASK;

    if (EVENTS_callBack != NULL)
    {
        EVENTS_callBack();
    }
}

void _events_reset(void)
{
    EVSYS->CTRL.reg = EVSYS_CTRL_SWRST;

    while (EVSYS->CTRL.reg & EVSYS_CTRL_SWRST) {
    }
}
