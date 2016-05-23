#ifndef _SLEEP_
#define _SLEEP_

#include "sam.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief the Sleep enum types.
 */
typedef enum
{
    SLEEP_IDLE_0,
    SLEEP_IDLE_1,
    SLEEP_IDLE_2,
    SLEEP_STANDBY,
    SLEEP_ACTIVE,
}SleepMode;


/**
 * \brief Set the desired sleep mode.
 *
 * \param sleep_mode the sleep mode.
 */
static __inline__ void sleepMode(SleepMode sleep_mode)
{
  NVMCTRL->CTRLB.bit.SLEEPPRM = 0;
  switch (sleep_mode) {
		case SLEEP_IDLE_0:
		case SLEEP_IDLE_1:
		case SLEEP_IDLE_2:
			SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
			PM->SLEEP.reg = sleep_mode;
			break;

		case SLEEP_STANDBY:
			SCB->SCR |=  SCB_SCR_SLEEPDEEP_Msk;
			break;

		default:
			return;
	}
}


/**
 * \brief Puts the cpu to sleep.
 */
static __inline__ void sleep(void)
{
  __DMB();
  __enable_irq();
  __DSB();
  __WFI();
}

#ifdef __cplusplus
}
#endif
#endif
