/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * System Core Clock is at 1MHz (8MHz/8) at Reset.
 * It is switched to 48MHz in the Reset Handler (startup.c)
 */
uint32_t SystemCoreClock=1000000ul ;

/*
void calibrateADC()
{
  volatile uint32_t valeur = 0;

  for(int i = 0; i < 5; ++i)
  {
    ADC->SWTRIG.bit.START = 1;
    while( ADC->INTFLAG.bit.RESRDY == 0 || ADC->STATUS.bit.SYNCBUSY == 1 )
    {
      // Waiting for a complete conversion and complete synchronization
    }

    valeur += ADC->RESULT.bit.RESULT;
  }

  valeur = valeur/5;
}*/

/*
 * Arduino Zero board initialization
 *
 * Good to know:
 *   - At reset, ResetHandler did the system clock configuration. Core is running at 48MHz.
 *   - Watchdog is disabled by default, unless someone plays with NVM User page
 *   - During reset, all PORT lines are configured as inputs with input buffers, output buffers and pull disabled.
 */
void init( void )
{
  uint32_t ul ;

  // Set Systick to 1ms interval, common to all Cortex-M variants

  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

  if ( SysTick_Config( SystemCoreClock / 1000 ) )
  {
    // Capture error
    while ( 1 ) ;
  }

  // Clock PORT for Digital I/O
  PM->APBBMASK.reg |= PM_APBBMASK_PORT | PM_APBBMASK_NVMCTRL;

  // Clock ADC/DAC for Analog
  PM->APBCMASK.reg |= PM_APBCMASK_ADC;

  // Setup all pins (digital and analog) in INPUT mode (default is nothing)
 //Adds high power for the moment
  // for ( ul = 0 ; ul < NUM_DIGITAL_PINS ; ul++ )
 // {
//	  pinMode( ul, INPUT ) ;
//  }
  // Initialize Analog Controller
  // Setting clock
  while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_ADC ) | // Generic Clock ADC
                      GCLK_CLKCTRL_GEN_GCLK0     | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV128 |    // Divide Clock by 512.
                   ADC_CTRLB_RESSEL_12BIT;         // 10 bits resolution as default

  ADC->SAMPCTRL.reg = 0x3f;                        // Set max Sampling Time Length

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)

  // Averaging (see datasheet table in AVGCTRL register description)
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_32 |    // 1 sample only (no oversampling nor averaging)
                     ADC_AVGCTRL_ADJRES(0x4ul);   // Adjusting result by 0

  analogReference( AR_DEFAULT ) ; // Analog Reference is AREF pin (3.3v)

}

#ifdef __cplusplus
}
#endif
