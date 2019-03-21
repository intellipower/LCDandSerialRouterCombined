/**************************************************
 *
 * This code is renamed from low_level_init.c from IAR Workbench install driectory:
 *   (C:\Program Files (x86)\IAR Systems\Embedded Workbench 7.3\430\src\lib\430 and
 *   placed in the project directory as IAR_low_level_init.c 
 *   It is included iff the __IAR compile option is defined in low_level_init.c
 *  
 * This is a template for early application low-level initialization.
 *
 * Copyright 1996-2013 IAR Systems AB.
 *
 * See the file 430/doc/licenses/IARSourceLicense.txt for detailed
 * license information.
 *
 * $Revision: 9569 $
 *
 **************************************************/

/*
 * The function __low_level_init it called by the start-up code before
 * "main" is called, and before data segment initialization is
 * performed.
 *
 * This is a template file, modify to perform any initialization that
 * should take place early.
 *
 * The return value of this function controls if data segment
 * initialization should take place. If 0 is returned, it is bypassed.
 *
 * For the MSP430 microcontroller family, please consider disabling
 * the watchdog timer here, as it could time-out during the data
 * segment initialization.
 */

/*
 * To disable the watchdog timer, include a suitable device header
 * file (or "msp430.h") and add the following line to the function
 * below:
 *
 *     WDTCTL = WDTPW+WDTHOLD;
 *
 */


#include <intrinsics.h>
#include "IAR_YES_NO.h"
#ifdef __IAR
	#include "io430.h"
	//#include "IAR_low_level_init.c"		// We need __low_level_init() function
#else						//   -- this runs before main() to turn of WDT
	#include <msp430x54x.h>
#endif

int __low_level_init(void)
{
  /* Insert your low-level initializations here */

	WDTCTL = WDTPW+WDTHOLD;

  /*
   * Return value:
   *
   *  1 - Perform data segment initialization.
   *  0 - Skip data segment initialization.
   */

  return 1;
}
