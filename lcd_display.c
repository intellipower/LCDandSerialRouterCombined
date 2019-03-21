// LCD Display and Serial Router

// Check configuration in system_config.h may be 5KVA dual board or Briefcase UPS

#include "IAR_YES_NO.h"
#ifdef __IAR
	#include "io430.h"
#else
	#include <msp430x54x.h>
#endif

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "types.h"
#include "low_level_init.h"
#include "timera.h"
#include "leds.h"
#include "system_config.h"						// includes file name for configuration
#include SYSTEM_CONFIG							// configuration header file
#include "HD44780_display.h"
#include "port1.h"
//#if LCD_DISPLAY==TRUE
	#include "lcd_display.h"
//#endif
#include "main.h" 								// called in lcd_display.h

/*
#include "uscia0_UART.h"
#include "uscia1_UART.h"
#include "uscia2_UART.h"
#include "uscia3_UART.h"
*/

#include "uscia_UART.h"

/*
Note: the following are notifications sent from the ups, this happens
when a button is pressed on the front panel or an important event occurs

RBUT01 Test button
RBUT02 Silence button
RBUT06 Off button
RBUT08 On button
RBUT12 Bypass button
RBUT25 Overcurrent or overload shutdown has occured
RBUT30 UPS started and Lucy and Panel tasks are running
RBUT31 Test Lightshow finished, update the display

*/

/* Port assignments

UART0 - Master/UPS
UART1 - SNMP
UART2 - RS485/Testport/Network
UART3 - Slave/Customer/Upsilon

*/
/*
#define FOREVER while(1)
*/
volatile struct upsDataStrucT upsOne, upsTwo, upsThree, upsBoss, *pUpsOne, *pUpsTwo, *pUpsThree, *pUpsBoss, *pUps;

volatile struct upsDataStrucT /* *pUpsMain, *pUpsDisplay, */ *pUpsLcd;
//volatile struct upsDataStrucT upsOne, upsTwo, upsBoss, *pUpsOne, *pUpsTwo, *pUpsBoss, *pUps;
#ifdef SNMP_MIMIC
	volatile struct snmpDataStruct charger;				// extern from main.c
#endif
#if (defined THALES_CHARGER)
	volatile struct snmpDataStruct charger;				// extern from main.c
#endif

// Max number of bars in the capacity and load display
#define NUM_BATT_BARS 10
#define NUM_LOAD_BARS 10
/*
// Bypass states
#define BYPASS_INACTIVE 0
#define BYPASS_ACTIVE 1
*/
// Define the button labels for on and off states
#define DISPLAY_OFF_BUTTON_LABELS\
	LCD_putbuf((uint8_t *)"On  ", 4, 0, 16);\
	LCD_putbuf((uint8_t *)"    ", 4, 1, 16);\
	LCD_putbuf((uint8_t *)"Mute", 4, 2, 16);\
	LCD_putbuf((uint8_t *)"More", 4, 3, 16)

#define DISPLAY_BUTTON_LABELS\
	LCD_putbuf((uint8_t *)"Off ", 4, 0, 16);\
	LCD_putbuf((uint8_t *)"    ", 4, 1, 16);\
	LCD_putbuf((uint8_t *)"Mute", 4, 2, 16);\
	LCD_putbuf((uint8_t *)"More", 4, 3, 16)

#define DISPLAY_FAULT_BUTTON_LABELS\
	LCD_putbuf((uint8_t *)"On  ", 4, 0, 16);\
	LCD_putbuf((uint8_t *)"Off ", 4, 1, 16);\
	LCD_putbuf((uint8_t *)"Mute", 4, 2, 16);\
	LCD_putbuf((uint8_t *)"More", 4, 3, 16)

// The option screen is called by the version screen, change button 4 based on use of option screen
#if !defined LCD_OPTION_DISABLED
	#define DISPLAY_VERSION_BUTTON_LABELS\
		LCD_putbuf((uint8_t *)"Back", 4, 0, 16);\
		LCD_putbuf((uint8_t *)"    ", 4, 1, 16);\
		LCD_putbuf((uint8_t *)"    ", 4, 2, 16);\
		LCD_putbuf((uint8_t *)"More", 4, 3, 16)		// if options enabled, show "More" button label
#else
	#define DISPLAY_VERSION_BUTTON_LABELS\
		LCD_putbuf((uint8_t *)"Back", 4, 0, 16);\
		LCD_putbuf((uint8_t *)"    ", 4, 1, 16);\
		LCD_putbuf((uint8_t *)"    ", 4, 2, 16);\
		LCD_putbuf((uint8_t *)"    ", 4, 3, 16)		// otherwise don't show label
#endif

//#if LCD_DISPLAY == TRUE		// So far, only works through LCD buttons
	volatile int comBypass = FALSE;	// tells router to directly connect master to slave (UPS to Serial Port)
//#endif

// LCD
typedef enum {
    LCD_OPT_START,
    LCD_OPT_CHOOSE,
    LCD_OPT_EXIT,
    LCD_OPT_TIMEOUT
} otherMenuStatesT;

int otherMenu = FALSE;
typedef enum {
	MS_OPERATIONAL,
	MS_OPTION,
	MS_VERSION,
	MS_PARAMETER,
	MS_COM_BYPASS,
	MS_NONE
} menuScreenT;
menuScreenT menuScreen = MS_OPERATIONAL, menuScreenLast = MS_NONE;
int optionMenuDone = TRUE;

struct timeT otherMenuTimeout;
ups_states_t lastUpsStateLcd = UPS_COM_SHUTDOWN;
otherMenuStatesT otherMenuState, otherMenuStateLast = LCD_OPT_EXIT;
int lcdButtonsPressed = 0, lcdButtonsPressedLast = 1;
int fakeButtonState = 0;	// set with bit representing buttons
// LCD Manager sets for other routines to read
#define LCD_BAR_START 5
#define LCD_BAR_END 10
#define LCD_BAR_NUM (LCD_BAR_END - LCD_BAR_START)
#define LCD_BAR_MAX_VERT 5	// Number of vertical lines in LCD character (5x8)

// Function prototypes

void lcd_debug(uint8_t rowbyte, uint8_t charbyte);

uint8_t lcd_debug_pos[4] = {0,0,0,0};
void lcd_debug(uint8_t row, uint8_t character){
	LCD_putc_precise(character, row, lcd_debug_pos[row]++);
	if (lcd_debug_pos[row] == 20) {
		lcd_debug_pos[row] = 0;
		LCD_putc_precise(' ', row, 0);
	}
	else LCD_putc_precise(' ', row, lcd_debug_pos[row]);
}
// This takes an argument of what buttons you want to check for the pressed state
// if you want button 1 just send BUTTON1, if you want buttons 1 and 2 send
// BUTTON1+BUTTON2, etc.  It will return True if those button are set and false
// if any one of the isn't set.
// FakeButtonState is set by lcdManager to button presses it wants other functions
// to see and act on.
// FB_CLEAR can be sent alone or with button test mask to clear fakeButtonState so more
// than one button or combonation can be tested in a row but the clear must be sent after
// a set of tests or it will look like "sticky button" syndrome

int fakeButton(int whichButton) {

	int test, result, clear;

	if (whichButton & FB_CLEAR) {
		clear = TRUE;
		whichButton = whichButton & ~FB_CLEAR;	// remove clear bit for switch test
	}
	else {
		clear = FALSE;
	}
	test = whichButton & fakeButtonState;		// Mask buttonState with requested buttons

	if (test == whichButton) {					// if state matches request, all requested buttons pressed
		result = TRUE;							// return result...easy
	}
	else {
		result = FALSE;
	}
	if (clear == TRUE) {				// command sent to clear fakeButtonState
		fakeButtonState = 0;					// results have been read, reset fake buttons
	}
	return result;								// pass the news back to calling function
}

void lcdOptionScreen(volatile struct upsDataStrucT *upsData) {

	static volatile int optionNumber = 0;
	#if !defined LCD_BYPASS_OPTION_DISABLED		// show and allow bypass option change
		int temp;									// prevent compiler warning
	#endif

	#if ((LCD_DISPLAY==TRUE) && (!defined LCD_OPTION_DISABLED))
		typedef enum {
			AUTOSTART_ON_OPT,
			AUTOSTART_OFF_OPT,
			AUTOSTART_NULL
		} optionSelectionsAutostartT;
		static optionSelectionsAutostartT optionSelectionAutostart;
	
		char optionAutostartStr[][20] = {
									   "Autostart On   ",
									   "Autostart Off  "
								   };
	#endif
	
	typedef enum {
		FREQ_AUTO,
		FREQ_50,
		FREQ_60,
		FREQ_NULL
	} optionSelectionsFreqT;
		
	#if !defined LCD_AUTOSTART_OPTION
		static optionSelectionsFreqT optionSelectionFreq;
		
		char optionFreqStr[][20] = {
									   "Freq Out Auto  ",
									   "Freq Out 50Hz  ",
									   "Freq Out 60Hz  "
								   };
	#endif											// !defined LCD_AUTOSTART_OPTION

	typedef enum {
		BYP_STARTUP,
		BYP_DYNAMIC,
		BYP_BOTH,
		BYP_NONE,
		BYP_NULL
	} optionSelectionsBypT;
		
	#if !defined LCD_AUTOSTART_OPTION
		static optionSelectionsBypT optionSelectionByp;

		char optionBypStr[][20] = {
									  "Byp Startup    ",
									  "Byp Dynamic    ",
									  "Byp Dyn+Start  ",
									  "Byp Fail Only  "
								  };
	
	#endif											// !defined LCD_AUTOSTART_OPTION

	switch (otherMenuState) {
	case LCD_OPT_START:
		if (otherMenuState != otherMenuStateLast) {
			otherMenuStateLast = otherMenuState;
		}
		LCD_clear();
		LCD_putstr((ubyte *)"Option Settings     ", 0, 0);
		LCD_putbuf((uint8_t *)"Done", 4, 0, 16);
		LCD_putbuf((uint8_t *)"Up  ", 4, 1, 16);
		LCD_putbuf((uint8_t *)"Down", 4, 2, 16);
		LCD_putbuf((uint8_t *)"Pick", 4, 3, 16);
		#if defined LCD_AUTOSTART_OPTION
			if (upsData->bank1 & BIT1) {				// bit 1 = switch 2, 1= autostart
				optionSelectionAutostart = AUTOSTART_ON_OPT;
				LCD_putstr((ubyte *) optionAutostartStr[AUTOSTART_ON_OPT], 2, 1);
			} else {									// bit 1 = switch 2, 0 = manual start
				optionSelectionAutostart = AUTOSTART_OFF_OPT;
				LCD_putstr((ubyte *) optionAutostartStr[AUTOSTART_OFF_OPT], 2, 1);
			}
		#else											// !defined LCD_AUTOSTART_OPTION, freq option
			if (upsData->bank2 & BIT8) {		// bit 8 = switch 9
				optionSelectionFreq = FREQ_AUTO;
				LCD_putstr((ubyte *) optionFreqStr[FREQ_AUTO], 2, 1);
			} else {
				if (upsData->bank3 & BIT8) {		// bit 8 = switch 9
					optionSelectionFreq = FREQ_50;
					LCD_putstr((ubyte *) optionFreqStr[FREQ_50], 2, 1);
				} else {
					optionSelectionFreq = FREQ_60;
					LCD_putstr((ubyte *) optionFreqStr[FREQ_60], 2, 1);
				}
			}
		#endif											// defined LCD_AUTOSTART_OPTION
		#if !defined LCD_BYPASS_OPTION_DISABLED		// show and allow bypass option change
			temp = 0;
			if (!(upsData->bank3 & BIT1)) {			// bit 1 = switch 2, off
				temp = BIT0;							// indicates Dynamic Bypass On
			}
			if (!(upsData->bank3 & BIT0)) {			// bit 0 = switch 1, off
				temp = temp + BIT1;						// indicates Startup Bypass On
			}
			switch (temp) {
			case 0:
				optionSelectionByp = BYP_NONE;
				LCD_putstr((ubyte *) optionBypStr[BYP_NONE], 3, 1);
				break;
			case 1:
				optionSelectionByp = BYP_DYNAMIC;
				LCD_putstr((ubyte *) optionBypStr[BYP_DYNAMIC], 3, 1);
				break;
			case 2:
				optionSelectionByp = BYP_STARTUP;
				LCD_putstr((ubyte *) optionBypStr[BYP_STARTUP], 3, 1);
				break;
			case 3:
				optionSelectionByp = BYP_BOTH;
				LCD_putstr((ubyte *) optionBypStr[BYP_BOTH], 3, 1);
				break;
			}
		#endif
		otherMenuState = LCD_OPT_CHOOSE;
		break;
	case LCD_OPT_CHOOSE:
		if (otherMenuState != otherMenuStateLast) {
			otherMenuStateLast = otherMenuState;
			optionNumber = 0;
			LCD_putbuf((uint8_t *)">", 1, optionNumber+2, 0);
		}
		#if !defined LCD_BYPASS_OPTION_DISABLED		// show and allow bypass option change
			if (lcdButtonsPressed & BUTTON2) {	// Up button pressed
				otherMenuTimeout = getTime();
				LCD_putbuf((uint8_t *)" ", 1, optionNumber+2, 0);
				optionNumber--;
				if (optionNumber < 0) {
					optionNumber = 1;		// rollover
				}
				LCD_putbuf((uint8_t *)">", 1, optionNumber+2, 0);
			}
			if (lcdButtonsPressed & BUTTON3) {	// Down button pressed
				otherMenuTimeout = getTime();
				LCD_putbuf((uint8_t *)" ", 1, optionNumber+2, 0);
				optionNumber++;
				if (optionNumber > 1) {
					optionNumber = 0;		// rollover
				}
				LCD_putbuf((uint8_t *)">", 1, optionNumber+2, 0);
			}
		#endif
		if (lcdButtonsPressed & BUTTON4) {	// set button pressed
			otherMenuTimeout = getTime();
			#if !defined LCD_AUTOSTART_OPTION
				if (optionNumber == 0) {	// frequency option
					optionSelectionFreq++;
					if (optionSelectionFreq == FREQ_NULL) {	// top of enum values
						optionSelectionFreq = FREQ_AUTO;	// roll over to bottom
					}
					LCD_putstr((ubyte *) optionFreqStr[optionSelectionFreq], 2, 1);
				} else {								// Bypass option
					optionSelectionByp++;
					if (optionSelectionByp == BYP_NULL) {	// top of enum values
						optionSelectionByp = BYP_STARTUP;	// roll over to bottom
					}
					LCD_putstr((ubyte *) optionBypStr[optionSelectionByp], 3, 1);
					//					LCD_putbuf((uint8_t *) optionBypStr[optionSelectionByp],
					//						strlen(optionBypStr[optionSelectionByp]), 3, 1);
				}
			#else										// defined LCD_AUTOSTART_OPTION
				if (optionNumber == 0) {				// Autostart option
					optionSelectionAutostart++;
					if (optionSelectionAutostart == AUTOSTART_NULL) {	// top of enum values
						optionSelectionAutostart = AUTOSTART_ON_OPT;	// roll over to bottom
					}
					LCD_putstr((ubyte *) optionAutostartStr[optionSelectionAutostart], 2, 1);
				}
			#endif										// !defined LCD_AUTOSTART_OPTION
		}
		if (lcdButtonsPressed & BUTTON1) {			// Done button pressed
			otherMenuState = LCD_OPT_EXIT;
		}
		if (timer(otherMenuTimeout, 30000))  {	// Or user timeout
			otherMenuState = LCD_OPT_TIMEOUT;
		}
		break;
	case LCD_OPT_EXIT:
		if (otherMenuState != otherMenuStateLast) {
			otherMenuStateLast = otherMenuState;
		}
		#if !defined LCD_AUTOSTART_OPTION
			switch (optionSelectionFreq) {
			case FREQ_AUTO:
				masterCmdAddBoth("^RSETF01");	// Send commands to one or both UPS to set bank switches for this option
				break;
			case FREQ_50:
				masterCmdAddBoth("^RSETF02");	// Send commands to one or both UPS to set bank switches for this option
				break;
			case FREQ_60:
				masterCmdAddBoth("^RSETF03");	// Send commands to one or both UPS to set bank switches for this option
				break;
			}
		#else									// defined LCD_AUTOSTART_OPTION
			switch (optionSelectionAutostart) {
			case AUTOSTART_ON_OPT:
				masterCmdAddBoth("^RSETA01");	// Send commands to one or both UPS to set bank switches for this option
				break;
			case AUTOSTART_OFF_OPT:
				masterCmdAddBoth("^RSETA02");	// Send commands to one or both UPS to set bank switches for this option
				break;
			}
		#endif									// !defined LCD_AUTOSTART_OPTION
		#if !defined LCD_BYPASS_OPTION_DISABLED	// show and allow bypass option change
			switch (optionSelectionByp) {
			case BYP_STARTUP:
				masterCmdAddBoth("^RSETB01");	// Send commands to one or both UPS to set bank switches for this option
				break;
			case BYP_DYNAMIC:
				masterCmdAddBoth("^RSETB02");	// Send commands to one or both UPS to set bank switches for this option
				break;
			case BYP_BOTH:
				masterCmdAddBoth("^RSETB03");	// Send commands to one or both UPS to set bank switches for this option
				break;
			case BYP_NONE:
				masterCmdAddBoth("^RSETB04");	// Send commands to one or both UPS to set bank switches for this option
				break;
			}
		#endif									// !defined LCD_BYPASS_OPTION_DISABLED
		// fall through
	default:								// if no associated case...leave
	case LCD_OPT_TIMEOUT:
		if (otherMenuState != otherMenuStateLast) {
			otherMenuStateLast = otherMenuState;
		}
		otherMenu = FALSE;
		optionNumber = 0;					// reset for next time
		otherMenuState = LCD_OPT_START;	//   "
		otherMenuStateLast = LCD_OPT_EXIT;//   "
		lastUpsStateLcd = UPS_NULL;		// going back to control screen, force refresh
		optionMenuDone = TRUE;
		break;
	}	// end select
}

void lcdOperatingScreen(volatile struct upsDataStrucT *upsData) {

	static volatile int infoLine = 0, infoLineOn = FALSE, infoLinePause = 0;// parametric information on 4th line of display
	volatile char tempStr[25], stemp[10];
	volatile long ltemp1, ltemp2;
	long val1, val2;                 // temporary long values  - used to fix IAR volatile reference issues             
	float tmpfloat1, tmpfloat2;      // temporary float values -   ditto above
        
	if (upsData->tAmbMode == ON_ALARM) { 		// Temperature alarm
		LED0_OFF;								// Indicates UPS On
		LED1_OFF;								// Battery Warn, yellow on bat, red Low bat
		LED2_RED;								// Indicates Overload, yellow Alarm, Red Trip
		LED3_RED;								// Indicates Temperature Alarm, S/D, Bypass, Fault
		LCD_putbuf((uint8_t *)"    Overtemp    ", 16, 0, 0);
		update_LCD_bar(upsData->batChgPct, 1);	// Update battery level on the LCD
		update_LCD_bar(upsData->loadPctOut, 2);	// Update load level on the LCD
		DISPLAY_BUTTON_LABELS;
	} else {
		#if UPS_STATE_CONTROL==TRUE						// Router UPS state machine in charge
			if (upsData->upsState != lastUpsStateLcd) {	// UPS State Changed
				LCD_clear();
			}
			switch(upsData->upsState) {
			case UPS_INIT:
				if (upsData->upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsData->upsState;
					LED0_OFF;
					LED1_OFF;
					LED2_OFF;
					LED3_OFF;
					LCD_putstr((ubyte *)VERSION, 0, 0);
					LCD_putbuf((ubyte *)"Initializing", 12, 1, 0);
				}
				break;
			case UPS_OFF:
				if (upsData->upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsOne.upsState;
					if (upsBoss.invOverloadTrip == FAULT) {
						LED0_OFF;
						LED1_OFF;
						LED2_RED;
						LED3_RED;
						if (upsData->bypassMode == ON) {		// overload bypass condition
							LCD_putbuf((uint8_t *)"Overload Bypass ", 16, 0, 0);
							DISPLAY_FAULT_BUTTON_LABELS;
						} else {
							LCD_putbuf((uint8_t *)" Overload Trip  ", 16, 0, 0);
							DISPLAY_OFF_BUTTON_LABELS;
						}
					} else {
						LED0_OFF;
						LED1_OFF;
						LED2_OFF;
						LED3_OFF;
						LCD_putbuf((uint8_t *)"    Load Off    ", 16, 0, 0);
						DISPLAY_OFF_BUTTON_LABELS;
					}
					infoLineOn = TRUE;
				}
				update_LCD_bar(upsData->batChgPct, 1);	// Update battery level on the LCD
				update_LCD_bar(upsData->loadPctOut, 2);	// Update load level on the LCD
				break;
			case UPS_SHUTDOWN:								// Shutdown, transition state
				if (upsData->upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsData->upsState;
					LED0_OFF;
					LED1_OFF;
					LED2_OFF;
					LED3_OFF;
					LCD_putbuf((uint8_t *)"    Shutdown    ", 16, 0, 0);
					DISPLAY_OFF_BUTTON_LABELS;
					infoLineOn = FALSE;
				}
				infoLineOn = FALSE;
				break;
			case UPS_ON_UTIL:
				if (upsData->upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsData->upsState;
					LED0_GRN;
					LED1_GRN;
					LED3_GRN;
					LCD_putbuf((uint8_t *)"     AC On      ", 16, 0, 0);
					DISPLAY_BUTTON_LABELS;
					infoLineOn = TRUE;
				}
				#if (defined SNMP_MIMIC)						// if charger not talking turn bat LED yellow
					if (charger.comOkay == FALSE) {
						LED1_YLW;
					} else {
						LED1_GRN;
					}
				#endif
				if (upsData->loadPctOut >= 115.0) {			// overload
					LED2_RED;
				}
				else {
					if (upsData->loadPctOut >= 105.0) {		// overload warning
						LED2_YLW;
					}
					else {										// load okay
						LED2_GRN;
					}
				}
				update_LCD_bar(upsData->batChgPct, 1);			// Update battery level on the LCD
				update_LCD_bar(upsData->loadPctOut, 2);		// Update load level on the LCD
				break;
			case UPS_ON_BAT:
				if (upsData->upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsData->upsState;
					LCD_putbuf((uint8_t *)"   On Battery   ", 16, 0, 0);
					DISPLAY_BUTTON_LABELS;
					infoLineOn = FALSE;
				}
				update_LCD_bar(upsData->batChgPct, 1);			// Update battery level on the LCD
				update_LCD_bar(upsData->loadPctOut, 2);		// Update load level on the LCD
				// take remaining battery run time in seconds and convert to min:sec
				ltemp1 = upsData->estSecBat;
				ltemp2 = ltemp1/60;

//				ltoa(ltemp2, (char *) upsData->estTimeStr);
				sprintf( (char *) upsData->estTimeStr,"%ld", ltemp2);
                                
				strcat((char *) upsData->estTimeStr,":");

//				ltemp1 -= ltemp2*60;            // no amount of parenthisis seemed to help with the ordering
				val1 = ltemp1;                  // val1 and val2 aren't used anywhere else
				val2 = ltemp2;                  //   so they don't need to be "volatile"...
				ltemp1 = (val1 - (val2*60));    
                                
				if (ltemp1 < 10) {								// want two digits for seconds if 1 number pad with leading zero
					strcat((char *) upsData->estTimeStr,"0");
				}

//				ltoa(ltemp1, (char *) stemp);
				sprintf( (char *) stemp, "%ld", ltemp1);
                                
				strcat((char *) upsData->estTimeStr,(char *) stemp);
				sprintf((char *)tempStr, "Time %s       ",(char *)upsData->estTimeStr);
				LCD_putbuf((ubyte *) tempStr, 15, 3, 0);		// str  #char, #row, start column
				#if DUAL_BOARD == TRUE							// Dual UPS configuration i.e. 5KVA
					if (upsBoss.batWarnFlag) {
						LED0_RED;
						LED1_RED;
						LED3_OFF;
					} else {
						LED0_RED;
						LED1_YLW;
						LED3_OFF;
					}

//					if ( ((upsData->voltBat <= upsData->vBatWarn) || (upsData->estSecBat < 180) )
//							&& (upsBoss.batWarnFlag == FALSE) ) {                                 
					tmpfloat1 = upsData->voltBat ;
					tmpfloat2 = upsData->vBatWarn;
					val1 = upsData->estSecBat;
					if ( ( (tmpfloat1 <= tmpfloat2) || (val1 < 180)) && (upsBoss.batWarnFlag == FALSE))
					{
						upsBoss.batWarnFlag = TRUE;			// used to see if alarm is active
						LCD_putbuf((uint8_t *)"  Low Battery   ", 16, 0, 0);
					}
				#else
					if (upsData->batSts != 0) {
						upsBoss.batWarnFlag = TRUE;			// used to see if alarm is active
						LCD_putbuf((uint8_t *)"  Low Battery   ", 16, 0, 0);
						LED0_RED;
						LED1_RED;
						LED3_OFF;
					}
					else {
						LCD_putbuf((uint8_t *)"   On Battery   ", 16, 0, 0);
						LED0_RED;
						LED1_YLW;
						LED3_OFF;
					}
				#endif
				if (upsData->loadPctOut >= 115.0) {			// overload
					LED2_RED;
				}
				else {
					if (upsData->loadPctOut >= 105.0) {		// overload warning
						LED2_YLW;
					}
					else {									// load okay
						LED2_GRN;
					}
				}
				break;
			case UPS_BYPASS:
				if (upsData->upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsData->upsState;
					LED0_OFF;
					LED1_OFF;
					LED2_OFF;
					LED3_RED;
					LCD_putbuf((uint8_t *)"   On Bypass     ", 16, 0, 0);
					DISPLAY_FAULT_BUTTON_LABELS;
					infoLineOn = TRUE;
				}
				update_LCD_bar(upsData->batChgPct, 1);	// Update battery level on the LCD
				update_LCD_bar(upsData->loadPctOut, 2);	// Update load level on the LCD
				break;
			case UPS_FAULT:
				if (upsData->upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsData->upsState;
					LED0_OFF;
					LED1_OFF;
					LED2_OFF;
					LED3_RED;
					LCD_putbuf((uint8_t *)"     Fault       ", 15, 0, 0);
					DISPLAY_FAULT_BUTTON_LABELS;
					infoLineOn = FALSE;
				}
				if (upsData->bypassMode == ON){		// if bypass active
					LCD_putbuf((uint8_t *)"   On Bypass     ", 16, 3, 0);
				}
				else {
					LCD_putbuf((uint8_t *)"                ", 16, 3, 0);
				}
				update_LCD_bar(upsData->batChgPct, 1);	// Update battery level on the LCD
				update_LCD_bar(upsData->loadPctOut, 2);	// Update load level on the LCD
				break;
			case UPS_COM_SHUTDOWN:
				if (upsData->upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsData->upsState;
					LED0_OFF;
					LED1_OFF;
					LED2_OFF;
					LED3_RED;
					LCD_putbuf((uint8_t *)"  Bad Com Link  ", 16, 0, 0);
					DISPLAY_BUTTON_LABELS;
					infoLineOn = FALSE;
				}
				update_LCD_bar(upsData->batChgPct, 1);	// Update battery level on the LCD
				update_LCD_bar(upsData->loadPctOut, 2);	// Update load level on the LCD
				break;
			default:
				if (upsData->upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsData->upsState;
					LED0_OFF;
					LED1_OFF;
					LED2_OFF;
					LED3_RED;
					LCD_putbuf((uint8_t *)" Unknown State  ", 16, 0, 0);
					DISPLAY_FAULT_BUTTON_LABELS;
					infoLineOn = FALSE;
				}
				update_LCD_bar(upsData->batChgPct, 1);	// Update battery level on the LCD
				update_LCD_bar(upsData->loadPctOut, 2);	// Update load level on the LCD
				break;
			}
		#else												// UPS1 in charge
			if (upsOne.invMode == AUTO_OFF) {				// fake ups states for display
				upsOne.upsState = UPS_OFF;
			} else if (upsOne.dcMode == AUTO_ON) {
				upsOne.upsState = UPS_ON_BAT;
			} else {
				upsOne.upsState = UPS_ON_UTIL;
			}
			if (upsOne.upsState != lastUpsStateLcd) {	// UPS State Changed
				LCD_clear();
			}
			switch(upsOne.upsState) {
			case UPS_INIT:
				if (upsOne.upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsOne.upsState;
					LED0_OFF;
					LED1_OFF;
					LED2_OFF;
					LED3_OFF;
					//LCD_putstr((ubyte *)VERSION, 0, 0);
					//LCD_putbuf((ubyte *)"Initializing", 12, 1, 0);
					upsOne.upsState = UPS_OFF;
				}
				break;
			case UPS_OFF:
				if (upsOne.upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsOne.upsState;
					if (upsBoss.invOverloadTrip == FAULT) {
						LED0_OFF;
						LED1_OFF;
						LED2_RED;
						LED3_RED;
						LCD_putbuf((uint8_t *)" Overload Trip  ", 16, 0, 0);
					} else {
						LED0_OFF;
						LED1_OFF;
						LED2_OFF;
						LED3_OFF;
						LCD_putbuf((uint8_t *)"    Load Off    ", 16, 0, 0);
					}
					DISPLAY_OFF_BUTTON_LABELS;
					infoLineOn = TRUE;
				}
				update_LCD_bar(upsBoss.batChgPct, 1);	// Update battery level on the LCD
				update_LCD_bar(upsBoss.loadPctOut, 2);	// Update load level on the LCD
				break;
			case UPS_ON_UTIL:
				if (upsOne.upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsOne.upsState;
					LED0_GRN;
					LED1_GRN;
					LED3_GRN;
					LCD_putbuf((uint8_t *)"     AC On      ", 16, 0, 0);
					DISPLAY_BUTTON_LABELS;
					infoLineOn = TRUE;
				}
				if (upsBoss.loadPctOut >= 115.0) {			// overload
					LED2_RED;
				}
				else {
					if (upsBoss.loadPctOut >= 105.0) {		// overload warning
						LED2_YLW;
					}
					else {										// load okay
						LED2_GRN;
					}
				}
				update_LCD_bar(upsBoss.batChgPct, 1);			// Update battery level on the LCD
				update_LCD_bar(upsBoss.loadPctOut, 2);		// Update load level on the LCD
				break;
			case UPS_ON_BAT:
				if (upsOne.upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsOne.upsState;
					LCD_putbuf((uint8_t *)"   On Battery   ", 16, 0, 0);
					DISPLAY_BUTTON_LABELS;
					infoLineOn = FALSE;
				}
				update_LCD_bar(upsBoss.batChgPct, 1);			// Update battery level on the LCD
				update_LCD_bar(upsBoss.loadPctOut, 2);		// Update load level on the LCD
				// take remaining battery run time in seconds and convert to min:sec
				ltemp1 = upsBoss.estSecBat;
				ltemp2 = ltemp1/60;

//				ltoa(ltemp2, (char *) upsBoss.estTimeStr);
				sprintf( (char *) upsBoss.estTimeStr,"%ld", ltemp2);

				strcat((char *) upsBoss.estTimeStr,":");
				ltemp1 -= ltemp2*60;
				if (ltemp1 < 10) {								// want two digits for seconds if 1 number pad with leading zero
					strcat((char *) upsBoss.estTimeStr,"0");
				}

//				ltoa(ltemp1, (char *) stemp);
				sprintf( (char *) stemp,"%ld", ltemp1);

				strcat((char *) upsBoss.estTimeStr,(char *) stemp);
				sprintf((char *)tempStr, "Time %s       ",(char *)upsBoss.estTimeStr);
				LCD_putbuf((ubyte *) tempStr, 15, 3, 0);		// str  #char, #row, start column
				#if DUAL_BOARD == TRUE							// Dual UPS configuration i.e. 5KVA
					if (upsBoss.batWarnFlag) {
						LED0_RED;
						LED1_RED;
						LED3_OFF;
					} else {
						LED0_RED;
						LED1_YLW;
						LED3_OFF;
					}
					if ( ((upsBoss.voltBat <= upsBoss.vBatWarn) || (upsBoss.estSecBat < 180) )
							&& (upsBoss.batWarnFlag == FALSE) ) {
						upsBoss.batWarnFlag = TRUE;			// used to see if alarm is active
						LCD_putbuf((uint8_t *)"  Low Battery   ", 16, 0, 0);
					}
				#else
					if (upsBoss.batSts != 0) {
						upsBoss.batWarnFlag = TRUE;			// used to see if alarm is active
						LCD_putbuf((uint8_t *)"  Low Battery   ", 16, 0, 0);
						LED0_RED;
						LED1_RED;
						LED3_OFF;
					}
					else {
						LCD_putbuf((uint8_t *)"   On Battery   ", 16, 0, 0);
						LED0_RED;
						LED1_YLW;
						LED3_OFF;
					}
				#endif
				if (upsBoss.loadPctOut >= 115.0) {			// overload
					LED2_RED;
				}
				else {
					if (upsBoss.loadPctOut >= 105.0) {		// overload warning
						LED2_YLW;
					}
					else {									// load okay
						LED2_GRN;
					}
				}
				break;
			case UPS_BYPASS:
				if (upsOne.upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsOne.upsState;
					LED0_OFF;
					LED1_OFF;
					LED2_OFF;
					LED3_RED;
					LCD_putbuf((uint8_t *)"   On Bypass     ", 16, 0, 0);
					DISPLAY_FAULT_BUTTON_LABELS;
					infoLineOn = TRUE;
				}
				update_LCD_bar(upsBoss.batChgPct, 1);	// Update battery level on the LCD
				update_LCD_bar(upsBoss.loadPctOut, 2);	// Update load level on the LCD
				break;
			case UPS_FAULT:
				if (upsOne.upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsOne.upsState;
					LED0_OFF;
					LED1_OFF;
					LED2_OFF;
					LED3_RED;
					LCD_putbuf((uint8_t *)"     Fault       ", 15, 0, 0);
					DISPLAY_FAULT_BUTTON_LABELS;
					infoLineOn = FALSE;
				}
				if (upsBoss.bypassMode == ON){		// if bypass active
					LCD_putbuf((uint8_t *)"   On Bypass     ", 16, 3, 0);
				}
				else {
					LCD_putbuf((uint8_t *)"                ", 16, 3, 0);
				}
				update_LCD_bar(upsBoss.batChgPct, 1);	// Update battery level on the LCD
				update_LCD_bar(upsBoss.loadPctOut, 2);	// Update load level on the LCD
				break;
			case UPS_COM_SHUTDOWN:
				if (upsOne.upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsOne.upsState;
					LED3_RED;
					LED0_OFF;
					LED1_OFF;
					LED2_OFF;
					LCD_putbuf((uint8_t *)"  Bad Com Link  ", 16, 0, 0);
					DISPLAY_BUTTON_LABELS;
					infoLineOn = FALSE;
				}
				update_LCD_bar(upsBoss.batChgPct, 1);	// Update battery level on the LCD
				update_LCD_bar(upsBoss.loadPctOut, 2);	// Update load level on the LCD
				break;
			default:
				if (upsOne.upsState != lastUpsStateLcd) {	// UPS State Changed
					lastUpsStateLcd = upsOne.upsState;
					LED0_OFF;
					LED1_OFF;
					LED2_OFF;
					LED3_RED;
					LCD_putbuf((uint8_t *)" Unknown State  ", 16, 0, 0);
					DISPLAY_FAULT_BUTTON_LABELS;
					infoLineOn = FALSE;
				}
				update_LCD_bar(upsBoss.batChgPct, 1);	// Update battery level on the LCD
				update_LCD_bar(upsBoss.loadPctOut, 2);	// Update load level on the LCD
				break;
			}
		#endif
		if (infoLineOn == TRUE) {
			if (upsData->batCond != NORMAL) {		// battery fault
				switch (upsData->batCond) {
				case WARN:
					strcpy((char *)tempStr, "Battery Warning   ");
					break;
				case FAULT:
					strcpy((char *)tempStr, "Bad Batteries     ");
					break;
				case OVER_VOLTAGE:
					strcpy((char *)tempStr, "Batteries OV      ");
					break;
				default:
					if (timer(upsBoss.timeStarted, 45000)) {			// wait until we get good information from charger
						strcpy((char *)tempStr, "Batteries?       ");
					} else {
						strcpy((char *)tempStr, "                 "); // keep bottom line clear during timeout
					}
					break;
				}
				// string, number of characters, row number, starting column number				
				LCD_putbuf((ubyte *) tempStr, 15, 3, 0);
			} else {
				if (infoLinePause++ >= 5) {					// this called once per second by lcdManager()
					infoLinePause = 0;
					switch (infoLine) {
					case 0:
					    //sprintf((char *)tempStr, "%3.0fW, %1.2fPF   "	,upsData->powOut,upsData->pfOut);                                          
						tmpfloat1 = upsData->powOut;                    
						tmpfloat2 = upsData->pfOut;
						sprintf((char *)tempStr, "%3.0fW, %1.2fPF   "	,tmpfloat1,tmpfloat2);

						LCD_putbuf((ubyte *) tempStr, 15, 3, 0);// str  #char, #row, start column
						infoLine++;
						break;
					case 1:
						sprintf((char *)tempStr, "Freq In %2.1f      ",upsData->freqIn);
						LCD_putbuf((ubyte *) tempStr, 15, 3, 0);// str  #char, #row, start column
						infoLine++;
						break;
					case 2:
						sprintf((char *)tempStr, "Freq Out %2.1f     ",upsData->freqOut);
						LCD_putbuf((ubyte *) tempStr, 15, 3, 0);// str  #char, #row, start column
						infoLine++;
						break;
					case 3:
						if (upsData->upsState == UPS_ON_BAT) {		// Don't show charge current, charger off
							LCD_putbuf((ubyte *) "Charger Off      ", 15, 3, 0);// str  #char, #row, start column
						} else {
							sprintf((char *)tempStr, "Charge %2.2fA    ",upsData->ampChg);
							LCD_putbuf((ubyte *) tempStr, 15, 3, 0);// str  #char, #row, start column
						}
						// if the charger should be talking and it's not, overwrite info line
						#if (defined SNMP_MIMIC)
							// If utility on, charger should be communicating
							if ( ((upsData->upsState == UPS_ON_UTIL) || (upsData->upsState == UPS_OFF))
								&& (charger.comOkay == FALSE)
								&& (upsData->dcMode != AUTO_ON) ) {
								LCD_putbuf((ubyte *) "Charger Com?   ", 15, 3, 0);
							}
						#endif
						infoLine = 0;
						break;
					default:
						infoLine = 0;
						break;
					}
				}
			}
		}
	}
}

void lcdManager(volatile struct upsDataStrucT *upsData) {

	pUpsLcd = upsData;		// set pointer to ups data structure address passed to this routine

	lcdButtonsPressed = buttons_query();			// get number with bits representing pressed buttons
	if (lcdButtonsPressed == BUTTON1) {	// debug
		__NOP;
	}
	if (lcdButtonsPressed != lcdButtonsPressedLast) {
		lcdButtonsPressedLast = lcdButtonsPressed;
		/*
		// reduced tempStr to 25 characters
		sprintf(tempStr, "%s = %d","lcdManager: button change",lcdButtonsPressed);
		addEvent(tempStr,9);
		*/
	}
	/*
	if ( (lcdButtonsPressed & BUTTON4) && (otherMenu == FALSE) ) {	// More button pressed
		otherMenu = TRUE;
		otherMenuState = LCD_OPT_START;
		otherMenuTimeout = getTime();
	}
	if (otherMenu == TRUE) {
		lcdOptionScreen(pUpsLcd);
	}
	else {	// otherMenu False, normal operating menu
		fakeButtonState = lcdButtonsPressed;		// Normal operation, pass buttons to all other functions
		lcdOperatingScreen(pUpsLcd);
	}
	*/

	if (lcdButtonsPressed == 
		(BUTTON2+BUTTON3+BUTTON4) ) {				// Communication Bypass = all buttons pressed
		menuScreen = MS_COM_BYPASS;
	}

	switch (menuScreen) {
	case MS_OPERATIONAL:
		if (menuScreen != menuScreenLast) {
			menuScreenLast = menuScreen;
			lastUpsStateLcd = UPS_NULL;			// force a screen repaint
		}
		if (lcdButtonsPressed & BUTTON4) {			// More button pressed
			menuScreen = MS_VERSION;
		} else {
			fakeButtonState = lcdButtonsPressed;	// Normal operation, pass buttons to all other functions
			if (lcdButtonsPressed == BUTTON1) {	// debug
			    //_NOP();
			    //__asm("nop");
			    __NOP;
			}
			lcdOperatingScreen(pUpsLcd);
		}
		break;
	case MS_OPTION:
		if (menuScreen != menuScreenLast) {
			menuScreenLast = menuScreen;
			otherMenu = TRUE;
			optionMenuDone = FALSE;
			otherMenuState = LCD_OPT_START;
			otherMenuTimeout = getTime();
		}
		lcdOptionScreen(pUpsLcd);
		if (optionMenuDone == TRUE) {				// More button pressed from option screen or timeout
			menuScreen = MS_OPERATIONAL;
		}
		break;
	case MS_VERSION:
		if (menuScreen != menuScreenLast) {
			menuScreenLast = menuScreen;
			LCD_clear();
			DISPLAY_VERSION_BUTTON_LABELS;
			LCD_putbuf((uint8_t *) "UPS Firmware    ", 15, 0, 0);
			#if ((UPS_STATE_CONTROL==TRUE) || (defined LCD_MANAGER_BOSS))
				if (strcmp((char *)upsOne.verSoftware,"Unknown") == 0) {	// version not updated yet
					LCD_putbuf((uint8_t *) "Checking...     ", 15, 1, 0);
				}
				else {
					LCD_putbuf((uint8_t *) upsOne.verSoftware, 15, 1, 0);
				}
			#else
				if (strcmp((char *)upsData->verSoftware,"Unknown") == 0) {	// version not updated yet
					LCD_putbuf((uint8_t *) "Checking...     ", 15, 1, 0);
				}
				else {
					LCD_putbuf((uint8_t *) upsData->verSoftware, 15, 1, 0);
				}
			#endif
			LCD_putbuf((uint8_t *) "Router Firmware ", 15, 2, 0);
			LCD_putstr((ubyte *) upsBoss.verSoftware, 3, 0);	// set on data structure initialization
			otherMenuTimeout = getTime();
		}
		if (lcdButtonsPressed & BUTTON1) {						// Back button pressed
			menuScreen = MS_OPERATIONAL;
		}
		#if !defined LCD_OPTION_DISABLED						// all options not disabled
			if (lcdButtonsPressed & BUTTON4) {						// More button pressed
				menuScreen = MS_OPTION;
			}
		#endif
		if (timer(otherMenuTimeout, 30000))  {					// Or user timeout
			menuScreen = MS_OPERATIONAL;
		}
		break;
	case MS_PARAMETER:
		if (menuScreen != menuScreenLast) {
			menuScreenLast = menuScreen;
		}
		break;
	case MS_COM_BYPASS:
		if (menuScreen != menuScreenLast) {
			menuScreenLast = menuScreen;
			lastUpsStateLcd = UPS_NULL;						// force a screen repaint
			LCD_clear();
			LCD_putstr((ubyte *)"   Communication    ", 0, 0);
			LCD_putstr((ubyte *)"   Bypass, press    ", 1, 0);
			LCD_putstr((ubyte *)"    any button      ", 2, 0);
			LCD_putstr((ubyte *)"    to return       ", 3, 0);
			comBypass = TRUE;
			usart_putstr((char *)"^RDAT23",UPS1_PORT);	// checksum off
			usart_putstr((char *)"^X",UPS1_PORT);	// exit parser
			#if (DUAL_BOARD==TRUE)
				usart_putstr((char *)"^RDAT23",UPS2_PORT);	// checksum off
				usart_putstr((char *)"^X",UPS2_PORT);	// exit parser
			#endif
			otherMenuTimeout = getTime();
		}
		if (timer(otherMenuTimeout, 5000))  {			// Or user timeout
			if (lcdButtonsPressed & 
				(BUTTON1+BUTTON2+BUTTON3+BUTTON4) ) {			// Communication Bypass Off = all buttons pressed
				menuScreen = MS_OPERATIONAL;
				usart_putchar('Q',UPS1_PORT);
				usart_putchar('X',UPS1_PORT);
				usart_putchar(13,UPS1_PORT);
				usart_putchar(13,UPS1_PORT);
				usart_putstr((char *)"^RDAT22",UPS1_PORT);	// checksum back on
				#if (DUAL_BOARD==TRUE)
					usart_putchar('Q',UPS2_PORT);
					usart_putchar('X',UPS2_PORT);
					usart_putchar(13,UPS2_PORT);
					usart_putchar(13,UPS2_PORT);
					usart_putstr((char *)"^RDAT22",UPS2_PORT);	// checksum back on
				#endif
				comBypass = FALSE;
			}
		}
		break;
	default:
		menuScreen = MS_OPERATIONAL;
		break;
	}
}

// pass level and rows 0-3, row 1 = battery, row 2 = load
// will print ex. "Bat |||||| 50%" (verts indicate bar)
void update_LCD_bar(float level, int row) {
	int i;
	int tempStrLen, tempData, tempNumFull, tempPartial;
	char tempStr[10];

	switch (row) {
	case 1:										// battery
		LCD_putbuf((ubyte *) "Bat  ", 5, row, 0);// str  #char, #row, start column
		break;
	case 2:										// battery
		LCD_putbuf((ubyte *) "Load ", 5, row, 0);// str  #char, #row, start column
		break;
	}
	tempData = (int) (level + 0.5);	// convert to int for faster processing
	sprintf(tempStr, "%3d", tempData);
	tempStrLen = strlen(tempStr);
	strcat(tempStr, "%");						// add percent char
	if (tempStrLen == 2) {						// most likely between 10 and 99
		LCD_putbuf((ubyte *)tempStr, 4, row, 11);	// nn% 3char, 3rd row, start column
	}
	else {
		if (tempStrLen == 1) {					// next most likely 0-9
			LCD_putbuf((ubyte *)tempStr, 4, row, 12);
		}
		else {									// 100+
			LCD_putbuf((ubyte *)tempStr, 4, row, 10);
		}
	}
	if (tempData > 100) {						// don't have bars longer than 100%
		tempData = 100;
	}
	tempNumFull =  (tempData * LCD_BAR_NUM)/100;// scale to number of bars, same as (tempData/100) * LCD_BAR_NUM but faster
	tempPartial = tempData - ((tempNumFull * 100) / LCD_BAR_NUM);	// figure the remainder for the partial bar character
	for (i = 0; i < LCD_BAR_NUM; i++) {		// do the bars, 10 bars 10% per bar
		if (i < tempNumFull) {
			LCD_putc_precise(0xFF, row, i+LCD_BAR_START);
		}
		else {
			if (i == tempNumFull) {
				if (tempData == 0) {			// don't show the first bar at 0%
					LCD_putc_precise(BAR0_CHAR, row, i+LCD_BAR_START);	// put in empty bar char
				}
				else {
					LCD_putc_precise((tempPartial/LCD_BAR_MAX_VERT)+1, row, i+LCD_BAR_START);
				}
			}
			else {
				LCD_putc_precise(BAR0_CHAR, row, i+LCD_BAR_START);	// put in empty bar char
			}
		}
	}
}

// Update the battery bars.  This is not elegant, but less overhead than
// doing floating point conversions.
void update_LCD_bat_levels(float level) {
	uint8_t i;
	int tempStrLen, tempData;
	char tempStr[10];

	tempData = (int) (level + 0.5);	// convert to int for faster processing
	sprintf(tempStr, "%3d", tempData);
	tempStrLen = strlen(tempStr);
	strcat(tempStr, "%");						// add percent char
	if (tempStrLen == 2) {						// most likely between 10 and 99
		LCD_putbuf((ubyte *)tempStr, 3, 3, 11);	// nn% 3char, 3rd row, start column
	}
	else {
		if (tempStrLen == 1) {					// next most likely 0-9
			LCD_putbuf((ubyte *)tempStr, 2, 3, 12);
		}
		else {									// 100+
			LCD_putbuf((ubyte *)tempStr, 4, 3, 10);
		}
	}
	for (i = 0; i < NUM_BATT_BARS; i++) {		// do the bars, 10 bars 10% per bar
		if (i < (tempData/10)) {				// do the bars up to percent/10
			LCD_putc_precise(0xFF, 3, i);
		}
		else {
			LCD_putc_precise('_', 3, i);		// rest of chars put underlines
		}
	}

}

// Update the load bars.  This is not elegant, but less overhead than
// doing floating point conversions.
void update_LCD_load_levels(float level) {
	uint8_t i;
	int tempStrLen, tempData;
	char tempStr[10];

	tempData = (int) (level + 0.5);	// convert to int for faster processing
	sprintf(tempStr, "%3d", tempData);
	tempStrLen = strlen(tempStr);
	strcat(tempStr, "%");						// add percent char
	if (tempStrLen == 2) {						// most likely between 10 and 99
		LCD_putbuf((ubyte *)tempStr, 3, 1, 11);	// nn% 3char, 3rd row, start column
	}
	else {
		if (tempStrLen == 1) {					// next most likely 0-9
			LCD_putbuf((ubyte *)tempStr, 2, 1, 12);
		}
		else {									// 100+
			LCD_putbuf((ubyte *)tempStr, 4, 1, 10);
		}
	}
	for (i = 0; i < NUM_BATT_BARS; i++) {		// do the bars, 10 bars 10% per bar
		if (i < (tempData/10)) {				// do the bars up to percent/10
			LCD_putc_precise(0xFF, 1, i);
		}
		else {
			LCD_putc_precise('_', 1, i);		// rest of chars put underlines
		}
	}
}


