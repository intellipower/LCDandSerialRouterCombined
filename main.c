// 5KVA Router

//#include <io430.h>              // default main include file -- IAR workbench

#include "IAR_YES_NO.h"
#ifdef __IAR
	#include "io430.h"
#else
	#include <msp430x54x.h>
#endif

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "types.h"
#include "main.h"

char StrParams[PARAM_NUM_MAX][PARAM_LEN_MAX];   // moved out of main.h to avoid multiple instantiations (lcd.display.h can load main.h)

/*
#include "uscia0_UART.h"
#include "uscia1_UART.h"
#include "uscia2_UART.h"
#include "uscia3_UART.h"
*/
#include "uscia_UART.h"
#include "low_level_init.h"
#include "timera.h"
#include "timerb.h"
#include "leds.h"
#include "utilities.h"
#if LCD_DISPLAY==TRUE
	#include "lcd_display.h"
	#include "port1.h"
	#include "HD44780_display.h"
#endif
#include "system_config.h"						// includes file name for configuration
#include SYSTEM_CONFIG							// configuration header file

/*
Note: the following are notifications sent from the ups, this happens
when a button is pressed on the front panel or an important event occurs

RBUT01 Test button
RBUT02 Silence button
RBUT06 Off button
RBUT08 On button
RBUT12 Bypass button
RBUT25 Overcurrent or overload shutdown has occurred
RBUT26 Special Overcurrent or overload shutdown has occurred, used for overload bypass
	   RBUT25 forces it on bypass this is higher overload to force shutdown, used with NO_BYPASS
	   compiler switch.
RBUT30 UPS started and Lucy and Panel tasks are running
RBUT31 Test Lightshow finished, update the display

*/
// General variables
volatile int rotatingCmdHold = TRUE;		// hold on rotating commands
#if UPS_STATE_CONTROL==FALSE				// use router to control UPS operation
	volatile int upsStartup = TRUE, upsShutdown = FALSE;
#endif
volatile int systemShutdown = TRUE;
volatile int delayedShutdown = FALSE, delayedStartup = FALSE;
volatile struct timeT delayedStartupTime, delayedShutdownTime;
volatile struct timeT timeStarted;

volatile char almStrOne[DISPLAY_LEN],almStrOneLast[DISPLAY_LEN],almStrTwo[DISPLAY_LEN],almStrTwoLast[DISPLAY_LEN];
volatile char loadDisp[DISPLAY_LEN],loadDispLast[DISPLAY_LEN],batDisp[DISPLAY_LEN],batDispLast[DISPLAY_LEN];

volatile ups_states_t upsState = UPS_OFF;
volatile int upsStateRS485turnon = FALSE;
volatile int batteryTrayStatus = FALSE;

volatile portName_t snmpBypassUps;			// in snmp bypass, which ups is connected 1=Master, 2=Slave
extern volatile struct upsDataStrucT upsOne, upsTwo, upsThree, upsBoss, *pUpsOne, *pUpsTwo, *pUpsThree, *pUpsBoss, *pUps;
volatile unsigned int *flashPointer;		// address pointer for flash read/write

#define OP_MODE_STR_MAX 14

char *opModeStr[OP_MODE_STR_MAX] = 
	{
	   "Off",
	   "On",
	   "Auto",
	   "Man",
	   "Off/Auto",
	   "On/Auto",
	   "Slow/Auto",
	   "Med/Auto",
	   "Fast/Auto",
	   "Off/Man",
	   "On/Man",
	   "Slow/Man",
	   "Med/Man",
	   "Fast/Man"
	};

char upsCmd[][CMD_STR_MAX] = 
	{
		 "^RDAT01",		// Main UPS parameters
		 "^RDAT02",		// Secondary parameters
		 "^RDAT03",		// Sub-system modes
		 "^RDAT04",		// user cmd to turn notify on
		 "^RDAT05",		// user cmd to turn notify off
		 "^RDAT06",		// misc parameters
		 "^RDAT07",		// Optoisolator states 4 fields, EPO, BatShort, On/Off, TBD - 0=off, 1=on, 2=not used
		 "^RDAT10",		// All parameters together
		 "^RDAT20",		// Slave mode on
		 "^RDAT21",		// Slave mode off
		 "^RDAT22",		// Checksum on
		 "^RDAT23",		// Checksum off
		 "^RDAT24",		// Display - save LED status
		 "^RDAT25",		// Display - reload saved LED status
		 "^RDAT26",		// Display - all LEDs on
		 "^RDAT27",		// Display - all LEDs off
		 "^RDAT28",		// Display - flash all LEDs
		 "^RDAT30",		// Display - Locally run LED Lightshow
		 "^RDAT31",		// Returns a string of Version UPS Microprocessor Firmware
		 "^RDAT32",		// Sends command to turn on low battery signal relay
		 "^RDAT33",		// Sends command to turn off low battery signal relay
		 //	The next commands just show the prefix, the rest varies so only test the prefix
		 "^RMOD",		// UPS Mode command
		 "^RLOD",		// Main UPS parameters
		 "^RBAT",		// Secondary parameters
		 "^RDIS",		// Sub-system modes
		 "^RALM",		// Sonalert and Aux 1, Aux 2 LED control
		 "^RSET",		// Bank switch set command
		 // The next set are unsolicited messages from the UPS, this is just the prefix ex. ^RBUT30
		 "^RBUT"			// Indicates front panel button press or notification of event i.e. Lightshow done
	};

#if (defined SNMP_MIMIC)
	struct timeT chargerComTimer;
#endif

#define UPS_CMD_ROTATION_MAX (16) 			// this has to point to last item below
upsParsePollEnum_t upsCmdRotation[] = 
{
  RDAT05_NOTIFY_OFF,	// Make sure Notify is turned off, %P puts lots of traffic on com
  RDAT01_PARAMS,		// Request Parameters
  RDAT06_MISC_PARMS,	// The rest of the parms not gotten by RDAT1,2,3
  RDAT03_STATUS,		// Request Status
  RDAT02_PARAMS2,		// Request more Parameters
  RDAT03_STATUS,		// Request Status
  RDAT06_MISC_PARMS,	// The rest of the parms not gotten by RDAT1,2,3
  RDAT01_PARAMS,		// Request Parameters
  RDAT03_STATUS,		// Request Status
  RDAT02_PARAMS2,		// Request more Parameters
  RDAT03_STATUS,		// Request Status
  RDAT06_MISC_PARMS,	// The rest of the parms not gotten by RDAT1,2,3
  /*
  #if defined UCLASS_FA10002
  RDAT07_OPTO_INPUTS,
  #endif
  */
  RDAT03_STATUS,		// Request Status
  RDAT01_PARAMS,		// Request Parameters
  RDAT03_STATUS,		// Request Status
  RDAT02_PARAMS2,		// Request more Parameters
  RDAT03_STATUS			// Request Status
};

snmpParsePollEnumT snmpParsePoll;

volatile char snmpParsePollCmd[SNMP_CMD_MAX][5] = 
	{
		"AP1",
		"AP2",
		"ATR",
		"BTS",
		"MAN",
		"MOD",
		"NOM",
		"OTC",
		"OTD",
		"PSD",		// Timed shutdown
		"ST1",
		"ST2",
		"ST3",
		"ST4",
		"ST5",
		"ST6",
		"ST7",
		"SDA",
		"STD",		// Timed startup
		"STR",
		"UBR",
		"UID",
		"VER",
		"XD1",
		"XD2"
	};

// This takes the enumerated values and puts them in an array mirroring the commands
upsParsePollEnum_t upsParsePoll[] = 
{
	RDAT01_PARAMS,			// Request Parameters
	RDAT02_PARAMS2,			// Request more Parameters
	RDAT03_STATUS,			// Request Status
	RDAT04_NOTIFY_ON,		// Turn on Notify
	RDAT05_NOTIFY_OFF,		// Turn off Notify
	RDAT06_MISC_PARMS,		// Left over parameters
	RDAT07_OPTO_INPUTS,		// Optoisolator states 4 fields, EPO, BatShort, On/Off, TBD - 0=off, 1=on, 2=not used
	RDAT10_COMBO_PARMS,		// All of the others requested together
	RDAT20_SLAVE_ON,		// Slave mode
	RDAT21_SLAVE_OFF,		// Slave mode
	RDAT22_CHKSUM_ON,		// Checksum mode
	RDAT23_CHKSUM_OFF,		// Checksum mode
	RDAT24_DIS_SAVE_LED,	// Display - save LED status
	RDAT25_DIS_LOAD_LED,	// Display - reload saved LED status
	RDAT26_DIS_ON_LED,		// Display - all LEDs on
	RDAT27_DIS_OFF_LED,		// Display - all LEDs off
	RDAT28_DIS_FLASH_LED,	// Display - flash all LEDs
	RDAT30_DIS_LIGHTSHOW,	// Display - Locally run LED Lightshow
	RDAT31_UPS_PROG_VER,	// Returns a string of Version UPS Microprocessor Firmware
	RDAT32_BAT_LOW_RLY_ON,	        // Sends command to turn on low battery signal relay
	RDAT33_BAT_LOW_RLY_OFF,         // Sends command to turn off low battery signal relay
	// The next commands just show the prefix, the rest varies so only test the prefix
	RMOD,				// Mode commands, returns "^1" for success, "^0" for failure
	RLOD,				// Display load bar set command
	RBAT,				// Display battery capacity bar set command
	RDIS,				// Display LEDs On, Bypass, Service Bat, Fault set command
	RALM,				// Sonalert and Aux 1, Aux 2 LED control
	RSET,				// Bank switch set command
	// The next set are unsolicited messages from the UPS, this is just the prefix ex. ^RBUT30
	RBUT,				// Indicates front panel button press or notification of event i.e. Lightshow done

	CMD_END				// special marker indicating no active command
};

// ups_com_handler definitions

volatile int lightshowFlag = FALSE, lightshowLast = FALSE;		// manage Test button activation of LED lightshow

// Router Event definitions
#if defined EVENT_REPORTING_ENABLED
	#define ROUTER_EVENTS_MAX 10
	#define ROUTER_EVENT_LENGTH 200
	struct eventT {
		volatile char instance[ROUTER_EVENTS_MAX][ROUTER_EVENT_LENGTH], strTemp[ROUTER_EVENT_LENGTH];
		volatile int bot, top;
		volatile long lastMsec;
	} event;
	volatile char eventStrTemp[ROUTER_EVENT_LENGTH];
#else
	volatile char eventStrTemp[200];
#endif

// rs485 definitions
#define RS485_CMD_LEN 25
volatile char rs485cmd[RS485_CMD_LEN];
//volatile int rs485cmdEnd = 0;
volatile struct upsDataStrucT *pRS485upsSelected;
volatile char upsSelectedTitle[25];

// LED Display definitions
volatile char ledDisp[DISPLAY_LEN],ledDispLast[DISPLAY_LEN];

// SNMP definitions
volatile struct snmpDataStruct snmp, upsilon, *pSnmp, *pUpsilon;

// Thales System Compiler Flag Variable usage
#ifdef SNMP_MIMIC
	volatile struct snmpDataStruct *pCharger;
	extern volatile struct snmpDataStruct charger;
	volatile struct chargerDataStruct chargerData;
#endif
#if (defined THALES_CHARGER)
	volatile struct snmpDataStruct *pCharger;
	extern volatile struct snmpDataStruct charger;
	volatile struct chargerDataStruct chargerData;
#endif
#ifdef SNMP2
	volatile struct snmpDataStruct snmp2, *pSnmp2;
#endif

void ups_rx_LED_on(void){
	LED1_GRN;
}

void ups_tx_LED_on(void){
	LED1_YLW;
}

// test local router board LEDs
void ledTest(void) {
	volatile struct timeT tempTime;

	tempTime = getTime();

	LED0_RED;
	LED1_RED;
	LED2_RED;
	LED3_RED;

	while (!timer(tempTime, 250)) ;

	LED0_YLW;
	LED1_YLW;
	LED2_YLW;
	LED3_YLW;

	while (!timer(tempTime, 500)) ;

	LED0_GRN;
	LED1_GRN;
	LED2_GRN;
	LED3_GRN;

	while (!timer(tempTime, 750)) ;

	LED0_RED;
	LED1_RED;
	LED2_RED;
	LED3_RED;

	while (!timer(tempTime, 1000)) ;

	LED0_YLW;
	LED1_YLW;
	LED2_YLW;
	LED3_YLW;

	while (!timer(tempTime, 1250)) ;

	LED0_GRN;
	LED1_GRN;
	LED2_GRN;
	LED3_GRN;

	while (!timer(tempTime, 1500)) ;
}

void updateComStatusLeds(void) {

	if (upsOne.upsComState == COM_IDLE) {	// Master UPS
		if (upsOne.comOkay == FALSE) {
			LED1_RED;
		}
		else {
			LED1_GRN;
		}
	}
	if (upsTwo.upsComState == COM_IDLE) {	// Slave UPS
		if (upsTwo.comOkay == FALSE) {
			LED2_RED;
		}
		else {
			LED2_GRN;
		}
	}
	if (snmp.snmpComState == SNMP_IDLE) {			// SNMP Module
		if ( (snmp.comOkay == FALSE) || (snmp.type == FAIL) ) {
			LED2_RED;
		}
		else {
			LED2_GRN;
		}
	}
}

void initEvent(void) {
	#if defined EVENT_REPORTING_ENABLED
		event.top = event.bot = 0;
		event.lastMsec = (long) 0;
	#endif
}

// addEvent(event text[100max], reporting level)
// Note: if you need to massage event text use eventStrTemp, it is sized to handle the same
// length as this routing.
// example:
//			sprintf(eventStrTemp,"ups_com:IDLE entered, ups %d",upsData->upsNumber);
//			addEvent(eventStrTemp,4);
// this also sets the report level to 4, in system_config.h you can set the reporting level
// 0 is the lowest (most) reporting, 10 the least.  If you set the level to 5 it will report everything
// with a reporting level equal to or greater than 5.

void addEvent(volatile char *str, volatile int reportLevel) {

	volatile struct timeT eventTime;
	static volatile char eventStr[200], stemp[20];
	// debug
	static volatile char debugStr[200];
	#if !defined EVENT_REPORTING_ENABLED
		static volatile long lastMsec = 0;
	#endif

	long ltmptime, ltmpmsec;    // variable to help eliminate volatile warnings w/IAR     
                
	strcpy((char *) debugStr, (char *)str);
	// debug end

	//__disable_interrupt();	// TODO - interrupt disable addEvent

	// check string length, if too long substitute following warning.  Subtract from total because
	// addEvent adds difference time between events and day and millisec of runtime for event.
	if (strlen((char *)str) >= (200)) {
		strcpy((char *)str,"Event Error...string sent to addEvent is too long");
	}
	if (reportLevel >= EVENT_REPORT_LEVEL) { // if this event meets setting, then report
		eventTime = getTime();
		/*
		sprintf((char *)eventStr,"%ld, %s, days=%d, msec=%ld",
		        eventTime.msec-event.lastMsec,str,eventTime.days,eventTime.msec);
		 */
		#if defined EVENT_REPORTING_ENABLED

//			ltoa(eventTime.msec-event.lastMsec,(char *) stemp);
			sprintf( (char *) stemp, "%ld", eventTime.msec-event.lastMsec);

		#else
			eventTime = getTime();
//			ltoa(eventTime.msec-lastMsec,(char *) stemp);
//			sprintf( (char *) stemp, "%ld", eventTime.msec-lastMsec); // IAR -- no ltoa
			ltmpmsec = eventTime.msec;
			ltmptime = (ltmpmsec - lastMsec);
			sprintf( (char *) stemp, "%ld", ltmptime);
//			lastMsec = eventTime.msec;
			lastMsec = ltmpmsec;            // just in case eventTime.msec changed since last read
		#endif
		strcpy((char *) eventStr,"*** ");
		strcat((char *) eventStr,(char *) stemp);
		strcat((char *) eventStr,", ");
		strcat((char *) eventStr, (char *) str);
		strcat((char *) eventStr,", days=");

//		ltoa(eventTime.days,(char *) stemp);
		sprintf( (char *) stemp, "%ld", (long) eventTime.days);

		strcat((char *) eventStr,(char *) stemp);
		strcat((char *) eventStr,", msec=");

//		ltoa(eventTime.msec,(char *) stemp);
		sprintf( (char *) stemp, "%ld", eventTime.msec);

		strcat((char *) eventStr,(char *) stemp);
		#if defined EVENT_REPORTING_ENABLED
			strcpy((char *)event.instance[event.top++],(char *)eventStr);
		#endif
		#ifndef RS485LOCKOUT				// RS485 LOCKOUT FLAG because charger_com() and snmp_com() uses RS485 on Thales
		usart_putstr(eventStr,UPSNET_PORT);		// send out RS485 port
		usart_putchar(13,UPSNET_PORT);
		usart_putchar(10,UPSNET_PORT);
		#endif
		#ifdef RS485LOCKOUT_DEBUG					// get around lockout for debug to RS485
		usart_putchar(13,UPSNET_PORT);
		usart_putchar(10,UPSNET_PORT);
		usart_putstr(eventStr,UPSNET_PORT);		// send out RS485 port
		usart_putchar(13,UPSNET_PORT);
		usart_putchar(10,UPSNET_PORT);
		#endif
		
		#if defined EVENT_REPORTING_ENABLED
			event.lastMsec = eventTime.msec;
			if (event.top >= ROUTER_EVENTS_MAX) {
				event.top = 0;
			}
			if (event.top == event.bot) {		// top has folded around and bumped into bottom, buffer is full
				if (++event.bot>= ROUTER_EVENTS_MAX) {	// start eating old events, string has already been overwritten
					event.bot = 0;
				}
			}
		#endif
	}
	//__enable_interrupt();	// TODO - interrupt enable addEvent
}

// This is used to see if there is a command in the command buffer, it simply checks to see the pointers
// aren't at the same spot.  It's also a check on the command total, if there aren't any commands it sets
// the total to zero.
int masterCmdPending(volatile struct upsDataStrucT *upsData) 
{
	volatile int answer;
	int tmptop;
        
	answer = FALSE;

//	if (upsData->masterCmdTop != upsData->masterCmdBot) 
        tmptop = upsData->masterCmdTop;
        if (tmptop != upsData->masterCmdBot)
	{
		answer = TRUE;
	}
        else 
	{
	  upsData->masterCmdTot = 0;	// Reset if there are no messages
	}

	return answer;
}

// Put string command in command buffer to be sent to a single UPS
int masterCmdAdd(volatile char *str, volatile struct upsDataStrucT *upsData)
{
	volatile int found;
	volatile upsParsePollEnum_t cmdIndex;

	upsParsePollEnum_t tmpPoll, tmpIndx;     // for eliminating IAR volatile warnings
        
	sprintf((char *)eventStrTemp, "masterCmdAdd, cmd=%s to cmdBuffer for UPS %d", str,upsData->upsNumber);
	addEvent(eventStrTemp,3);

	found = FALSE;
	if (upsData->masterCmdTot >= (MAX_MASTER_CMDS - 1)) 	// buffer full
	{
		return FALSE;										// function failed
	} 
        else 
        {
		// search commands ^RMOD, ^RLOD, etc. to find enum version
		cmdIndex = RMOD;
		while (upsParsePoll[cmdIndex] != CMD_END) 			// indicates end of list
		{
			// returns 0 when equal, just checking "^RMOD" of "^RMODIEA"
			if (strncmp((char *)str,&upsCmd[cmdIndex][0],5) == 0) 
			{
				// put label in buffer
//				upsData->masterCmd[upsData->masterCmdTop] = upsParsePoll[cmdIndex];
				tmpPoll = upsParsePoll[cmdIndex];
				upsData->masterCmd[upsData->masterCmdTop] = tmpPoll;
				// put unique string in string buffer
				strcpy((char *)upsData->masterCmdStr[upsData->masterCmdTop],(char *)str);
				found = TRUE;								// Indicate task is done for this command
				break;										// this while loop is done, no need to look further...leave
			}
			cmdIndex++;										// go to next item
		}
		if (found == FALSE) 								// prior check didn't find it, check regular commands
		{
			cmdIndex = RDAT01_PARAMS;						// first of regular commands
			while (upsParsePoll[cmdIndex] != RMOD) 			// indicates end of list for regular commands
			{
				// returns 0 when equal
				if (strcmp((char *)str,&upsCmd[cmdIndex][0]) == 0) 
				{
					// put label in buffer
//					upsData->masterCmd[upsData->masterCmdTop] = upsParsePoll[cmdIndex];
					tmpPoll = upsParsePoll[cmdIndex];
					upsData->masterCmd[upsData->masterCmdTop] = tmpPoll;
					// put regular command string in string buffer
//					strcpy((char *)upsData->masterCmdStr[upsData->masterCmdTop],&upsCmd[cmdIndex][0]);
					tmpIndx = cmdIndex;
					strcpy((char *)upsData->masterCmdStr[upsData->masterCmdTop],&upsCmd[tmpIndx][0]);
					found = TRUE;							// Indicate task is done for this command
					break;									// this while loop is done, no need to look further...leave
				}
				cmdIndex++;									// go to next item
			}
		}
		if (found == TRUE) 									// Success!  String was found, adjust pointer and total
		{
			upsData->masterCmdTop++;
			if (upsData->masterCmdTop >= MAX_MASTER_CMDS) 	// off the top
			{
				upsData->masterCmdTop = 0;					// reset it
			}												// function succeeded
			upsData->masterCmdTot++;						// increase total indicated in buffer
			return TRUE;								// indicate function succeeded to calling function
		}
		else 
		{
			return FALSE;								// indicate function failed to calling function
		}
	}
}

// Put string command in command buffer to be sent to a both UPSs
int masterCmdAddBoth(volatile char *str){

	volatile int answer;

	answer = TRUE;

	if (masterCmdAdd(str, &upsOne) == FALSE) {
		answer = FALSE;
	}
	if (masterCmdAdd(str, &upsTwo) == FALSE) {
		answer = FALSE;
	}
	return answer;
}

// Put command Enumerated label in command buffer to be sent to a single UPS
// it will look up associated string to put in string command buffer.
// Do Not Use for RMOD, RLOD, RBAT, RDIS, RALM since full command varies
int masterCmdLabelAdd(volatile upsParsePollEnum_t cmdLabel, volatile struct upsDataStrucT *upsData) {

	int tmpNumbr;
	upsParsePollEnum_t tmpLabl;
  
//	sprintf((char *)eventStrTemp, "masterCmdLabelAdd, cmd=%s to cmdBuffer for UPS %d", (char *)&upsCmd[cmdLabel],upsData->upsNumber);
	tmpNumbr = upsData->upsNumber;
	sprintf((char *)eventStrTemp, "masterCmdLabelAdd, cmd=%s to cmdBuffer for UPS %d", (char *)&upsCmd[cmdLabel],tmpNumbr);
	addEvent(eventStrTemp,3);

	if (upsData->masterCmdTot >= (MAX_MASTER_CMDS - 1)) 	// buffer full
	{
		return FALSE;										// function failed
	} 
	else 
	{

//		upsData->masterCmd[upsData->masterCmdTop] = cmdLabel;
		tmpLabl = cmdLabel;
		upsData->masterCmd[upsData->masterCmdTop] = tmpLabl;
//		strcpy((char *)upsData->masterCmdStr[upsData->masterCmdTop],&upsCmd[0][cmdLabel]);
		strcpy((char *)upsData->masterCmdStr[upsData->masterCmdTop],&upsCmd[0][tmpLabl]);
		upsData->masterCmdTop++;
		if (upsData->masterCmdTop >= MAX_MASTER_CMDS) 	// off the top
		{
			upsData->masterCmdTop = 0;					// reset it
		}												// function succeeded
		upsData->masterCmdTot++;						// increase total indicated in buffer
		return TRUE;								// indicate function succeeded to calling function
	}
}

// Put command Enumerated label in command buffer to be sent to both UPSs
// it will look up associated string to put in string command buffer.
// Do Not Use for RMOD, RLOD, RBAT, RDIS, RALM since full command varies
int masterCmdLabelAddBoth(volatile upsParsePollEnum_t cmdLabel){

	volatile int answer;

	answer = TRUE;
	if (masterCmdLabelAdd(cmdLabel, &upsOne) == FALSE) {
		answer = FALSE;
	}
	if (masterCmdLabelAdd(cmdLabel, &upsTwo) == FALSE) {
		answer = FALSE;
	}
	return answer;
}

int masterCmdSend(volatile struct upsDataStrucT *upsData)
{

	int tmptop;
	portName_t tmpport;
        
//	if (upsData->masterCmdTop == upsData->masterCmdBot) {	// buffer empty
	tmptop = upsData->masterCmdTop;
	if (tmptop == upsData->masterCmdBot)
	{
		upsData->masterCmdTot = 0;						// reset total
		return FALSE;								// function failed
	} 
	else 
	{
		upsData->currentCmd = upsData->masterCmd[upsData->masterCmdBot];
//		usart_putstr(upsData->masterCmdStr[upsData->masterCmdBot++], upsData->port);
		tmpport = upsData->port;
		usart_putstr(upsData->masterCmdStr[upsData->masterCmdBot++], tmpport);                
		if (upsData->masterCmdBot >= MAX_MASTER_CMDS) 		// off the top
		{
			upsData->masterCmdBot = 0;					// reset it
		}									// function succeeded
		if (--upsData->masterCmdTot < 0) 					// decrement before checking for neg number
		{
			upsData->masterCmdTot = 0;					// reset to zero
		}
		return TRUE;
	}
}

/*
This is used to find which command is being sent
*/

void updateAlm(volatile alm_states_t state) {
	//	"^RALMSNNN"		S= on solid, F=flash, N=off
	//		  |->		Sonalert
	//		   |->		Aux LED1
	//		    |->		Aux LED2
	//		     |->	Blank all LEDs

	switch(state) {
	case SONALERT_ON:
		upsOne.almMask[5] = 'S';
		break;
	case SONALERT_OFF:
		upsOne.almMask[5] = 'N';
		upsTwo.almMask[5] = 'N';
		break;
	case SONALERT_BEEP:
		upsOne.almMask[5] = 'F';
		break;
	case BLANK_ALL:
		upsOne.almMask[8] = 'N';
		upsTwo.almMask[8] = 'N';
		break;
	case UNBLANK_ALL:
		upsOne.almMask[8] = 'S';
		upsTwo.almMask[8] = 'S';
		break;
	case TEST_LED:
		upsOne.almMask[8] = 'T';
		upsTwo.almMask[8] = 'T';
		break;
	case UPDATE:					// This routine is called just to update the UPS alarm state
	default:
		break;
	}
	strcpy((char *)almStrOne,(char *)upsOne.almMask);
	strcpy((char *)almStrTwo,(char *)upsTwo.almMask);
}

void updateLoadDisplay(void) {

	//	volatile char loadDisp[10];

	#if defined THALES_3KVA
		if ( (upsBoss.dcMode == AUTO_ON) || (upsBoss.dcMode == MANUAL_ON) ) {
			upsBoss.upsState = UPS_ON_BAT;
		} else if ( (upsBoss.invMode == AUTO_ON) || (upsBoss.invMode == MANUAL_ON) ){
			upsBoss.upsState = UPS_ON_UTIL;
		} else {
			upsBoss.upsState = UPS_OFF;
		}
	#endif
	if ( (upsBoss.upsState == UPS_ON_BAT) || (upsBoss.upsState == UPS_ON_UTIL) ) {
		if (upsBoss.loadPctOut < 10.0) {
			strcpy((char *)loadDisp,"^RLOD00S");
		} else {
			if (upsBoss.loadPctOut < 25.0) {
				strcpy((char *)loadDisp,"^RLOD01S");
			} else {
				if (upsBoss.loadPctOut < 50.0) {
					strcpy((char *)loadDisp,"^RLOD02S");
				} else {
					if (upsBoss.loadPctOut < 75.0) {
						strcpy((char *)loadDisp,"^RLOD03S");
					} else {
						if (upsBoss.loadPctOut < 100.0) {
							strcpy((char *)loadDisp,"^RLOD04S");
						} else {
							if (upsBoss.loadPctOut < 105.0) {
								strcpy((char *)loadDisp,"^RLOD05S");
							} else {
								strcpy((char *)loadDisp,"^RLOD05F");
							}
						}
					}
				}
			}
		}
		if (inverterControl() == WARN) {
			loadDisp[7] = 'F';				// if high temp or other problem (besides over power), flash bar
		} else if (inverterControl() == AUTO_ON) {
			loadDisp[7] = 'S';				// if okay, solid bar
		}
	}
	else {								// inverter not running
		strcpy((char *)loadDisp,"^RLOD00S");	// so blank load bar (filtering causes load indication to decrease over time
	}
	//	masterCmdAddBoth(loadDisp);			// update both displays with command
}

void updateBatDisplay(void) {

	strcpy((char *)upsBoss.StrTemp,"^RBAT");	// command prefix

	if (upsBoss.batChgPct < 6.0) {
		strcat((char *)upsBoss.StrTemp,"01");	// higher led for higher capacity
	} else {
		if (upsBoss.batChgPct < 25.0) {
			strcat((char *)upsBoss.StrTemp,"02");
		} else {
			if (upsBoss.batChgPct < 50.0) {
				strcat((char *)upsBoss.StrTemp,"03");
			} else {
				#if !defined THALES_3KVA
					if ( (upsBoss.batChgPct >= 95.0) && (upsOne.chgMode == AUTO_SLOW) && (upsTwo.chgMode == AUTO_SLOW) ) {
						strcat((char *)upsBoss.StrTemp,"05");
					} else {	// this will only happen after going to float charge
						strcat((char *)upsBoss.StrTemp,"04");
					}
				#else // defined THALES_3KVA
					if ( (upsBoss.batChgPct >= 95.0) && (upsBoss.chgMode == AUTO_SLOW) ) {
						strcat((char *)upsBoss.StrTemp,"05");
					} else {	// this will only happen after going to float charge
						strcat((char *)upsBoss.StrTemp,"04");
					}
				#endif
			}
		}
	}
	if (upsBoss.upsState == UPS_ON_BAT) {
		strcat((char *)upsBoss.StrTemp,"F");	// Flash bar
	} else {
		strcat((char *)upsBoss.StrTemp,"S");	// Steady on
	}
	strcpy((char *)batDisp,(char *)upsBoss.StrTemp);
	//	masterCmdAddBoth(upsBoss.StrTemp);	// put out command
}

void updateLedDisplay(void) 
{

	//		masterCmdAddBoth("^RDISSNNN");
	//							    |->		On LED
	//								 |->	Bypass LED
	//							      |->	Service Battery LED
	//								   |->	Fault LED
	// On LED
	#if !defined THALES_3KVA
		#if (defined CANES_FA10249)
			if (upsBoss.invOverloadTrip == FAULT)
			{
				strcpy((char *)upsBoss.StrTemp,"^RDISN");
			}
			else if (upsBoss.invMode == AUTO_ON)
			{
				strcpy((char *)upsBoss.StrTemp,"^RDISS");
			}
			else 
			{
				strcpy((char *)upsBoss.StrTemp,"^RDISN");
			}
		#elif (DUAL_BOARD == TRUE)
			if (upsBoss.invMode == AUTO_ON) 
			{
				strcpy((char *)upsBoss.StrTemp,"^RDISS");
			} 
			else 
			{
				strcpy((char *)upsBoss.StrTemp,"^RDISN");
			}
		#else
			if (upsOne.invMode == AUTO_ON) 
			{
				strcpy((char *)upsBoss.StrTemp,"^RDISS");
			} 
			else 
			{
				strcpy((char *)upsBoss.StrTemp,"^RDISN");
			}
		#endif
	#else // defined THALES_3KVA
		if (upsBoss.invMode == AUTO_ON) {
			strcpy((char *)upsBoss.StrTemp,"^RDISS");
		} else {
			strcpy((char *)upsBoss.StrTemp,"^RDISN");
		}
	#endif
	// Bypass LED
	#if (!defined CANES_FA10249)
        if (upsBoss.bypassMode == ON)               // ON, OFF or AUTO
        {
            strcat((char *)upsBoss.StrTemp,"S");
        } else {
            strcat((char *)upsBoss.StrTemp,"N");
        }
    #else                                           // defined CANES_FA10249
	    if (upsBoss.invOverloadTrip == FAULT)
	    {
            strcat((char *)upsBoss.StrTemp,"N");
	    }
	    else if (upsBoss.bypassMode == ON)          // ON, OFF or AUTO
	    {
            
            strcat((char *)upsBoss.StrTemp,"S");
        } 
        else 
        {
            strcat((char *)upsBoss.StrTemp,"N");
        }
	#endif
	// Service Battery LED
	switch (upsBoss.batCond) {
	case NORMAL:
		strcat((char *)upsBoss.StrTemp,"N");
		break;
	case WARN:
		strcat((char *)upsBoss.StrTemp,"F");
		break;
	case OVER_VOLTAGE:
	case FAULT:
		strcat((char *)upsBoss.StrTemp,"S");
		break;
	}
	// Fault LED
	#if !defined THALES_3KVA
		if ( (upsBoss.upsState == UPS_FAULT) || (upsBoss.upsState == UPS_COM_SHUTDOWN) ) {
			strcat((char *)upsBoss.StrTemp,"S");
		} else {
			strcat((char *)upsBoss.StrTemp,"N");
		}
	#else // defined THALES_3KVA
		if (upsBoss.upsState == UPS_FAULT) {
			strcat((char *)upsBoss.StrTemp,"S");
		} else {
			strcat((char *)upsBoss.StrTemp,"N");
		}
	#endif
	strcpy((char *)ledDisp,(char *)upsBoss.StrTemp);
	//	masterCmdAddBoth(upsBoss.StrTemp);
}

void initUpdateDisplay(void) {

	strcpy((char *)loadDisp,NULL);
	strcpy((char *)loadDispLast,NULL);
	strcpy((char *)ledDisp,NULL);
	strcpy((char *)ledDispLast,NULL);
	strcpy((char *)batDisp,NULL);
	strcpy((char *)batDispLast,NULL);
	strcpy((char *)almStrOne,NULL);
	strcpy((char *)almStrOneLast,NULL);
	strcpy((char *)almStrTwo,NULL);
	strcpy((char *)almStrTwoLast,NULL);
	updateAlm(UPDATE);			// initialize almStrX
}

// The alarm string is set by updateAlm(), this function checks to see if it has changed or 5 seconds
// have passed to send the string to both UPS
void refreshAlarmString (void) {
	
	static volatile struct timeT timeAlm;

	if ( (strcmp((char *)almStrOne,(char *)almStrOneLast)) || (strcmp((char *)almStrTwo,(char *)almStrTwoLast)) ) { // non-zero response for different strings
		strcpy((char *)almStrOneLast,(char *)almStrOne);	// update history to see if it changes for next time
		strcpy((char *)almStrTwoLast,(char *)almStrTwo);
		masterCmdAdd(almStrOne, &upsOne); 		// Update UPS, string may be different between the two
		masterCmdAdd(almStrTwo, &upsTwo);
		timeAlm = getTime();					// reset timer
	} else {
		if (timer(timeAlm, 5000)) {			// if no updates, refresh display
			timeAlm = getTime();				// reset timer
			masterCmdAdd(almStrOne, &upsOne); 	// Update UPS, string may be different between the two
			masterCmdAdd(almStrTwo, &upsTwo);
		}
	}

}

// This checks for changes in display information and immediately sends new info to UPS for display
// as a backup it will refresh the display in each part every few seconds until there is new info
void updateDisplayRefresh(void) {

	static volatile struct timeT timeLoadDisp, timeLedDisp, timeBatDisp;

	//__disable_interrupt();                    // TODO - interrupt disable updateDisplay

	refreshAlarmString();						// this updates alarm string to UPS for Sonalert and some LEDs
	updateLoadDisplay();
	if (strcmp((char *)loadDisp,(char *)loadDispLast)) { // non-zero response for different strings
		strcpy((char *)loadDispLast,(char *)loadDisp);	// update history to see if it changes for next time
		masterCmdAddBoth(loadDisp);			    // update both displays with command
		timeLoadDisp = getTime();				// reset timer
	} else {
		if (timer(timeLoadDisp, 5000)) {		// if no updates, refresh display
			timeLoadDisp = getTime();			// reset timer
			masterCmdAddBoth(loadDisp);		    // update both displays with command
		}
	}
	updateLedDisplay();
	if (strcmp((char *)ledDisp,(char *)ledDispLast)) { // non-zero response for different strings
		strcpy((char *)ledDispLast,(char *)ledDisp);	// update history to see if it changes for next time
		masterCmdAddBoth(ledDisp);				// update both displays with command
		timeLedDisp = getTime();				// reset timer
	} else {
		if (timer(timeLedDisp, 5000)) {		    // if no updates, refresh display
			masterCmdAddBoth(ledDisp);			// update both displays with command
			timeLedDisp = getTime();			// reset timer
		}
	}
	updateBatDisplay();
	if (strcmp((char *)batDisp,(char *)batDispLast)) { // non-zero response for different strings
		strcpy((char *)batDispLast,(char *)batDisp);	// update history to see if it changes for next time
		masterCmdAddBoth(batDisp);				// update both displays with command
		timeBatDisp = getTime();				// reset timer
	} else {
		if (timer(timeBatDisp, 5000)) {		// if no updates, refresh display
			masterCmdAddBoth(batDisp);			// update both displays with command
			timeBatDisp = getTime();			// reset timer
		}
	}
	//__enable_interrupt();	// TODO - interrupt enable updateDisplay
}

// This will switch the display between normal display and LED test display
void updateDisplay(void) 
{

	static volatile struct timeT lightshowTime;
	int tmpFlag;

	// Test button has been pressed
	if (lightshowFlag == TRUE) 					// Test requested
	{
	    //if (lightshowFlag != lightshowLast) {	// first time after request
		tmpFlag = lightshowFlag;
		if (tmpFlag != lightshowLast)
		{
		    //lightshowLast = lightshowFlag;		// update flag
			lightshowLast = tmpFlag;
			lightshowTime = getTime();			// remember start time
			masterCmdAddBoth("^RDAT30");		// Run Test "Light Show"
		}
		if (timer(lightshowTime, 10000)) 		// Timeout, in case UPS "Lightshow Done" message garbled
		{
			lightshowFlag = lightshowLast = FALSE;  // not sure why IAR didn't flag this -- multiple volatiles
		}
	}
	else 	// run normal display routine
	{
		updateDisplayRefresh();
	}
}

operatingModesT selectStrOpMode(volatile char *str){

	volatile int i;
	volatile operatingModesT result;

	result = NULL_MODE;						// mode not found

	for (i=0;i<OP_MODE_STR_MAX;i++) {
		if(strcmp((char *)str,opModeStr[i]) == 0){	// returns 0 when equal
			result = (operatingModesT) i;	// if same string, return index of match
		}
	}
	return result;
}

// Used in upsCom to look at front of response string to see what command it is responding to
// checks against all commands and returns the index so the correct processing can occur
// input: UPS response string
// output: index into command enumerated values matching response header

upsParsePollEnum_t scanResponseHeader(volatile char *str) {
	volatile upsParsePollEnum_t cmdIndex,commandFoundIndex;
	//	volatile char tempStr[150],tempCmd[15];
	//	volatile int tempCmdLen=0;

	//	strcpy(tempStr,str);

	commandFoundIndex = CMD_END;							// CMD_END is last enumerated value = not found, if found then 0-CMD_END
	cmdIndex = RDAT01_PARAMS;								// start a beginning of list
	while (upsParsePoll[cmdIndex] < CMD_END) {				// indicates end of list
		//		strcpy(tempCmd,&upsCmd[cmdIndex][0]);
		//		strlen(&upsCmd[cmdIndex][0]);
		if (cmdIndex < RMOD) {								// Check full command
			if (strncmp((char *)str,&upsCmd[cmdIndex][0],7) == 0) {	// returns 0 when equal
				commandFoundIndex = cmdIndex;							// CMD_END is last enumerated value = not found, if found then 0-CMD_END
				break;										// this while loop is done, no need to look further...leave
			}
		}
		else {												// just checking "^RMOD" of "^RMODIEA"
			if (strncmp((char *)str,&upsCmd[cmdIndex][0],5) == 0) {	// returns 0 when equal
				commandFoundIndex = cmdIndex;							// CMD_END is last enumerated value = not found, if found then 0-CMD_END
				break;										// this while loop is done, no need to look further...leave
			}
		}
		cmdIndex++;											// go to next item
	}
	if (commandFoundIndex != CMD_END) {							// match found
		sprintf((char *)eventStrTemp, "scanResponseHeader, msg=[%s], cmd=[%s]", str,upsCmd[commandFoundIndex]);
		addEvent(eventStrTemp,4);
	}
	else {
		sprintf((char *)eventStrTemp, "scanResponseHeader failed to find a match for msg=[%s]", (char *)str);
		addEvent(eventStrTemp,4);
	}
	return commandFoundIndex;
}


// take string from UPS BAT_TIME format "150:00" and parse it into seconds
long scanEstSecBat(volatile char *str) 
{

	volatile int pIn = 0, pOut = 0;
	volatile long sec = 0;
	volatile char strOut[10];

	char tmpchar;                   // IAR compiler issue

	pIn = pOut = 0;								// since the compiler may use registers for local
	sec = 0;									// variables, make sure they are initialized, the 
	strOut[0] = NULL;							// register may have non-zero values from prior use
	
	do {
		if (str[pIn] == ':')                                            // minutes
		{					
			sec = (long) (atol((char *)strOut) * 60);
			strOut[0] = NULL;					// reset string
			pOut = 0;
		}
		else 
		{       
		    //strOut[pOut] = str[pIn];			// take char from input string to output
			tmpchar = str[pIn];
			strOut[pOut] = tmpchar;
			pOut++;							// bump pointer
			strOut[pOut] = NULL;			// always end with null
			if (str[pIn + 1] == NULL) 		// next char end of input?
			{
			  sec += (long) atol((char *)strOut);   // if yes, finish by add remaining seconds
			}
		}
		pIn++;								// next input char
	} while ((str[pIn] != NULL) && ((unsigned) pIn < strlen((char *)str)));	// leave if done
	return(sec);
}

int scanParams(volatile struct upsDataStrucT *upsData) {

	volatile int msgPos, paramPos, paramNum, msgUnderstood = FALSE;
	volatile char msgChar;
        
	int tmpNum;
	char tmpChar;

	upsData->lastParam = -1;							// if no params found this will be untouched

	paramPos = paramNum = 0;							// reset pointers

	if (strncmp("^D",(char *)upsData->msgStr,2) == 0) {		// see if response to SNMP request
		msgPos = 5;										// start with first char after "^D000"
		if (strlen((char *)upsData->msgStr) > 5) {
			upsData->lastParam = 0;					// something is there besides preamble
		}
	} else {
		msgPos = 0;										// computer protocol, start with first char
		if (strlen((char *)upsData->msgStr) > 0) {
			upsData->lastParam = 0;					// something is there
		}
	}

	if (strncmp("^0",(char *)upsData->msgStr,2) != 0) {		// returns 0 if same, so a non-zero means the ups understood command

		msgUnderstood = TRUE;
		// run until end of message buffer or finding a NULL char
		while ( (upsData->msgStr[msgPos] != NULL) && (msgPos < (TEMP_BUFFER_LENGTH-2)) ){
			msgChar = upsData->msgStr[msgPos++];		// get character
			if (msgChar == ',') {						// comma delimited list
				if (paramNum < PARAM_NUM_MAX) {		// don't overrun max number of parameters, if so keep using last index
					upsData->lastParam = ++paramNum;	// update data structure
				}
				paramPos = 0;							// reset char pointer for next param
			}
			else {
				if (paramPos < (PARAM_LEN_MAX-2))		// check for overrun, leave room for next char and null
				{		                       
				    //StrParams[paramNum][paramPos++] = msgChar;
				    tmpNum = paramNum;                            //IAR fix
				    tmpChar = msgChar;
				    StrParams[tmpNum][paramPos++] = tmpChar;           
				    // Null will move until last char put into string insuring string terminated with null
				    //StrParams[paramNum][paramPos] = NULL;
				    StrParams[tmpNum][paramPos] = NULL;
				}
				else 
				{
				    // empty else clause
				}
			}
		}
	}
	if ((msgPos < (TEMP_BUFFER_LENGTH-2)) && (msgUnderstood == TRUE)) {
		return (TRUE);
	} else {
		return (FALSE);
	}
}

// special version of scanParams for SNMP, needed for new SNMP commands with multiple parameters, extended snmpDataStruct
void scanSnmpParams(volatile struct snmpDataStruct *parseType) {

	int msgPos, paramPos, paramNum;
	char msgChar;

	parseType->lastParam = -1;						// if no params found this will be untouched

	paramPos = paramNum = 0;						// reset pointers

	msgPos = 3;										// start with first char after "OTC"
	if (strlen((char *) parseType->aData) > 3) {
		parseType->lastParam = 0;					// something is there besides preamble
	}

	StrParams[0][0] = NULL;				// clear param0 by putting NULL in char 0

	// run until end of message buffer or finding a NULL char
	while ( (parseType->aData[msgPos] != NULL) && (msgPos < (SNMP_STR_MAX-3)) ){
		msgChar = parseType->aData[msgPos++];		// get character
		if (msgChar == ',') {						// comma delimited list
			if (paramNum < PARAM_NUM_MAX) {		// don't overrun max number of parameters, if so keep using last index
				parseType->lastParam = ++paramNum;	// update data structure
			}
			paramPos = 0;							// reset char pointer for next param
			// clean out previous contents, may skip right to next param if next char is a ','
			StrParams[paramNum][paramPos] = NULL;
		} else {
			if (paramPos < (PARAM_LEN_MAX-2)) {		// check for overrun, leave room for next char and null
				StrParams[paramNum][paramPos++] = msgChar;
				// Null will move until last char put into string insuring string terminated with null
				StrParams[paramNum][paramPos] = NULL;
			}
		}
	}
}

void tab(volatile char *str, volatile int tabTo) 
{
//	volatile int pad, lastChar;             // IAR fix
	int pad, lastChar;                      // These variables are local and they aren't going to chg unless we changem
    										// Adding temp variables that aren't volatile would be overkill
	lastChar = strlen((char *)str);
	pad = tabTo - lastChar;					// How many spaces to add
	if (pad > 0) {							// only add, if needed
		for(; lastChar > pad; lastChar++)   // variable already set to start
		{
			str[lastChar] = ' ';			// space
			str[lastChar+1] = NULL;			// string termination
		}
	}
}

void rs485tabularData(void) 
{
	volatile char rs485msg[100], stemp[10];
	volatile struct upsDataStrucT *ups;
	static volatile struct timeT msgTimer;
	volatile long ltemp1, ltemp2;
	volatile int itemp;
	static volatile int i;
        
	float tmpfval1,tmpfval2,tmpfval3,tmpfval4;       // IAR compiler "fix"  
	char *tmpMod1, *tmpMod2, *tmpMod3, *tmpMod4;
	long tmplval2;

	ups = pRS485upsSelected;
											// not finished, need to set a flag and call this from 
											// rs485com()
	// Since this will output enough characters to overload the transmit buffer and RAM is at a premium
	// this will sent the information in sections with waits between each transmission to spool it out
	// to the port.
	msgTimer = getTime();
	for(i=0;i<=3;i++) {
		__no_operation();
		switch(i) {
		case 0:
			usart_putstr(upsSelectedTitle,UPSNET_PORT);
			//sprintf((char *)rs485msg,"\r\nOutput: %1.1f%%, %1.1f Watts, %1.1f VA, ",
			//    ups->loadPctOut, ups->powOut, ups->vaOut);
			tmpfval1 = ups->loadPctOut;
			tmpfval2 = ups->powOut;
			tmpfval3 = ups->vaOut;
			sprintf((char *)rs485msg,"\r\nOutput: %1.1f%%, %1.1f Watts, %1.1f VA, ",
			tmpfval1, tmpfval2, tmpfval3);
			usart_putstr(rs485msg,UPSNET_PORT);
			//sprintf((char *)rs485msg,"%1.1f Volts, %1.2f Amps, %1.1f Hz, %1.2fPF",
			//    ups->voltOut, ups->ampOut, ups->freqOut, ups->pfOut);
			tmpfval1 = ups->voltOut;
			tmpfval2 = ups->ampOut;
			tmpfval3 = ups->freqOut; 
			tmpfval4 = ups->pfOut;
			sprintf((char *)rs485msg,"%1.1f Volts, %1.2f Amps, %1.1f Hz, %1.2fPF",
					tmpfval1, tmpfval2, tmpfval3, tmpfval4);
			usart_putstr(rs485msg,UPSNET_PORT);
			// used to pace out spooling so buffer doesn't overrun
			while (!timer(msgTimer,100));
			msgTimer = getTime();
			break;
		case 1:
		    //sprintf((char *)rs485msg,"\r\nInput: %1.1f Volts, %1.1f Hz",
		    //    ups->voltIn, ups->freqIn);
			tmpfval1 = ups->voltIn;
			tmpfval2 = ups->freqIn;
			sprintf((char *)rs485msg,"\r\nInput: %1.1f Volts, %1.1f Hz",
			tmpfval1, tmpfval2);                        
			usart_putstr(rs485msg,UPSNET_PORT);
			//sprintf((char *)rs485msg,"\r\nModes: Inverter=%s, DC/DC=%s, Charger=%s, Bypass=%s",
			//    opModeStr[ups->invMode], opModeStr[ups->dcMode], opModeStr[ups->chgMode],
			//    opModeStr[ups->bypassMode]);
			tmpMod1 = opModeStr[ups->invMode]; 
			tmpMod2 = opModeStr[ups->dcMode]; 
			tmpMod3 = opModeStr[ups->chgMode];
			tmpMod4 = opModeStr[ups->bypassMode];                        
			sprintf((char *)rs485msg,"\r\nModes: Inverter=%s, DC/DC=%s, Charger=%s, Bypass=%s",
					tmpMod1, tmpMod2, tmpMod3, tmpMod4);                        
			usart_putstr(rs485msg,UPSNET_PORT);
			// used to pace out spooling so buffer doesn't overrun
			while (!timer(msgTimer,100));
			msgTimer = getTime();
			break;
		case 2:
		    //sprintf((char *)rs485msg,"\r\nBattery: %1.2f Volts, %1.1f Discharge Amps, %1.1f Charge Amps, %1.1f",
		    //    ups->voltBat, ups->ampBat, ups->ampChg, ups->batChgPct);		// \x25 hex ascii for %, doesn't work
			tmpfval1 = ups->voltBat;
			tmpfval2 = ups->ampBat;
			tmpfval3 = ups->ampChg; 
			tmpfval4 = ups->batChgPct;
			sprintf((char *)rs485msg,"\r\nBattery: %1.2f Volts, %1.1f Discharge Amps, %1.1f Charge Amps, %1.1f",
			tmpfval1, tmpfval2, tmpfval3, tmpfval4);		
			strcat((char *) rs485msg,"% Charge");
			usart_putstr(rs485msg,UPSNET_PORT);
			strcpy((char *)rs485msg,"\r\nBattery Run Time Remaining: ");
			ltemp1 = ups->estSecBat;
			ltemp2 = ltemp1/60;

			//ltoa(ltemp2, (char *) stemp);
			sprintf( (char *) stemp, "%ld", ltemp2);

			strcat((char *) rs485msg,(char *) stemp);
			strcat((char *) rs485msg,":");
			//ltemp1 -= (ltemp2*60);
			tmplval2 = ltemp2;
			ltemp1 = ltemp1 - (tmplval2*60);
			if (ltemp1 < 10) 					// want two digits for seconds if 1 number pad with leading zero
			{
                           strcat((char *) rs485msg,"0");
			}

			//ltoa(ltemp1, (char *) stemp);
			sprintf( (char *) stemp, "%ld", ltemp1);

			strcat((char *) rs485msg,(char *) stemp);
			strcat((char *) rs485msg," Min:Sec, ");

			//ltoa(ups->batJoule, (char *) stemp);
			sprintf( (char *) stemp, "%ld", ups->batJoule);

			strcat((char *) rs485msg,(char *) stemp);
			strcat((char *) rs485msg,"/");
			#if DUAL_BOARD == TRUE							// two battery strings, double number
				if (ups->upsNumber == 0) 						// upsBoss=0, upsOne=1, upsTwo=2
				{			
				    //ltoa(BAT_MAX_JOULE * 2, (char *) stemp);
					sprintf( (char *) stemp, "%ld", BAT_MAX_JOULE * 2);
				} 
				else 
				{

				    //ltoa(BAT_MAX_JOULE, (char *) stemp);
					sprintf( (char *) stemp, "%ld", BAT_MAX_JOULE);
				}
			#else

			    //ltoa(BAT_MAX_JOULE, (char *) stemp);
				sprintf( (char *) stemp, "%ld", BAT_MAX_JOULE);

			#endif
			strcat((char *) rs485msg,(char *) stemp);
			strcat((char *) rs485msg," Actual/Max Joules");
			usart_putstr(rs485msg,UPSNET_PORT);
			// used to pace out spooling so buffer doesn't overrun
			while (!timer(msgTimer,100));
			msgTimer = getTime();
			break;
		case 3:
		    //sprintf((char *)rs485msg,"\r\nTemperature: %1.0fF\x2f%1.0fC Ambient, %1.0fF\x2f%1.0fC Heatsink",
		    //    (ups->tAmb*1.8)+32, ups->tAmb, (ups->tAmb*1.8)+32, ups->tSink);		// \x2f = '/'
			tmpfval1 = (ups->tAmb*1.8)+32;
			tmpfval2 = ups->tAmb;
			tmpfval3 = (ups->tAmb*1.8)+32; 
			tmpfval4 = ups->tSink;                  
			sprintf((char *)rs485msg,"\r\nTemperature: %1.0fF\x2f%1.0fC Ambient, %1.0fF\x2f%1.0fC Heatsink",
					tmpfval1, tmpfval2, tmpfval3, tmpfval4);				// \x2f = '/'
			usart_putstr(rs485msg,UPSNET_PORT);
			//sprintf((char *)rs485msg,"\r\nMisc: %1.1fVDC Bus, %1.1fVDC PS, %1.3fADC PS",
			//    ups->voltBus, ups->voltSupply, ups->ampSupply);
			tmpfval1 = ups->voltBus;
			tmpfval2 = ups->voltSupply;
			tmpfval3 = ups->ampSupply;
			sprintf((char *)rs485msg,"\r\nMisc: %1.1fVDC Bus, %1.1fVDC PS, %1.3fADC PS",
					tmpfval1, tmpfval2, tmpfval3);                      
			usart_putstr(rs485msg,UPSNET_PORT);
			sprintf((char *)rs485msg,"\r\nSoftware Version: %s \r\n\r\n", (char *)ups->verSoftware);
			usart_putstr(rs485msg,UPSNET_PORT);
			// used to pace out spooling so buffer doesn't overrun
			while (!timer(msgTimer,100));
			msgTimer = getTime();
			break;
		}
	}
}

void initRs485com(void) {
	strcpy((char *)rs485cmd,NULL);			// initialize string
	//rs485cmdEnd = 0;
	pRS485upsSelected = &upsBoss;
	strcpy((char *)upsSelectedTitle,"\r5KVA Virtual UPS");
}

void rs485com(void) {
    //volatile char newChar;
    char newChar;                           // IAR compiler fix - once the new char is set, it doesn't chg
    #if ((DUAL_BOARD == TRUE) && (BAT_CAP_METHOD == BAT_CAP_JOULE))
        long tmp1joule,tmp2joule;           //   within this subroutine and since its local, should be OK
    #endif
    volatile float fTemp;
	static volatile int rs485cmdEnd = 0;
	#if BAT_CAP_METHOD	== BAT_CAP_JOULE	    // prevent compiler warning if not used
		volatile char *pString;					// string pointer to pass elements of string array
	#endif


	if (usart_rx_buffer_count(UPSNET_PORT)) {    // anything coming in?
		newChar = usart_getchar(UPSNET_PORT);    // get top char in buffer
		rs485cmd[rs485cmdEnd++] = newChar;        // add char and bump pointer
		usart_putchar(newChar,UPSNET_PORT);		// echo back
		if (rs485cmdEnd == 1) {					// not continuation of multichar command
			switch(newChar) {
			case '?':								// tabular display
				rs485tabularData();
				rs485cmdEnd = 0;					// reset for next single character command
				break;
			case 'h': case 'H':			// Help display
				usart_putstr("\r\nHelp",UPSNET_PORT);
				usart_putstr("\r\nPress 1, 2 or 3 for 5KVA, Master or Slave UPS information",UPSNET_PORT);
				usart_putstr("\r\nPress ? for data",UPSNET_PORT);
				usart_putstr("\r\nPress s or S to start without timeout",UPSNET_PORT);
				#if BAT_CAP_METHOD	== BAT_CAP_JOULE
				usart_putstr("\r\nPress '.' to set joule level of batteries",UPSNET_PORT);
				#endif
				usart_putstr("\r\n",UPSNET_PORT);
				rs485cmdEnd = 0;					// reset for next single character command
				break;
			case '1':					// Select UPS to interrogate from RS485
				pRS485upsSelected = &upsBoss;
				strcpy((char *)upsSelectedTitle,"\r\n5KVA Virtual UPS");
				usart_putstr("\r\nYou have selected 5KVA Virtual UPS\r\n",UPSNET_PORT);
				rs485cmdEnd = 0;					// reset for next single character command
				break;
			case '2':					// Select Master UPS to interrogate from RS485
				pRS485upsSelected = &upsOne;
				strcpy((char *)upsSelectedTitle,"\rMaster Sub-UPS");
				usart_putstr("\r\nYou have selected Master Sub-UPS\r\n",UPSNET_PORT);
				rs485cmdEnd = 0;					// reset for next single character command
				break;
			case '3':					// Select Slave UPS to interrogate from RS485
				pRS485upsSelected = &upsTwo;
				strcpy((char *)upsSelectedTitle,"\rSlave Sub-UPS");
				usart_putstr("\rYou have selected Slave Sub-UPS\r",UPSNET_PORT);
				rs485cmdEnd = 0;					// reset for next single character command
				break;
			case '4':					// If any other number selected
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			case '0':
				pRS485upsSelected = &upsBoss;
				strcpy((char *)upsSelectedTitle,"\r5KVA Virtual UPS");
				usart_putstr("\r\nYou have selected 5KVA Virtual UPS\r\n",UPSNET_PORT);
				rs485cmdEnd = 0;					// reset for next single character command
				break;
			case '.':									// set joules by entering percent
				usart_putstr("Enter Joule Capacity in Percent\r\n", UPSNET_PORT);
				break;
			case 'S': case 's':				// Start UPS bypassing turn on timeout
				usart_putstr("\r\nTurning on UPS\r\n",UPSNET_PORT);
				upsStateRS485turnon = TRUE;
				rs485cmdEnd = 0;					// reset for next single character command
				break;
			default:
				rs485cmd[rs485cmdEnd++] = newChar;	// add char and bump pointer
				rs485cmd[rs485cmdEnd] = NULL;		// terminate with null for string function
				break;
			}
			usart_putstr("\r\n",UPSNET_PORT);
		} else {						// not first char, check for multi char entry
			if ((newChar=='\r') || (newChar=='\n')) {	// carriage return or newline, end of command
				rs485cmdEnd = 0;
				switch (rs485cmd[0]) {					// what single character command is being processed?
				#if BAT_CAP_METHOD == BAT_CAP_JOULE
				case '.':								// Test Parser: set joules by entering percent
					pString = &rs485cmd[1];
					fTemp = atof((char *) pString);
					fTemp = fMax(fTemp,0.0);			// minimum 0%
					fTemp = fMin(fTemp,100.0);			// maximum 100%
					#if DUAL_BOARD == TRUE
						upsOne.batJoule = (long) ((fTemp / 100.0) * BAT_MAX_JOULE);
						upsTwo.batJoule = (long) ((fTemp / 100.0) * BAT_MAX_JOULE);
						//upsBoss.batJoule = upsOne.batJoule + upsTwo.batJoule;
						tmp1joule = upsOne.batJoule;
						tmp2joule = upsTwo.batJoule;                                                
						upsBoss.batJoule = tmp1joule + tmp2joule;
					#else
						upsBoss.batJoule = (long) ((fTemp / 100.0) * BAT_MAX_JOULE);
						#ifdef  THALES_3KVA				// calc done on upsOne
							upsOne.batJoule = upsBoss.batJoule;
						#endif
					#endif
					rs485cmdEnd = 0;					// reset for next command
					break;
				#endif
				default:
					rs485cmdEnd = 0;					// reset for next command
					break;
				}
			}
		}
	}
	if (rs485cmdEnd >= (RS485_CMD_LEN-1)) {		// at end of command string
		rs485cmdEnd = 0;						// reset
		rs485cmd[0] = NULL;
	}
	//	char rs485cmd[RS485_CMD_LEN];

	// void usart_putstr(char *buffer,UPSNET_PORT){

}

void init_ups_com(void)	// TODO - Add virtual UPS Nominal values in initiation
{
	upsBoss.upsNumber = 0;
	upsOne.upsNumber = 1;
	upsTwo.upsNumber = 2;
	upsOne.port = UPS1_PORT;
	upsTwo.port = UPS2_PORT;
	strcpy((char *)upsBoss.upsId,"Unknown");					// in snmp com it responds with "^D00200"
	strcpy((char *)upsBoss.model,UPS_MODEL);
	strcpy((char *)upsBoss.man,UPS_MANUFACTURER);
	//strcpy((char *)upsBoss.verSoftware,DESCRIPTION);			// String representing config description, set at beginning of main.c
	strcpy((char *)upsBoss.verSoftware,VERSION);				// String representing Router software version, set at beginning of main.c
	strcpy((char *)upsOne.verSoftware,"Unknown");				// String requested from Master UPS representing Firmware version
	strcpy((char *)upsTwo.verSoftware,"Unknown");				// String requested from Slave UPS representing Firmware version
	upsBoss.freqInNom = UPS_FREQINNOM;
	upsBoss.voltInNom = UPS_VOLTINNOM;
	upsBoss.timeLoBatNom = UPS_TIMELOBATNOM;
	upsBoss.timeLoBatNom = UPS_TIMELOBATNOM;
	upsBoss.freqOutNom = UPS_FREQOUTNOM;
	upsBoss.powOutNom = UPS_POWOUTNOM;
	upsBoss.vaOutNom = UPS_VAOUTNOM;
	upsBoss.voltOutNom = UPS_VOLTOUTNOM;
	upsOne.msgStr[0] = upsTwo.msgStr[0] = upsBoss.msgStr[0] = NULL;
	upsOne.msgChr = upsTwo.msgChr = upsBoss.msgChr = ' ';
	upsOne.nextChrPos = upsTwo.nextChrPos = upsBoss.nextChrPos = 0;
	#if UPS_CHECKSUM == TRUE
		upsOne.checksumMode = upsTwo.checksumMode = upsBoss.checksumMode = CHKSUM_ON;
	#else
		upsOne.checksumMode = upsTwo.checksumMode = upsBoss.checksumMode = CHKSUM_OFF;
	#endif
	upsOne.timeOutStart = upsTwo.timeOutStart = upsBoss.timeOutStart = getTime();
	#if (defined SNMP_MIMIC)
		chargerComTimer = getTime();
	#endif
	upsOne.masterCmdBot = upsTwo.masterCmdBot = upsBoss.masterCmdBot = 0;
	upsOne.masterCmdTop = upsTwo.masterCmdTop = upsBoss.masterCmdTop = 0;
	upsOne.masterCmdTot = upsTwo.masterCmdTot = upsBoss.masterCmdTot = 0;
	upsOne.upsComState = upsTwo.upsComState = upsBoss.upsComState = COM_IDLE;
	upsOne.lastUpsComState = upsTwo.lastUpsComState = upsBoss.lastUpsComState = COM_WAITING;
	upsOne.notifyMsg = upsTwo.notifyMsg = upsBoss.notifyMsg = FALSE;
	upsOne.comErrors = upsTwo.comErrors = upsBoss.comErrors = 0;
	upsOne.timedCmd = upsTwo.timedCmd = upsBoss.timedCmd = 0 ; // start with first command
	upsOne.upsState = upsTwo.upsState = upsBoss.upsState = UPS_OFF;
	upsOne.startupInvCmd = upsTwo.startupInvCmd = FALSE;
	upsOne.timeCmdMade.msec = upsTwo.timeCmdMade.msec = upsBoss.timeCmdMade.msec = 0;
	upsOne.timeCmdMade.days = upsTwo.timeCmdMade.days = upsBoss.timeCmdMade.days = 0;
	upsOne.comErrors = upsTwo.comErrors = upsBoss.comErrors = 0;
	upsOne.invFaultAlm = upsTwo.invFaultAlm = upsBoss.invFaultAlm = OFF_ALARM;
	upsOne.batCond = upsTwo.batCond = upsBoss.batCond = NORMAL;
	upsOne.batSts = upsTwo.batSts = upsBoss.batSts = upsBoss.batStsLast = 0;
	//upsOne.batChgMode = upsTwo.batChgMode = upsBoss.batChgMode = ON_AUTO;
	upsOne.ampBat = upsTwo.ampBat = upsBoss.ampBat = 0;
	upsOne.ampChg = upsTwo.ampChg = upsBoss.ampChg = 0;
	upsOne.voltBat = upsTwo.voltBat = upsBoss.voltBat = NUM_CELLS * 2.0;
	upsOne.vBatShutdown = upsTwo.vBatShutdown = upsBoss.vBatShutdown = NUM_CELLS * 2.0;
	upsOne.vBatWarn = upsTwo.vBatWarn = upsBoss.vBatWarn = NUM_CELLS * 2.0;
	upsOne.batChgPct = upsTwo.batChgPct = upsBoss.batChgPct = 100.0;
	upsOne.freqIn = upsTwo.freqIn = upsBoss.freqIn = UPS_FREQINNOM;
	upsOne.voltIn = upsTwo.voltIn = upsBoss.voltIn = UPS_VOLTINNOM;
	upsOne.freqOut = upsTwo.freqOut = upsBoss.freqOut = UPS_FREQOUTNOM;
	upsOne.loadPctOut = upsTwo.loadPctOut = upsBoss.loadPctOut = 0;
	upsOne.powOut = upsTwo.powOut = upsBoss.powOut = 0;
	upsOne.voltOut = upsTwo.voltOut = upsBoss.voltOut = UPS_VOLTOUTNOM;
	upsOne.vaOut = upsTwo.vaOut = upsBoss.vaOut = 0;
	upsOne.pfOut = upsTwo.pfOut = upsBoss.pfOut = 0;
	upsOne.ampOut = upsTwo.ampOut = upsBoss.ampOut = 0;
	upsOne.voltBus = upsTwo.voltBus = upsBoss.voltBus = 0;
	upsOne.voltSupply = upsTwo.voltSupply = upsBoss.voltSupply = 0;
	upsOne.ampSupply = upsTwo.ampSupply = upsBoss.ampSupply = 0;
	upsOne.tAmb = upsTwo.tAmb = upsBoss.tAmb = 20.0;
	upsOne.tSink = upsTwo.tSink = upsBoss.tSink = 20.0;
	upsOne.invMode = upsTwo.invMode = upsBoss.invMode = MANUAL_OFF;
	upsOne.invOverloadTrip = upsTwo.invOverloadTrip = upsBoss.invOverloadTrip = NORMAL;
	upsOne.invFaultAlm = upsTwo.invFaultAlm = upsBoss.invFaultAlm = OFF_ALARM;
	upsOne.dcMode = upsTwo.dcMode = upsBoss.dcMode = AUTO_OFF;
	upsOne.chgMode = upsTwo.chgMode = upsBoss.chgMode = AUTO_SLOW;
	upsOne.bypassMode = upsTwo.bypassMode = upsBoss.bypassMode = AUTO;
	upsOne.syncMode = upsTwo.syncMode = upsBoss.syncMode = MANUAL_OFF;
	upsOne.secShutdownDelay = upsTwo.secShutdownDelay = upsBoss.secShutdownDelay = -1;
	upsOne.fanMode = upsTwo.fanMode = upsBoss.fanMode = AUTO_FAST;
	upsOne.batJouleNom = upsTwo.batJouleNom = BAT_MAX_JOULE;
	upsOne.batJoule = upsTwo.batJoule = BAT_MAX_JOULE;
	upsOne.batJouleFraction = upsTwo.batJouleFraction = upsBoss.batJouleFraction = 0;
	upsOne.battleShort = upsTwo.battleShort = upsBoss.battleShort = 0;		// off
	#if DUAL_BOARD==FALSE
		upsBoss.batJouleNom = BAT_MAX_JOULE;
		upsBoss.batJoule = BAT_MAX_JOULE;
	#else
		upsBoss.batJouleNom = BAT_MAX_JOULE * 2;
		upsBoss.batJoule = BAT_MAX_JOULE * 2;
	#endif
	//upsOne.batMode = upsTwo.batMode = upsBoss.batMode = NORMAL;
	strcpy((char *)upsOne.almMask,"^RALMNNNS");
	strcpy((char *)upsTwo.almMask,"^RALMNNNS");
	strcpy((char *)upsBoss.almMask,"^RALMNNNS");
	#ifdef CRYSTAL_FIX
		upsBoss.pfOutFiltered = 1.0;
	#endif
	upsOne.batWarnFlag = upsTwo.batWarnFlag = upsBoss.batWarnFlag = FALSE;
	//upsOne.startupPending = upsTwo.startupPending = upsBoss.startupPending = FALSE;
	#if defined UCLASS_FA10002
		upsBoss.optoOnOffLast = 3;
	#endif
	upsOne.bank1 = upsTwo.bank1 = upsBoss.bank1 = -1;			// set so we know when real data read
	upsOne.bank2 = upsTwo.bank2 = upsBoss.bank2 = -1;
	upsOne.bank3 = upsTwo.bank3 = upsBoss.bank3 = -1;
    #if (defined SNMP_MIMIC)
        chargerComTimer = getTime();
    #endif
    #if (defined NOV_CAT_J)
        // ORed summary relay status from Micro board
        upsOne.bossSummaryRelayStatus = 0;
        upsTwo.bossSummaryRelayStatus = 0;
        upsBoss.bossSummaryRelayStatus = 0;
    #endif
}

void ups_com(volatile struct upsDataStrucT *upsData)
{
	volatile int i = 0;
	//	volatile int debug = 0, debugMsgNumber = 1;	// debug
	//	volatile char debugStr[150];
	//	volatile int upsHuman = 1;
	//	volatile char tempChar;
	volatile char *pString;	// string pointer to pass elements of string array
	volatile float ftemp1, ftemp2;

        char *ptmpchar;                   // IAR compiler fixes/workarounds
        char tmpchar;
        comStates_t tmpcState;
        portName_t tmpport;		
        comWaitStates_t tmpwState;
        long tmplval1, tmplval2;
        float tmpfval1, tmpfval2;
        int tmpival1, tmpival2;

	//	__disable_interrupt();	// TODO - interrupt disable ups_com
	//DEBUG_PORT1_1;	// #################
	#if defined ETI_VERBOSE
		bool tempCheck;
	#endif

	pUps = upsData;		// set pointer to ups data structure address passed to this routine

	switch (upsData->upsComState) {

	case COM_IDLE:	// Waiting for timed command, state change cmd or ups generated notification (button press, etc.)
	    //if (upsData->upsComState != upsData->lastUpsComState)	// state entry code
        tmpcState = upsData->upsComState;
        if (tmpcState != upsData->lastUpsComState)
        {
			upsData->lastUpsComState = upsData->upsComState;
			// TODO - Communication dropout debug code
			P4OUT &= ~BIT7;  // set to zero
			sprintf((char *)eventStrTemp,"ups_com:IDLE entered, ups %d",upsData->upsNumber);
			addEvent(eventStrTemp,4);
		}  // end state entry code

		// first check to see if selected ups is starting to notify router with a changing condition
		if (usart_rx_buffer_count(upsData->port)) 	// char in buffer between router sourced cmds
		{
		  upsData->notifyMsg = TRUE;			// set flag so it's processed correctly
		  upsData->upsComState = COM_WAITING;		// Wait for reply
		  // debug
		  if (upsData == &upsTwo) 
		  {
             __NOP;
		  }
			// end debug
		} 
		else 	// check for command outside of rotation next...
		{
			// True if that UPS has command in buffer, but wait because average response time of UPS is ~200msec
			#if (!defined THALES_3KVA)
			  if ((masterCmdPending(pUps)) && (timer(upsData->timeOutStart, 250))) 
			#else
			// Thales 3KVA charger (upsTwo) was getting commands for LED display but not rotating commands
			// so this will force it to not take this branch.
			  if ((masterCmdPending(pUps)) && (timer(upsData->timeOutStart, 250)) && (upsData != &upsTwo)) 
			#endif
			  {
				// debug
				if (upsData == &upsTwo) 
				{
					__NOP;
				}
				// end debug
				// see if flag has been set to immediately send Inverter start command
				if (upsData->startupInvCmd == TRUE) 
				{
					usart_putstr("^RMODIEA", upsData->port);
					upsData->currentCmd = RMOD;
					upsData->startupInvCmd = FALSE;
				} 
				else 
				{
					// see if there is a inverter startup command in the command buffer
					if ( (strcmp((char *)upsData->masterCmdStr[upsData->masterCmdBot],"^RMODIEA") == 0) &&
					        (upsData->masterCmdTot != 0) )		// and it's active (not empty with pointer at old command)
					{		
						upsOne.startupInvCmd = upsTwo.startupInvCmd = TRUE;
						usart_putstr("^RMODIEA", upsData->port);
						upsData->currentCmd = RMOD;
						upsData->startupInvCmd = FALSE;
						if (++upsData->masterCmdBot >= MAX_MASTER_CMDS) 	// remove command, see if beyond buffer
						{
							upsData->masterCmdBot = 0;						// if it is, reset it
						}
						if (--upsData->masterCmdTot < 0) 					// decrement before checking for neg number
						{
							upsData->masterCmdTot = 0;						// reset to zero
						}
					} 
					else 	// if not, run regular command
					{
						// debug
						if (upsData == &upsTwo) 
						{
							__NOP;
						}
						// end debug
						masterCmdSend(pUps);
					}
				}
				// debug
				if (upsData == &upsTwo) 
				{
					__NOP;
				}
				// end debug
				upsData->upsComState = COM_WAITING;		// Wait for reply
			} else {	// otherwise, see if timer has elapsed for rotating command
				// see if timer elapsed for next command, but wait because average response time of UPS is ~200msec
				if ( (timer(upsData->timeCmdMade, UPS_POLL_INTERVAL)) &&
				        (timer(upsData->timeOutStart, 250)) &&
				        (rotatingCmdHold == FALSE) ) {			// waited for ups to be ready, controlled in UPS state machine
					// timedCmd is index into rotating command
					upsData->currentCmd = upsCmdRotation[upsData->timedCmd];
					/*
					if ((unsigned) ++upsData->timedCmd > (sizeof(upsCmdRotation)/sizeof(int) - 1))  // this works, I tested it.
					{
						upsData->timedCmd = 0;
					}
					*/
					if ((unsigned) ++upsData->timedCmd >= UPS_CMD_ROTATION_MAX) { // This is simpler and less likely to break with a different compiler
						upsData->timedCmd = 0;
					}
					// Then pick enum indexing into upsCmd
					strcpy((char *)upsData->StrTemp,&upsCmd[upsData->currentCmd][0]);
					//usart_putstr(&upsCmd[upsData->currentCmd][0], upsData->port);
					tmpport = upsData->port;
					usart_putstr(&upsCmd[upsData->currentCmd][0], tmpport);
					// debug
					if (upsData == &upsTwo) {
					    __NOP;
					}
					// end debug
					upsData->upsComState = COM_WAITING;		// Wait for reply
				}
			}
		}
		break;
	case COM_WAITING:	// waiting for incoming msg to complete
	    //if (upsData->upsComState != upsData->lastUpsComState) {	// state entry code
		tmpcState = upsData->upsComState;
		if (tmpcState != upsData->lastUpsComState) 	// state entry code
		{  
			upsData->lastUpsComState = upsData->upsComState;
			sprintf((char *)eventStrTemp,"ups_com:WAITING entered, ups %d",upsData->upsNumber);
			addEvent(eventStrTemp,4);
			upsData->msgStr[0] = upsData->StrTemp[0] = NULL;	// reset for new message
			upsData->nextChrPos = upsData->nextTempChrPos = 0;	// reset string pointers
			upsData->timeOutStart = upsData->timeCmdMade = getTime();
			//			ups_rx_buffer_flush(upsData->upsNumber);
			upsData->ComWaitState = COM_WAIT_START;
			upsData->lastComWaitState = COM_WAIT_DONE;
		}  // end state entry code
		
		// moved to COM_WAIT_MESSAGE in case noise is resetting timer
		if (timer(upsData->timeOutStart, 2000)) 
		{
			upsData->comErrors++;			// track errors
			upsData->upsComState = COM_IDLE;	// dump message and get next
		}
		
		// Pulled out of switch statement because it only runs if character is in the buffer, so the
		// message would end or timeout and until next message's 1st char could it execute, messing up
		// message synchronization
		switch (upsData->ComWaitState) 
		{
		case COM_WAIT_START:
		    //if (upsData->ComWaitState != upsData->lastComWaitState) {	// state entry code
			tmpwState = upsData->ComWaitState;
			if (tmpwState != upsData->lastComWaitState)
			{  
				upsData->lastComWaitState = upsData->ComWaitState;
				sprintf((char *)eventStrTemp,"ups_com:WAITING:COM_WAIT_START entered, ups %d",upsData->upsNumber);
				addEvent(eventStrTemp,4);
			}  															// end state entry code
			if (usart_rx_buffer_count(upsData->port)) 					// if char pending, look at it
			{			
				upsData->msgChr = usart_peekchar(upsData->port);		// look at, don't remove from buffer
				if (upsData->msgChr  == '^')  					        // start of message
				{
				    //sprintf((char *)eventStrTemp,"ups_com:WAITING 1st char of response received, ups %d, char[%c]"
				    //    ,upsData->upsNumber,upsData->msgChr);
					tmpchar = upsData->msgChr;
					sprintf((char *)eventStrTemp,"ups_com:WAITING 1st char of response received, ups %d, char[%c]"
					        ,upsData->upsNumber,tmpchar);                                        
					addEvent(eventStrTemp,4);
					upsData->ComWaitState = COM_WAIT_MESSAGE;			// good start of message, continue
				}
				else 									// unprintable, not part of the message
				{
				    //sprintf((char *)eventStrTemp,"ups_com:WAITING 1st char of response received, ups %d, ascii[%d]"
				    //    ,upsData->upsNumber,upsData->msgChr);
					tmpchar = upsData->msgChr;
					sprintf((char *)eventStrTemp,"ups_com:WAITING 1st char of response received, ups %d, ascii[%d]"
					        ,upsData->upsNumber,tmpchar);                                  
					addEvent(eventStrTemp,4);
					upsData->msgChr = usart_getchar(upsData->port);	// eat all unpritable characters
				}
			}
			break;
		case COM_WAIT_MESSAGE:
		    //if (upsData->ComWaitState != upsData->lastComWaitState) {	// state entry code
            tmpwState = upsData->ComWaitState;
            if (tmpwState != upsData->lastComWaitState)
            {
                upsData->lastComWaitState = upsData->ComWaitState;
                sprintf((char *)eventStrTemp,"ups_com:WAITING:COM_WAIT_MESSAGE entered, ups %d",upsData->upsNumber);
                addEvent(eventStrTemp,4);
            }  													// end state entry code
			/*
			if (timer(upsData->timeOutStart, 1000))  				// if waiting too long for valid message, quit
			{
			    //sprintf((char *)eventStrTemp,"ups_com:COM_WAIT_MESSAGE, Com Error, Message Timeout for [%s], ups %d"
			    //    ,upsCmd[upsData->currentCmd],upsData->upsNumber);
				ptmpchar = upsCmd[upsData->currentCmd];
				sprintf((char *)eventStrTemp,"ups_com:COM_WAIT_MESSAGE, Com Error, Message Timeout for [%s], ups %d"
						,ptmpchar,upsData->upsNumber);                               
				addEvent(eventStrTemp,4);
				upsData->upsComState = COM_IDLE;					// dump message and get next
				upsData->comErrors++;								// track errors
			}
			*/
			if (usart_rx_buffer_count(upsData->port)) 				// if char pending, get it
			{
				upsData->msgChr = usart_getchar(upsData->port);
				if (upsData->msgChr == 10) {						// newline shows end of data, ready for checksum
					#if defined COMM_MONITOR_PORT					// communication monitoring enabled
						if (upsData->upsNumber == 1) 				// Master number = 1
						{
						  usart_putstr("From Master: ", COMM_MONITOR_PORT); // send string to indicated port
						} 
						  else 
						{
						  usart_putstr("From Slave:  ", COMM_MONITOR_PORT); // send string to indicated port
						}
						usart_putstr(upsData->msgStr, COMM_MONITOR_PORT);   // send string to indicated port
						usart_putstr("\r\n", COMM_MONITOR_PORT);            // send string to indicated port
					#endif													// defined COMM_MONITOR_PORT
					if (upsData->checksumMode == CHKSUM_ON) {
						upsData->ComWaitState = COM_WAIT_CHECKSUM;
					} 
					else 
					{
						upsData->ComWaitState = COM_WAIT_DONE;
					}
				} 
				else 
				{
					if ( (upsData->msgChr >= 32) && (upsData->msgChr <= 126) ) 	// only store printable chars
					{	
					    //upsData->msgStr[upsData->nextChrPos++] = upsData->msgChr;
						tmpchar = upsData->msgChr;
						upsData->msgStr[upsData->nextChrPos++] = tmpchar;
						upsData->msgStr[upsData->nextChrPos] = NULL;	    // make it easy for string functions to process
						if (upsData->nextChrPos >= (TEMP_BUFFER_LENGTH-4))      // too big
						{
						  upsData->upsComState = COM_IDLE;		            // stop, dump and get next one
						}
					}
				}
			}
			break;
		case COM_WAIT_CHECKSUM:
		    //if (upsData->ComWaitState != upsData->lastComWaitState) {		// state entry code
			tmpwState = upsData->ComWaitState;
			if (tmpwState != upsData->lastComWaitState)
			{
				upsData->lastComWaitState = upsData->ComWaitState;
				sprintf((char *)eventStrTemp,"ups_com:WAITING:COM_WAIT_CHECKSUM entered, ups %d",upsData->upsNumber);
				addEvent(eventStrTemp,4);
			}  																// end state entry code
			if (usart_rx_buffer_count(upsData->port))						// if char pending, get it
			{  
				upsData->msgChr = usart_getchar(upsData->port);
				if (upsData->msgChr == 10)									// newline shows end of data, ready for checksum
				{			
					upsData->checksumMsg = atol((char *)upsData->StrTemp);
					#if defined COMM_MONITOR_PORT				          // communication monitoring enabled
						if (upsData->upsNumber == 1) 			          // see if the number is Master
						{
                                                  usart_putstr("Master Cksum:", COMM_MONITOR_PORT); // send string to indicated port
						} 
						else 
						{
						  usart_putstr("Slave Cksum: ", COMM_MONITOR_PORT); // send string to indicated port
						}
						usart_putstr(upsData->StrTemp, COMM_MONITOR_PORT); // send string to indicated port
						usart_putstr("\r\n", COMM_MONITOR_PORT);           // send string to indicated port
					#endif							   // defined COMM_MONITOR_PORT
					upsData->ComWaitState = COM_WAIT_DONE;
				} 
				else 
				{
                    if ( (upsData->msgChr >= '0') && (upsData->msgChr <= '9') ) // only store numbers
                    {
                        //upsData->StrTemp[upsData->nextTempChrPos++] = upsData->msgChr;
                        tmpchar = upsData->msgChr;
                        upsData->StrTemp[upsData->nextTempChrPos++] = tmpchar;
                        upsData->StrTemp[upsData->nextTempChrPos] = NULL;	   // make it easy for string functions to process
                        if (upsData->nextTempChrPos >= (TEMP_BUFFER_LENGTH-4))   // too big
                        {
                            upsData->upsComState = COM_IDLE;			   // stop, dump and get next one
                        }
                    }
				}
			}
			break;
		case COM_WAIT_DONE:
		    //if (upsData->ComWaitState != upsData->lastComWaitState) {			// state entry code
			tmpwState = upsData->ComWaitState;
			if (tmpwState != upsData->lastComWaitState)
			{
                upsData->lastComWaitState = upsData->ComWaitState;
                sprintf((char *)eventStrTemp,"ups_com:WAITING:COM_WAIT_DONE entered, ups %d, msg [%s]"
                    ,upsData->upsNumber,upsData->msgStr);
                addEvent(eventStrTemp,4);
			}  																	// end state entry code
			upsData->upsComState = COM_RESPONSE;
			break;
		default:
			upsData->upsComState = COM_IDLE;	// restart, wait for next transaction
			break;
		}
		break;
	case COM_RESPONSE:	// Not used for timed updating, this is when question is first asked by Upsilon
	    //if (upsData->upsComState != upsData->lastUpsComState) {				// state entry code
		tmpcState = upsData->upsComState;
		if (tmpcState != upsData->lastUpsComState)							// state entry code
		{
			upsData->lastUpsComState = upsData->upsComState;
			addEvent("ups_com:RESPONSE entered",4);
		}																	// end state entry code
		if (upsData->checksumMode == CHKSUM_ON) 
		{
			upsData->checksumCalc = 0;										// initialize
			for (i = 0;  (unsigned) i < strlen((char *)upsData->msgStr);i++) 
			{
			    //upsData->checksumCalc += (long) upsData->msgStr[i];			// add the ascii number of each char in string
				tmpchar = upsData->msgStr[i];
				upsData->checksumCalc += (long) tmpchar;
			}
			//if ( (upsData->checksumMsg == upsData->checksumCalc) ||			// make sure checksum...checks
			//    (strncmp("^RBUT",(char *)upsData->msgStr,5) == 0) )		// or button command patch (RBUT31 checksum bug)
			tmplval1 = upsData->checksumMsg;
			tmplval2 = upsData->checksumCalc;
			if ( (tmplval1 == tmplval2) ||			// make sure checksum...checks
					(strncmp("^RBUT",(char *)upsData->msgStr,5) == 0) )                        
			{	
				upsData->comMsgOkay = TRUE;
			}
			else 
			{
				upsData->comMsgOkay = FALSE;
			}
		} 
		else 
		{																// no checksum check, message passes
			upsData->comMsgOkay = TRUE;
		}
		if (upsData->comMsgOkay == TRUE) {										// this message is okay
			// in order to process correct response, scan and find from the header which case to select
			upsData->currentCmd = scanResponseHeader(upsData->msgStr);
			scanParams(pUps);													// get list of parameters
			// debug
			if (upsData == &upsTwo) 
			{
				__NOP;
			}
			// end debug
			if (--upsData->comErrors < 0) {								// successful, remove one error if any
				upsData->comErrors = 0;									// no less than 0 errors
			}
			if (--upsData->comErrors < 0) {								// successful, remove one error if any
				upsData->comErrors = 0;									// no less than 0 errors
			}
			/*	Following code commented out...
			if (upsData->currentCmd != CMD_END) 						// valid command
			{
				if (--upsData->comErrors < 0) 							// successful, remove one error if any
				{
					upsData->comErrors = 0;								// no less than 0 errors
				}
			}
			*/
			switch (upsData->currentCmd) {
			case RDAT01_PARAMS:													// parameter 0 is header
				// Item 1, voltage out
				pString = &StrParams[1][0];
				upsData->voltOut = atof((char *)pString);
				#if DUAL_BOARD==FALSE							// not ganged Model F boards
					upsBoss.voltOut = upsOne.voltOut;
				#else
					tmpfval2 = upsTwo.voltOut;                                      // set up for voltage computation
					tmpfval1 = upsOne.voltOut+tmpfval2;
					#if DUAL_BOARD_SERIES==TRUE										// dual boards in series
					    //upsBoss.voltOut = (upsOne.voltOut + upsTwo.voltOut);	
					    upsBoss.voltOut = tmpfval1;                                 // Ok, use the sum - series
					#else															// dual boards in parallel
					    //upsBoss.voltOut = (upsOne.voltOut + upsTwo.voltOut)/2;      // average
					    upsBoss.voltOut = tmpfval1/2;                               // in parrallel - take avg
					#endif
				#endif
				// Item 2, frequency out
				pString = &StrParams[2][0];
				upsData->freqOut = atof((char *)pString);
				#if DUAL_BOARD==FALSE
					upsBoss.freqOut = upsOne.freqOut;
				#else
				    //upsBoss.freqOut = (upsOne.freqOut + upsTwo.freqOut)/2;		// average
					tmpfval1 = upsOne.freqOut;
					tmpfval2 = tmpfval1 + upsTwo.freqOut;
					upsBoss.freqOut = tmpfval2/2;
				#endif
				// Item 3, current out
				pString = &StrParams[3][0];
				upsData->ampOut = atof((char *)pString);
				#if DUAL_BOARD==FALSE											// not ganged Model F boards
					upsBoss.ampOut = upsOne.ampOut;
				#else
					tmpfval2 = upsTwo.ampOut;									// set up for amperage computation
					tmpfval1 = upsOne.ampOut+tmpfval2;
					#if DUAL_BOARD_SERIES==TRUE									// dual boards in series
					    //upsBoss.ampOut = (upsOne.ampOut + upsTwo.ampOut)/2;		// average
						upsBoss.ampOut = tmpfval1/2;
					#else														// dual boards in parallel
					    //upsBoss.ampOut = (upsOne.ampOut + upsTwo.ampOut);		// add
						upsBoss.ampOut = tmpfval1;
					#endif
				#endif
				// Item 4, power out
				pString = &StrParams[4][0];
				upsData->powOut = atof((char *)pString);
				#if DUAL_BOARD==FALSE                                        // single board
					upsBoss.powOut = upsOne.powOut;
				#else
				    //upsBoss.powOut = upsOne.powOut + upsTwo.powOut;	        // add, in parallel
					tmpfval2 = upsTwo.powOut;
					tmpfval1 = upsOne.powOut+tmpfval2;
					upsBoss.powOut = tmpfval1;
				#endif
				// Item 5, volt-amps out
				pString = &StrParams[5][0];
				upsData->vaOut = atof((char *)pString);
				#if DUAL_BOARD==FALSE											// single board
					upsBoss.vaOut = upsOne.vaOut;
				#else
				    //upsBoss.vaOut = upsOne.vaOut + upsTwo.vaOut;			// add, in series
					tmpfval2 = upsTwo.vaOut;
					tmpfval1 = upsOne.vaOut+tmpfval2;
					upsBoss.vaOut = tmpfval1;
				#endif
				// Item 6, power factor out
				pString = &StrParams[6][0];
				upsData->pfOut = atof((char *)pString);
				#if DUAL_BOARD==FALSE
					upsBoss.pfOut = upsOne.pfOut;
				#else
				    //upsBoss.pfOut = (upsOne.pfOut + upsTwo.pfOut)/2;		// average
					tmpfval2 = upsTwo.pfOut;
					tmpfval1 = upsOne.pfOut+tmpfval2;
					upsBoss.pfOut = tmpfval1/2;
				#endif
				// Item 7, load% out
				pString = &StrParams[7][0];
				upsData->loadPctOut = atof((char *)pString);
				#if DUAL_BOARD==FALSE										// not ganged Model F boards
					upsBoss.loadPctOut = upsOne.loadPctOut;
				#else
				    //upsBoss.loadPctOut = fMax(upsOne.loadPctOut,upsTwo.loadPctOut);	// max
					tmpfval2 = upsTwo.loadPctOut;
					tmpfval1 = upsOne.loadPctOut;
					upsBoss.loadPctOut = fMax(tmpfval1,tmpfval2);
				#endif
				// Crystal can't read input voltage/freq unless with update
				#ifndef CRYSTAL_OLD	
					// Item 8, volt input
					pString = &StrParams[8][0];
					upsData->voltIn = atof((char *)pString);
					#if DUAL_BOARD==FALSE									// not ganged Model F boards
						upsBoss.voltIn = upsOne.voltIn;
					#else
					    //upsBoss.voltIn = fMin(upsOne.voltIn,upsTwo.voltIn);	// min
						tmpfval2 = upsTwo.voltIn;
						tmpfval1 = upsOne.voltIn;
						upsBoss.voltIn = fMin(tmpfval1,tmpfval2);
					#endif
					// Item 9, frequency input
					pString = &StrParams[9][0];
					upsData->freqIn = atof((char *)pString);
					#if DUAL_BOARD==FALSE || defined THALES_3KVA				// not ganged Model F boards
						upsBoss.freqIn = upsOne.freqIn;
					#else
						//upsBoss.freqIn = (upsOne.freqIn + upsTwo.freqIn + 0.5)/2;	    // average
						//upsBoss.freqIn = ((float)upsOne.freqIn + (float)upsTwo.freqIn)*0.5; // average, faster
						tmpfval2 = upsTwo.freqIn;
						tmpfval1 = upsOne.freqIn + tmpfval2;
						upsBoss.freqIn = tmpfval1*0.5;
					#endif
				#endif
				// Item 10, ambient temperature
				pString = &StrParams[10][0];
				upsData->tAmb = atof((char *)pString);
				#if DUAL_BOARD==FALSE && !defined THALES_3KVA					// not ganged Model F boards
					upsBoss.tAmb = upsOne.tAmb;
				#else
				    //upsBoss.tAmb = fMax(upsOne.tAmb,upsTwo.tAmb);				// max
					tmpfval2 = upsTwo.tAmb;
					tmpfval1 = upsOne.tAmb;
					upsBoss.tAmb = fMax(tmpfval1,tmpfval2);
				#endif
				// Item 11, heatsink temperature
				pString = &StrParams[11][0];
				upsData->tSink = atof((char *)pString);
				#if DUAL_BOARD==FALSE	&& !defined THALES_3KVA				// not ganged Model F boards
					upsBoss.tSink = upsOne.tSink;
				#else
				    //upsBoss.tSink = fMax(upsOne.tSink,upsTwo.tSink);			// max
					tmpfval1 = upsOne.tSink;
					tmpfval2 = upsTwo.tSink;
					upsBoss.tSink = fMax(tmpfval1,tmpfval2);
				#endif
				// Item 12, battery voltage
				pString = &StrParams[12][0];
				upsData->voltBat = (float) atof((char *)pString);
				#if DUAL_BOARD==FALSE							// not ganged Model F boards
					upsBoss.voltBat = upsOne.voltBat;	
				#else
				    //upsBoss.voltBat = fMin(upsOne.voltBat,upsTwo.voltBat);	// min
					tmpfval2 = upsTwo.voltBat;
					tmpfval1 = upsOne.voltBat;
					upsBoss.voltBat = fMin(tmpfval1,tmpfval2);
				#endif
				// Item 13, battery discharge current
				pString = &StrParams[13][0];
				upsData->ampBat = atof((char *)pString);
				#if DUAL_BOARD==FALSE										// not ganged Model F boards
					upsBoss.ampBat = upsOne.ampBat;
				#else
				    //upsBoss.ampBat = (upsOne.ampBat + upsTwo.ampBat);		// add two strings
					tmpfval2 = upsTwo.ampBat;
					upsBoss.ampBat = upsOne.ampBat + tmpfval2;                                        
				#endif
				#if BAT_CAP_METHOD==BAT_CAP_MODEL_F							// do local calculation for battery capacity
					// Item 14, estimated battery runtime available
					pString = &StrParams[14][0];
					upsData->estSecBat = scanEstSecBat(pString);
					tmplval1 = upsTwo.estSecBat;
					upsBoss.estSecBat = lMin(upsOne.estSecBat,tmplval1);	// min
				#endif
				#if (defined NOV_CAT_J)
					// upsData is pointer, upsOne is variable, compare addresses
					if (upsData == &upsOne) {								// is this coming from upsBoss? (upsOne)
						pString = &StrParams[15][0];
						upsOne.linesNumIn = atoi((char *)pString);
					}
				#endif														// #if defined NOV_CAT_J
				#ifdef CRYSTAL_OLD											// Original Crystal can't sense input voltage
					if (upsBoss.upsState == UPS_ON_UTIL) {	
						upsOne.voltIn = upsTwo.voltIn = UPS_VOLTINNOM;
						upsOne.freqIn = upsTwo.freqIn = UPS_FREQINNOM;
						#ifdef CRYSTAL_FIX
							upsBoss.pfOutFiltered += (upsBoss.pfOut-upsBoss.pfOutFiltered)*0.1;
						#endif
					} else {
						upsOne.voltIn = upsTwo.voltIn = 0.0;
						upsOne.freqIn = upsTwo.freqIn = 0.0;
						#ifdef CRYSTAL_FIX
							upsBoss.powOut = (upsOne.voltBat * upsOne.ampBat) + (upsTwo.voltBat * upsTwo.ampBat);
							upsBoss.vaOut = upsBoss.powOut/upsBoss.pfOutFiltered;
							upsBoss.ampOut = upsBoss.vaOut/upsBoss.voltOut;
							upsBoss.pfOut = upsBoss.pfOutFiltered;
							ftemp1 = (upsBoss.powOut/UPS_POWOUTNOM) * 100.0;
							ftemp2 = (upsBoss.vaOut/UPS_VAOUTNOM) * 100.0;
							upsBoss.loadPctOut = fMax(ftemp1,ftemp2);
						#endif
					}
				#endif
				break;
			case RDAT02_PARAMS2:
				// Item 1, DC bus voltage
				pString = &StrParams[1][0];								// parameter 0 is header
				upsData->voltBus = atof((char *)pString);
				#if DUAL_BOARD == FALSE || defined THALES_3KVA
					upsBoss.voltBus = upsData->voltBus;							// It is just the one
				#else
				    //upsBoss.voltBus = (upsOne.voltBus + upsTwo.voltBus)/2;		// average
					tmpfval2 = upsTwo.voltBus;
					tmpfval1 = upsOne.voltBus + tmpfval2;
					upsBoss.voltBus = tmpfval1/2;
				#endif
				// Item 2, 12VDC power supply voltage
				pString = &StrParams[2][0];
				upsData->voltSupply = atof((char *)pString);
				#if DUAL_BOARD == FALSE
					upsBoss.voltSupply = upsData->voltSupply;						// It is just the one
				#else
				    //upsBoss.voltSupply = (upsOne.voltSupply + upsTwo.voltSupply)/2;	// average
					tmpfval2 = upsTwo.voltSupply;
					tmpfval1 = upsOne.voltSupply + tmpfval2;
					upsBoss.voltSupply = tmpfval1/2;
				#endif
				// Item 3, 12VDC power supply current
				pString = &StrParams[3][0];
				upsData->ampSupply = atof((char *)pString);
				#if DUAL_BOARD == FALSE
					upsBoss.ampSupply = upsData->ampSupply;						// It is just the one
				#else
				    //upsBoss.ampSupply = (upsOne.ampSupply + upsTwo.ampSupply)/2;	// average
					tmpfval2 = upsTwo.ampSupply;
					tmpfval1 = upsOne.ampSupply + tmpfval2;
					upsBoss.ampSupply = tmpfval1/2;
				#endif
				pString = &StrParams[4][0];
				upsData->bank1 = atol((char *)pString);
				upsBoss.bank1 = upsOne.bank1;										// use Master
				pString = &StrParams[5][0];
				upsData->bank2 = atol((char *)pString);
				upsBoss.bank2 = upsOne.bank2;										// use Master
				pString = &StrParams[6][0];
				upsData->bank3 = atol((char *)pString);
				upsBoss.bank3 = upsOne.bank3;										// use Master
				// empty param
				pString = &StrParams[8][0];
				upsData->ledStatus[0] = atol((char *)pString);
				upsBoss.ledStatus[0] = upsOne.ledStatus[0];						// use Master
				pString = &StrParams[9][0];
				upsData->ledStatus[1] = atol((char *)pString);
				upsBoss.ledStatus[1] = upsOne.ledStatus[1];						// use Master
				pString = &StrParams[10][0];
				upsData->ledStatus[2] = atol((char *)pString);
				upsBoss.ledStatus[2] = upsOne.ledStatus[2];						// use Master
				pString = &StrParams[11][0];
				upsData->ledStatus[3] = atol((char *)pString);
				upsBoss.ledStatus[3] = upsOne.ledStatus[3];						// use Master
				break;
			case RMOD:																// changing mode will return current modes
			case RDAT03_STATUS:
				#if !defined THALES_3KVA
					pString = &StrParams[1][0];							// parameter 0 is header
					upsData->invMode = selectStrOpMode(pString);
					#if DUAL_BOARD==FALSE											// single board
						upsBoss.invMode = upsOne.invMode;
					#endif
					pString = &StrParams[2][0];
					upsData->dcMode = selectStrOpMode(pString);
					#if DUAL_BOARD==FALSE											// single board
						upsBoss.dcMode = upsOne.dcMode;
					#endif
					pString = &StrParams[3][0];
					upsData->chgMode = selectStrOpMode(pString);
					#if DUAL_BOARD==FALSE											// single board
						#if (!defined THALES_3KVA)
							upsBoss.chgMode = upsOne.chgMode;
						#else	// defined THALES_3KVA, upsTwo is Model F charger
							if ((upsBoss.dcMode == AUTO_ON) || (upsBoss.dcMode == MANUAL_ON)) {// On battery
								upsBoss.chgMode = AUTO_OFF;							// Auto Off
							} else {
								upsBoss.chgMode = upsTwo.chgMode;
							}
						#endif	// !defined THALES_3KVA
					#else															// dual boards
						#if !defined SNMP_MIMIC										// dual ups, dual charger RS485
							//if (upsOne.chgMode == upsTwo.chgMode) {
							tmpival1 = upsTwo.chgMode;
							if (upsOne.chgMode == tmpival1) {
								upsBoss.chgMode = upsOne.chgMode;
							}
						#else
							if ((upsBoss.dcMode == AUTO_ON) || (upsBoss.dcMode == MANUAL_ON)) {// On battery
								upsBoss.chgMode = AUTO_OFF;							// Auto Off
							} else {
								upsBoss.chgMode = upsThree.chgMode;
							}
						#endif
					#endif
					pString = &StrParams[4][0];
					upsData->bypassMode = selectStrOpMode(pString);					// if either UPS on bypass
					#if DUAL_BOARD==FALSE											// single board
						upsBoss.bypassMode = upsOne.bypassMode;
					#else															// dual boards
						#if defined DUAL_BOARD_MASTER_BYPASS_CONTROL
							upsBoss.bypassMode = upsOne.bypassMode;					// Report Master only
						#else														// not DUAL_BOARD_MASTER_BYPASS_CONTROL
							if ((upsOne.bypassMode == ON) || (upsTwo.bypassMode == ON)) 
							{
								upsBoss.bypassMode = ON;							// report system on bypass
							} 
							else if ((upsOne.bypassMode == OFF) || (upsTwo.bypassMode == OFF)) 
                            {
                                upsBoss.bypassMode = OFF;							// same for bypass forced off
                            } 
                            else 
                            {
                                upsBoss.bypassMode = AUTO;							// otherwise report Bypass in auto
                            }
						#endif														// DUAL_BOARD_MASTER_BYPASS_CONTROL
					#endif
					pString = &StrParams[5][0];
					upsData->syncMode = selectStrOpMode(pString);
					#if DUAL_BOARD==FALSE											// single board
						upsBoss.syncMode = upsOne.syncMode;
					#endif
					pString = &StrParams[6][0];
					upsData->fanMode = selectStrOpMode(pString);
					#if DUAL_BOARD==FALSE											// single board
						upsBoss.fanMode = upsOne.fanMode;
					#endif
				#else	//	defined THALES_3KVA
					pString = &StrParams[1][0];							// parameter 0 is header
					upsData->invMode = selectStrOpMode(pString);
					upsBoss.invMode = upsOne.invMode;								// upsOne is UPS
					pString = &StrParams[2][0];
					upsData->dcMode = selectStrOpMode(pString);
					upsBoss.dcMode = upsOne.dcMode;								// upsOne is UPS
					pString = &StrParams[3][0];
					upsData->chgMode = selectStrOpMode(pString);
					upsBoss.chgMode = upsTwo.chgMode;								// upsTwo is Model F charger
					pString = &StrParams[4][0];
					upsData->bypassMode = selectStrOpMode(pString);				// if either UPS on bypass
					upsBoss.bypassMode = upsOne.bypassMode;						// upsOne is UPS
					pString = &StrParams[5][0];
					upsData->syncMode = selectStrOpMode(pString);
					upsBoss.syncMode = upsOne.syncMode;							// upsOne is UPS
					pString = &StrParams[6][0];
					upsData->fanMode = selectStrOpMode(pString);
					upsBoss.fanMode = upsOne.fanMode;								// upsOne is UPS
				#endif	//	!defined THALES_3KVA
				#if defined BATTERY_TRAY_INDICATION
					pString = &StrParams [7][0];
					batteryTrayStatus = atoi ((char *) pString);
				#endif
				break;
			case RDAT06_MISC_PARMS:
				#if (!defined SNMP_MIMIC)
					pString = &StrParams[1][0];								// parameter 0 is header
					//upsData->batCond 
					i = atoi((char *)pString);
					switch(i) {
					case 0:
						upsData->batCond = NORMAL;
						break;
					case 1:
						upsData->batCond = WARN;
						break;
					case 2:
						upsData->batCond = FAULT;
						break;
					case 3:
						upsData->batCond = OVER_VOLTAGE;
						break;
					default:
						upsData->batCond = FAULT;
						break;
					}
					#if DUAL_BOARD==FALSE												// single board
						#if (!defined THALES_3KVA)
							upsBoss.batCond = upsOne.batCond;
							// chgModeSnmp 0=float, 1=Charging, 2=Resting, 3=Discharging
							// chgMode 1=fast, 2=slow, 3=off
							pString = &StrParams[3][0];
							upsData->chgModeSnmp = atoi((char *)pString);
							upsBoss.chgModeSnmp = upsOne.chgModeSnmp;
						#else	// defined THALES_3KVA, upsTwo is Model F charger
							
							pString = &StrParams[1][0];								// parameter 0 is header
							//upsData->batCond 
							i = atoi((char *)pString);
							switch(i) {
							case 0:
								upsData->batCond = NORMAL;
								break;
							case 1:
								upsData->batCond = WARN;
								break;
							case 2:
								upsData->batCond = FAULT;
								break;
							case 3:
								upsData->batCond = OVER_VOLTAGE;
								break;
							default:
								upsData->batCond = FAULT;
								break;
							}
							// substitute information from Charger (upsTwo) under certain conditions
							switch (upsOne.batCond) {
							case OVER_VOLTAGE:
								//upsData->batCond = OVER_VOLTAGE;
								upsBoss.batCond = OVER_VOLTAGE;
								break;
							case FAULT:
								//upsData->batCond = FAULT;
								upsBoss.batCond = FAULT;
								break;
							default:									// no override, use charger condition
								if ((upsTwo.comOkay == FALSE) && (upsBoss.upsState != UPS_ON_BAT)) {
									upsBoss.batCond = FAULT;				// show charger fault
								} else {
									upsBoss.batCond = upsTwo.batCond;	// use charger battery condition
								}
								break;
							}

							// Battery Low, 1=Low, 0=Normal
							pString = &StrParams[2][0];
							upsData->batSts = atoi((char *)pString);
							//upsData->batSts = upsTwo.batSts;
							upsBoss.batSts = upsTwo.batSts;

							// chgModeSnmp 0=float, 1=Charging, 2=Resting, 3=Discharging
							// chgMode 1=fast, 2=slow, 3=off
							pString = &StrParams[3][0];
							upsData->chgModeSnmp = atoi((char *)pString);
							upsData->chgModeSnmp = upsTwo.chgModeSnmp;
							upsBoss.chgModeSnmp = upsTwo.chgModeSnmp;
							// With separate charger, we can only get information from it when utility is available
							// so if UPS is on battery, report discharging, if not report Charger status
							if ((upsBoss.dcMode == AUTO_ON) || (upsBoss.dcMode == MANUAL_ON)) {
								upsBoss.chgModeSnmp = 3;						// Discharging
							} else {
								// if utility available but charger not talking or energized
								if ((upsTwo.comOkay == FALSE) && (upsBoss.upsState != UPS_ON_BAT)) {
									upsBoss.chgModeSnmp = 2;// Battery Charge, 0=float, 1=Charging, 2=Resting, 3=Discharging
								} else {
									// chgModeSnmp 0=float, 1=Charging, 2=Resting, 3=Discharging
									// chgMode 1=fast, 2=slow, 3=off
									switch (upsBoss.chgMode) {
									case 1:
										upsBoss.chgModeSnmp = 1;
										break;
									case 2:
										upsBoss.chgModeSnmp = 0;
										break;
									}
								}
							}
						#endif	// !defined THALES_3KVA
					#else		// DUAL_BOARD==TRUE
						if ((upsOne.batCond == NORMAL) && (upsTwo.batCond == NORMAL)) {
							upsBoss.batCond = NORMAL;
						} else {
							//upsBoss.batCond = (operatingModesT) iMax((int) upsOne.batCond, (int) upsTwo.batCond);
							tmpival1 = upsTwo.batCond;
							upsBoss.batCond = (operatingModesT) iMax((int) upsOne.batCond, (int) tmpival1);
						}
					#endif		// DUAL_BOARD==FALSE single board
					// Battery Low, 1=Low, 0=Normal
					pString = &StrParams[2][0];
					upsData->batSts = atoi((char *)pString);
					#if DUAL_BOARD==FALSE											// single board
						upsBoss.batSts = upsOne.batSts;
					#else	// DUAL_BOARD==TRUE
						if ((upsOne.batSts == 1) || (upsTwo.batSts == 1)) 
						{
							upsBoss.batSts = 1;
						} 
						else 
						{
							upsBoss.batSts = 0;
						}

						// chgModeSnmp 0=float, 1=Charging, 2=Resting, 3=Discharging
						// chgMode 1=fast, 2=slow, 3=off
						pString = &StrParams[3][0];
						upsData->chgModeSnmp = atoi((char *)pString);
						//if (upsOne.chgModeSnmp == upsTwo.chgModeSnmp) {	// if they agree
						tmpival1 = upsTwo.chgModeSnmp;
						if (upsOne.chgModeSnmp == tmpival1) {	// if they agree
							upsBoss.chgModeSnmp = upsOne.chgModeSnmp;		// tell the Boss
						}
					#endif
					tmpival1 = upsBoss.batStsLast;
					if (upsBoss.batSts != tmpival1) {
						upsBoss.batStsLast = upsBoss.batSts;
						if (upsBoss.batSts == 1) {
							masterCmdAddBoth("^RDAT32");					// command to turn on low bat signal relay
						} else {
							masterCmdAddBoth("^RDAT33");					// command to turn off low bat signal relay
						}
					}
					/*
					#if (!defined THALES_3KVA)
						// in ups chg_status 1=fast, 2=slow, 3=off, reports => fast=1, slow=0, Discharging=3
						pString = &StrParams[3][0];
						upsData->chgModeSnmp = atoi((char *)pString);
					#else	// defined THALES_3KVA, upsTwo is Model F Charger
						// in ups chg_status 1=fast, 2=slow, 3=off, reports => fast=1, slow=0, Discharging=3
						pString = &StrParams[3][0];
						upsData->chgModeSnmp = atoi((char *)pString);
						upsData->chgModeSnmp = upsTwo.chgModeSnmp;
						upsBoss.chgModeSnmp = upsTwo.chgModeSnmp;
						// With separate charger, we can only get information from it when utility is available
						// so if UPS is on battery, report discharging, if not report Charger status
						if ((upsBoss.dcMode == AUTO_ON) || (upsBoss.dcMode == MANUAL_ON)) {// On battery
							upsBoss.chgModeSnmp = 3;						// Discharging
						} else {
							upsBoss.chgModeSnmp = upsThree.chgModeSnmp;
						}
					#endif	// !defined THALES_3KVA
					*/
				#else	// defined SNMP_MIMIC
					// SNMP_MIMIC configuration has charger box with two chargers connected through RS485
					pString = &StrParams[1][0];								// parameter 0 is header
					//upsData->batCond 
					i = atoi((char *)pString);
					switch(i) {
					case 0:
						upsData->batCond = NORMAL;
						break;
					case 1:
						upsData->batCond = WARN;
						break;
					case 2:
						upsData->batCond = FAULT;
						break;
					case 3:
						upsData->batCond = OVER_VOLTAGE;
						break;
					default:
						upsData->batCond = FAULT;
						break;
					}
					// keep reset for charger loss of communication alarm blanking below
					if (upsBoss.upsState == UPS_ON_BAT) {
						chargerComTimer = getTime();
					}
					// substitute information from Charger (upsThree) under certain conditions
					if ((upsOne.batCond == OVER_VOLTAGE) || (upsTwo.batCond == OVER_VOLTAGE)) 
					{
						upsBoss.batCond = OVER_VOLTAGE;
					} 
					else 
					{
						if ((upsOne.batCond == FAULT) || (upsTwo.batCond == FAULT)) 
						{
							upsBoss.batCond = FAULT;
						}
						else 
						{
							// check communication if on utility and wait after returning from battery so fault can clear
							if ( (upsThree.comOkay == FALSE) && (upsBoss.upsState != UPS_ON_BAT)
								&& (timer(chargerComTimer, 60000)) ) 
							{
								upsBoss.batCond = FAULT;				// show charger fault
							} 
							else 
							{
								// since returning from a battery run needs some delay and making
								// sure the charger is running with good communication
								if ( (upsThree.comOkay == TRUE) && (upsBoss.upsState != UPS_ON_BAT)
									&& (timer(chargerComTimer, 60000)) ) 
								{
									upsBoss.batCond = upsThree.batCond;	// use charger battery condition
								}
							}
						}
					}
					// Battery Low, 1=Low, 0=Normal
					pString = &StrParams[2][0];
					upsData->batSts = atoi((char *)pString);
					upsData->batSts = upsThree.batSts;
					upsBoss.batSts = upsThree.batSts;
					// in ups chg_status 1=fast, 2=slow, 3=off, reports => fast=1, slow=0, Discharging=3
					pString = &StrParams[3][0];
					upsData->chgModeSnmp = atoi((char *)pString);
					upsData->chgModeSnmp = upsThree.chgModeSnmp;
					upsBoss.chgModeSnmp = upsThree.chgModeSnmp;
					// With separate charger, we can only get information from it when utility is available
					// so if UPS is on battery, report discharging, if not report Charger status
					if ((upsBoss.dcMode == AUTO_ON) || (upsBoss.dcMode == MANUAL_ON)) // On battery
					{
						upsBoss.chgModeSnmp = 3;						// Discharging
					} 
					else 
					{
						// if utility available but charger not talking or energized
						if ((charger.comOkay == FALSE) && (upsBoss.upsState != UPS_ON_BAT)) 
						{
							upsBoss.chgModeSnmp = 2;// Battery Charge, 0=float, 1=Charging, 2=Resting, 3=Discharging
						}
					}
				#endif	// defined SNMP_MIMIC
				// Second on battery
				pString = &StrParams[4][0];
				upsData->secOnBat = atol((char *)pString);
				#if DUAL_BOARD==FALSE											// single board
					upsBoss.secOnBat = upsOne.secOnBat;
				#else
				    //upsBoss.secOnBat = lMax(upsOne.secOnBat,upsTwo.secOnBat);
					tmplval2 = upsTwo.secOnBat;
					tmplval1 = upsOne.secOnBat;
					upsBoss.secOnBat = lMax(tmplval1,tmplval2);
				#endif
				// Battery Capacity
				#if BAT_CAP_METHOD==BAT_CAP_MODEL_F							// if true then calculated in batCapJouleUpdate()
					pString = &StrParams[5][0];
					upsData->batChgPct = atof((char *)pString)/10.0;			// 90.4% = 904
					if ((upsData->batChgPct < 0) || (upsData->batChgPct > 100.0)) {
						upsData->batChgPct = 0;
					}
					#if DUAL_BOARD==FALSE										// single board
						upsBoss.batChgPct = upsOne.batChgPct;
					#else
					    tmpfval1 = upsTwo.batChgPct;
						upsBoss.batChgPct = fMin(upsOne.batChgPct,tmpfval1);
					#endif
				#endif
				// Temperature Alarm, Ambient
				pString = &StrParams[6][0];
				if (atoi((char *)pString) == 1) {			// temp alarm
					upsData->tAmbMode = ON_ALARM;
				}
				else {
					upsData->tAmbMode = NORMAL;
				}
				#if DUAL_BOARD==FALSE											// single board
					upsBoss.tAmbMode = upsOne.tAmbMode;
				#endif
				// Inverter Fault
				pString = &StrParams[7][0];		// output bad alarm
				if (atoi((char *)pString) == 1) {
				    if (upsBoss.upsState != UPS_OFF) {                    // only report alarm when inverter running
				        upsBoss.invFaultAlm = upsData->invFaultAlm = ON_ALARM;
					}
				} else {
					upsData->invFaultAlm = OFF_ALARM;
					// see if both Inverter Fault alarms are now clear
					#if DUAL_BOARD==FALSE											// single board
						upsBoss.invFaultAlm = upsOne.invFaultAlm;
					#else
						if ((upsOne.invFaultAlm == OFF_ALARM) && (upsTwo.invFaultAlm == OFF_ALARM)) {
							upsBoss.invFaultAlm = OFF_ALARM;	// if so, clear main alarm
						}
					#endif
				}
				//	pString = &StrParams[8][0];	// UPS Software version
				//	strcpy(upsData->verSoftware, pString);
				pString = &StrParams[9][0];
				#if !defined SNMP_MIMIC					// this is set in charger_com()
					upsData->ampChg = atof((char *)pString);
				#endif
				#if DUAL_BOARD==FALSE											// single board
					#if (!defined THALES_3KVA)
						upsBoss.ampChg = upsOne.ampChg;
					#else	// defined THALES_3KVA, upsTwo is Model F Charger
						upsBoss.ampChg = upsTwo.ampChg;
					#endif	// !defined THALES_3KVA
				#else
				    //upsBoss.ampChg = (upsOne.ampChg + upsTwo.ampChg);
					tmpfval2 = upsTwo.ampChg;
					tmpfval1 = upsOne.ampChg + tmpfval2;
					upsBoss.ampChg = tmpfval1;
				#endif
				if (strlen((char *)&StrParams[10][0]) < IDENT_LEN_MAX) {
					strcpy((char *)upsData->man,(char *)&StrParams[10][0]);
				}
				if (strlen((char *)&StrParams[11][0]) < IDENT_LEN_MAX) {
					strcpy((char *)upsData->model,(char *)&StrParams[11][0]);
				}
				#if (defined NOV_CAT_J)
                    // RDAT06-12 Summary Alarm Status, 1=on 0=off
                    // ORed summary relay status from Micro board
                    pString = &StrParams[12][0];
                    upsData->bossSummaryRelayStatus = atoi((char *)pString);
                    if ((upsOne.bossSummaryRelayStatus == 1) || (upsTwo.bossSummaryRelayStatus == 1))
                    {
                        upsBoss.bossSummaryRelayStatus = 1;
                    }
                    else
                    {
                        upsBoss.bossSummaryRelayStatus = 0;
                    }
				#endif
				break;
			#if defined UCLASS_FA10002
			case RDAT07_OPTO_INPUTS:
				pString = &StrParams[1][0];							// parameter 0 is header
				upsData->optoEpo = atoi((char *) pString);
				upsBoss.optoEpo = upsData->optoEpo;
				pString = &StrParams[2][0];
				upsData->optoBattleShort = atoi((char *) pString);
				upsBoss.battleShort = upsData->optoBattleShort;
				pString = &StrParams[3][0];
				upsData->optoOnOff = atoi((char *) pString);
				upsBoss.optoOnOff = upsData->optoOnOff;
				pString = &StrParams[4][0];
				upsData->optoTbd = atoi((char *) pString);				// if either UPS on bypass
				upsBoss.optoTbd = upsData->optoTbd;
				break;
			#endif 																// defined UCLASS_FA10002
			case RDAT31_UPS_PROG_VER:			// Get UPS Master and Slave Firmware Version
				// should be 1 params up to slot 0 and less than max size for target array item
				if (strlen((char *)&StrParams[1][0]) < IDENT_LEN_MAX) {
					strcpy((char *)upsData->verSoftware,(char *)&StrParams[1][0]);
				} else {
					upsData->comOkay = FALSE;
					#if DUAL_BOARD==FALSE											// single board
						upsBoss.comOkay = upsOne.comOkay;
					#endif
				}
				break;
			case RBUT:
				#if defined ETI_DEBUG_PRINT
					// Put ETI User Requested Protocol Extension handlers here for RBUT20-23
					pString = &StrParams[1][0];
					tempCheck = (strcmp ((char *) pString, "^1") == 0);
					pString = &StrParams [0][0];
					if (strcmp ((char *) pString, "^RBUT20") == 0) {
						if (tempCheck) {
							usart_putstr ("Running...\r", ETI_PORT);	// RBUT20 Accepted
						} else {
							usart_putstr ("Rejected.\r", ETI_PORT);		// RBUT20 Rejected
						}
					}
					if (strcmp ((char *) pString, "^RBUT21") == 0) {
						if (tempCheck) {
							usart_putstr ("Complete.\r", ETI_PORT);		// RBUT21 Accepted
						} else {
							usart_putstr ("Rejected.\r", ETI_PORT);		// RBUT21 Rejected
						}
					}
					if (strcmp ((char *) pString, "^RBUT22") == 0) {
						if (tempCheck) {
							usart_putstr ("Running...\r", ETI_PORT);	// RBUT22 Accepted
						} else {
							usart_putstr ("Rejected.\r", ETI_PORT);		// RBUT22 Rejected
						}
					}
					if (strcmp ((char *) pString, "^RBUT23") == 0) {
						if (tempCheck) {
							usart_putstr ("Complete.\r", ETI_PORT);		// RBUT23 Accepted
						} else {
							usart_putstr ("Rejected.\r", ETI_PORT);		// RBUT23 Rejected
						}
					}
				#endif
				strcpy((char *)upsBoss.msgStr,(char *)upsData->msgStr);
				upsData->notifyMsg = upsBoss.notifyMsg = TRUE;
				if (strcmp((char *)upsData->msgStr,"^RBUT02") == 0) {	// "Silence" button
					updateAlm(SONALERT_OFF); 					// Turn off sonalert
				}
				/*
				if (strcmp((char *)upsData->msgStr,"^RBUT08") == 0) {	// "On" button, or Remote on
					upsData->startupPending = TRUE; 					// Set flag for ups state machine
				}
				*/
				break;
			case RDAT04_NOTIFY_ON:	// for now, all of these will be processed the same way
			case RDAT05_NOTIFY_OFF:
			case RDAT20_SLAVE_ON:
			case RDAT21_SLAVE_OFF:
			case RDAT22_CHKSUM_ON:
			case RDAT23_CHKSUM_OFF:
			case RDAT24_DIS_SAVE_LED:
			case RDAT25_DIS_LOAD_LED:
			case RDAT26_DIS_ON_LED:
			case RDAT27_DIS_OFF_LED:
			case RDAT28_DIS_FLASH_LED:
			case RDAT30_DIS_LIGHTSHOW:
			case RDAT32_BAT_LOW_RLY_ON:
			case RDAT33_BAT_LOW_RLY_OFF:
			case RLOD:
			case RBAT:
			case RDIS:
			case RALM:
			case RSET:
				pString = &StrParams[1][0];
				if (strncmp((char *)pString, "^1", 2) == 0) { // 0 = strings match
				} else {
					if (strncmp((char *)pString, "^0", 2) == 0) { // 0 = strings match
					} else {
						upsData->comOkay = FALSE;
						#if DUAL_BOARD==FALSE								// single board
							upsBoss.comOkay = upsOne.comOkay;
						#endif
					}
				}
				break;
			default:
				upsData->upsComState = COM_IDLE;	// then wait for next communication on this ups
				break;
			}	// end of response case
		}
		else {	// checksum failed, message ignored
			upsData->comOkay = FALSE;			// error flag set
			// TODO - Communication dropout debug code
			if (upsData->upsNumber == 2) {
				P4OUT |= BIT7;  // set to zero
			}
		}
		upsData->upsComState = COM_IDLE;		// wait for next message, hope it's good
		// check for notification message
		if (upsData->notifyMsg == TRUE) {
			#if UPS_STATE_CONTROL==TRUE				// use router to control UPS operation
				ups_state_controller();			// call controller for fast response
			#endif
		}
		//sprintf((char *)eventStrTemp, "ups_com:RESPONSE Cmd=%s, response=%s, pass=%d, chksum msg=%d calc=%d for UPS %d",
        //  upsCmd[upsData->currentCmd],upsData->msgStr,upsData->comOkay,
        //  (int)upsData->checksumMsg,(int)upsData->checksumCalc,upsData->upsNumber);		
		ptmpchar = upsCmd[upsData->currentCmd];
		tmpival1 = upsData->comOkay;
		tmplval1 = upsData->checksumMsg; 
		tmplval2 = upsData->checksumCalc;
		tmpival2 = upsData->upsNumber;
		sprintf((char *)eventStrTemp, "ups_com:RESPONSE Cmd=%s, response=%s, pass=%d, chksum msg=%d calc=%d for UPS %d",
		        ptmpchar,upsData->msgStr,tmpival1,(int)tmplval1,(int)tmplval2,tmpival2);                        
		addEvent(eventStrTemp,5);
		break;
	default:
		upsData->comOkay = FALSE;
		upsData->upsComState = COM_IDLE;
		break;
	}	// end switch
	/*
	if (timer(upsData->timeOutStart, 1000)) {
		//upsData->comErrors++;			// track errors
	} else {
		if (--upsData->comErrors < 0) {								// successful, remove one error if any
			upsData->comErrors = 0;									// no less than 0 errors
		}
	}
	*/
	if (upsData->comErrors > 10) 
	{
		upsData->comOkay = FALSE;
		if (upsData->comErrors > 15) 			// limit maximum count so when problem solved it doesn't take a lot of time to clear errors
		{
			upsData->comErrors = 15;
		}
	} 
	else 
	{
		if (upsData->comErrors < 5) 			// zero check above where good messages reduce error count
		{
			upsData->comOkay = TRUE;
		}
	}
	//	__enable_interrupt();	// TODO - interrupt enable ups_com
	//DEBUG_PORT1_0;	// ####################
}

/*
This is used to find which command is being sent
*/

snmpParsePollEnumT selectSnmpCmd(volatile char *str){

	volatile int i;
	volatile snmpParsePollEnumT result;

	result = NO_SNMP_CMD;						// command not found

	// debug
	if (str[0] == 'O') {
		__no_operation();
	}
	// debug end

	for (i=0;i<SNMP_CMD_MAX;i++){
		if(strncmp((char *) str,(char *) &snmpParsePollCmd[i][0],3) == 0){	// returns 0 when equal, check first 3 char for cmd ex. "ST1"
			result = (snmpParsePollEnumT) i;	// if same string, return index of match
			// debug
			if (i==7) {
				__no_operation();
			}
			// debug end
		}
	}
	return result;
}


void init_snmp_com(volatile struct snmpDataStruct *parseType) {
	parseType->aLen[0] = parseType->aData[0] = NULL;
	parseType->pos = parseType->subPos = 0;
	parseType->snmpComState = SNMP_IDLE;
	parseType->lastSnmpComState = SNMP_RESPONSE;	// force state initialization
}

void snmp_com(volatile struct upsDataStrucT *upsData, volatile struct snmpDataStruct *parseType)
{
	static volatile char str[150], responseStr[150];
	static volatile int temp1, temp2, temp[10];
	static volatile long ltemp1;
	static volatile struct timeT rs485Delay;			// Delay timer for the RS485 port when used
	#if (defined CHARGER_MONSTER)						// Dual Model F charger
		volatile char *pString;						// string pointer to pass elements of string array
	#endif

        snmpStates_t tmpcState;
        char tmpchar;
        int tmpival1;
        float tmpfval1, tmpfval2, tmpfval3, tmpfval4, tmpfval5;
        long tmplval1, tmplval2;
                
	str[0] = responseStr[0] = NULL;
	switch (parseType->snmpComState) {
	case SNMP_BYPASS:                                              // in this mode do not call ups_com() for any ups.
	    //if (parseType->snmpComState != parseType->lastSnmpComState) 	// state entry code
		tmpcState = parseType->snmpComState;
		if (tmpcState != parseType->lastSnmpComState)
		{
			parseType->lastSnmpComState = parseType->snmpComState;
			snmpBypassUps = UPS1_PORT;					// select master
			usart_putstr("^RDAT23",upsOne.port);		// checksum off Master
			usart_putstr("^X",upsOne.port);			// exit SNMP on Model F Board
			#if (DUAL_BOARD == FALSE)
                usart_putstr("^RDAT23",upsTwo.port);		// checksum off Slave
                usart_putstr("^X",upsTwo.port);			// exit SNMP on Model F Board
			#endif
			usart_putchar(13,snmpBypassUps);			// pass on char to selected UPS
			usart_putchar(10,snmpBypassUps);			// pass on char to selected UPS
			usart_putchar(13,parseType->snmpPort);
			usart_putchar(10,parseType->snmpPort);
			usart_putstr("Router Software Version: ",parseType->snmpPort);
			usart_putstr(VERSION,parseType->snmpPort);
			usart_putchar(13,parseType->snmpPort);
			usart_putchar(10,parseType->snmpPort);
			usart_putstr("You are now connected directly to the Master Model F board",parseType->snmpPort);
			usart_putchar(13,parseType->snmpPort);
			usart_putchar(10,parseType->snmpPort);
			#if (DUAL_BOARD == FALSE)
                usart_putstr("1=Model F,0=Exit",parseType->snmpPort);
			#else
                usart_putstr("1=Master,2=Slave,0=Exit",parseType->snmpPort);
			#endif
			usart_putchar(13,parseType->snmpPort);
			usart_putchar(10,parseType->snmpPort);
		}  // end state entry code
		if (usart_rx_buffer_count(parseType->snmpPort)) {    // if char pending, get it
			parseType->snmpChar = usart_getchar(parseType->snmpPort);
			switch (parseType->snmpChar) {
			case '0':
				usart_putchar(13,parseType->snmpPort);
				usart_putchar(10,parseType->snmpPort);
				#ifdef COM_SNMP
				usart_putstr("You are now connected back to SNMP mode",parseType->snmpPort);
				#endif
				#ifdef COM_UPSILON
				usart_putstr("You are now connected back to Upsilon mode",parseType->snmpPort);
				#endif
				usart_putchar(13,parseType->snmpPort);
				usart_putchar(10,parseType->snmpPort);
				usart_putstr("^RDAT22",upsOne.port);	// checksum on Master
				usart_putstr("^RDAT22",upsTwo.port);	// checksum on Slave
				parseType->snmpComState = SNMP_IDLE;	// exit bypass
				break;
			case '1':
				snmpBypassUps = UPS1_PORT;				// select master
				usart_putstr("You are now connected directly to the Master Model F board",parseType->snmpPort);
				usart_putchar(13,parseType->snmpPort);
				usart_putchar(10,parseType->snmpPort);
				break;
			case '2':
			    #if (DUAL_BOARD == TRUE)
                    snmpBypassUps = UPS2_PORT;							// select slave
                    usart_putstr("You are now connected directly to the Slave Model F board",parseType->snmpPort);
                    usart_putchar(13,parseType->snmpPort);
                    usart_putchar(10,parseType->snmpPort);
			    #endif
				break;
			case 'h': case 'H':
                #if (DUAL_BOARD == FALSE)
                    usart_putstr("1=Model F,0=Exit",parseType->snmpPort);
                #else
                    usart_putstr("1=Master,2=Slave,0=Exit",parseType->snmpPort);
                #endif
				usart_putchar(13,parseType->snmpPort);
				usart_putchar(10,parseType->snmpPort);
				//usart_putchar(parseType->snmpChar,snmpBypassUps);	// pass on char to selected UPS
				tmpchar = parseType->snmpChar;
				usart_putchar(tmpchar,snmpBypassUps);
				break;
			default:
			    //usart_putchar(parseType->snmpChar,snmpBypassUps);	// pass on char to selected UPS, ups side in Main
				tmpchar = parseType->snmpChar;
				usart_putchar(tmpchar,snmpBypassUps);
				break;
			}
		}
		if (usart_rx_buffer_count(snmpBypassUps)) 					// Master selected
		{
			parseType->snmpChar = usart_getchar(snmpBypassUps);
			//usart_putchar(parseType->snmpChar,parseType->snmpPort);
			tmpchar = parseType->snmpChar;
			usart_putchar(tmpchar,parseType->snmpPort);
		}
		break;
	case SNMP_IDLE:	// Waiting for timed command, state change cmd or ups generated notification (button press, etc.)
	    //if (parseType->snmpComState != parseType->lastSnmpComState) {	// state entry code
		tmpcState = parseType->snmpComState;
		if (tmpcState != parseType->lastSnmpComState)
		{
			parseType->lastSnmpComState = parseType->snmpComState;
			parseType->timeOutStart = getTime();
		}  // end state entry code

		if (usart_rx_buffer_count(parseType->snmpPort)) {			// beginning of message
			parseType->snmpComState = SNMP_WAITING;
		}
		if (timer(parseType->timeOutStart, 5000)) {
			parseType->comOkay = FALSE;
		}
		break;
	case SNMP_WAITING:	// waiting for incoming msg to complete
	    //if (parseType->snmpComState != parseType->lastSnmpComState) {	// state entry code
		tmpcState = parseType->snmpComState;
		if (tmpcState != parseType->lastSnmpComState)
		{
			parseType->lastSnmpComState = parseType->snmpComState;

			str[0] = parseType->aLen[0] = parseType->aData[0] = NULL;
			parseType->pos = parseType->subPos = 0;
			parseType->phase = START;
			parseType->type = UNKNOWN;
		}  // end state entry code
		// second msg started before first one complete!
		if ((usart_peekchar(parseType->snmpPort) == '^') && (parseType->pos > 0)) {
			parseType->type = FAIL;
		} else {	// continue getting message
			if (usart_rx_buffer_count(parseType->snmpPort)) {									// if char pending, get it
				if (parseType->pos >= (SNMP_STR_MAX-2)) {					// str buffer almost full
					parseType->type = FAIL;								// dump message, start again
				} else {	// str buffer not full
					// get char, put in string, bump pointer
					//str[parseType->pos++] = parseType->snmpChar = usart_getchar(parseType->snmpPort);
					parseType->snmpChar = usart_getchar(parseType->snmpPort);
					tmpchar = parseType->snmpChar;
					str[parseType->pos++] = tmpchar;
                    switch (parseType->phase) 
                    {
                    case START:
                        // nothing to process
                        break;
                    case LEN:
                        // make sure it's an ascii code within chars '0' and '9' and buffer not full
                        if ((parseType->snmpChar >= '0') && (parseType->snmpChar <= '9') && (parseType->subPos < SNMP_STR2_MAX)) {
                            //parseType->aLen[parseType->subPos++] = parseType->snmpChar;
                            tmpchar = parseType->snmpChar;
                            parseType->aLen[parseType->subPos++] = tmpchar;
                            parseType->aLen[parseType->subPos] = NULL;						// terminate with null so string processes correctly atoi
                        } else {
                            parseType->type = FAIL;
                        }
                        break;
                    case DATA:
                        // make sure buffer not full
                        if (parseType->subPos < (SNMP_STR_MAX-2)) 							// at string max - 2, add char and null
                        {
                            //parseType->aData[parseType->subPos++] = parseType->snmpChar;	// accumulate data string
                            tmpchar = parseType->snmpChar;
                            parseType->aData[parseType->subPos++] = tmpchar;
                            parseType->aData[parseType->subPos] = NULL;						// easy to process, no matter when we stop
                            //if (parseType->subPos >= parseType->dataLen) {					// when position+1 = length, stop
                            tmpival1 = parseType->subPos;
                            if (tmpival1 >= parseType->dataLen)
                            {
                                str[0] = NULL;						// reset, used for part of response msg
                                #ifdef RS485LOCKOUT					// Wait 10mS if RS485LOCKOUT, because there is a timing issue on RS485 line
                                    parseType->snmpComState = SNMP2_RESPONSE_DELAY;			// Set state for the delay
                                #else
                                    parseType->snmpComState = SNMP_RESPONSE;				// message is done, now to respond to it
                                #endif
                            }
                        }
                        else 
                        {
                            parseType->type = FAIL;
                        }
                        break;
                    default:
                        break;
                    }
                    switch (parseType->pos) { // since this is bumped after getting char, 2 means 2nd char in snmpChar
                    case 2:	// check after we have two characters
                        switch (parseType->snmpChar) {
                        case 'P':
                            parseType->type = POLL;
                            break;
                        case 'S':
                            parseType->type = SET;
                            break;
                        case 'X':
                            parseType->snmpComState = SNMP_BYPASS;		// "^X" same as UPS to exit SNMP
                            break;
                        default:
                            parseType->type = FAIL;
                            break;
                        }
                        parseType->phase = LEN;	// next get number of char in message
                        parseType->subPos = 0;		// reset to fill sub field
                        break;
                    case 5:	// now aLen should have string with length number in it
                        parseType->dataLen = atoi((char *) parseType->aLen);
                        if ((parseType->dataLen > 0) && (parseType->dataLen < SNMP_STR_MAX)) {
                            parseType->phase = DATA;	// next get number of char in message
                            parseType->subPos = 0;		// reset to fill sub field
                        }
                        else {
                            parseType->type = FAIL;
                        }
                        break;
                    default:
                        break;
                    }
				}
			}
		}
		// message failed to conform to protocol in some way
		if (parseType->type == FAIL) {
			usart_putstr("^0",parseType->snmpPort);		// inform SNMP that message isn't understood
			parseType->snmpComState = SNMP_IDLE;	// start again with new msg, old one is ignored
		}
		else {	// message came through and was understood, reset comOkay
			parseType->comOkay = TRUE;
		}
		break;
	case SNMP2_RESPONSE_DELAY:
		// This delay before going to response will be present on all Thales SNMP communications, but not on other standard systems.
		//if (parseType->snmpComState != parseType->lastSnmpComState) {	// state entry code
		tmpcState = parseType->snmpComState;
		if (tmpcState != parseType->lastSnmpComState)
		{
			parseType->lastSnmpComState = parseType->snmpComState;		// Update the state
			rs485Delay = getTime ();									// Get the current time information
		}  // end state entry code
		if (timer (rs485Delay, 10)) {
			parseType->snmpComState = SNMP_RESPONSE;}					// message is done and delay is done, now to respond to it
		break;
	case SNMP_RESPONSE:	// Not used for timed updating, this is when question is first asked by Upsilon
	    //if (parseType->snmpComState != parseType->lastSnmpComState) {	// state entry code
		tmpcState = parseType->snmpComState;
		if (tmpcState != parseType->lastSnmpComState)
		{
			parseType->lastSnmpComState = parseType->snmpComState;
		}  // end state entry code
		parseType->snmpCmd = selectSnmpCmd(parseType->aData);	// get enum value related to string command
		#if defined COMM_MONITOR_PORT				// communication monitoring enabled
			usart_putstr("From SNMP:   ", COMM_MONITOR_PORT); // send string to indicated port
			usart_putstr(parseType->aData, COMM_MONITOR_PORT); // send string to indicated port
			usart_putstr("\r\n", COMM_MONITOR_PORT); // send string to indicated port
		#endif										// defined COMM_MONITOR_PORT
		// Note: all unimplemented data fields within the string represented with empty field ",,"
		// the last field has no comma at the end even if there are unimplemented items after it.
		switch(parseType->snmpCmd) {
		case AP1:		// Poll commands, some Set commands, variables implemented by us
			//				usart_putstr("^D0517,8,10,14,16,17,18,19,20,22,24,36,37,38,39,40,43,46",SNMP_PORT);
			strcpy((char *) str,"7,8,10,14,16,17,18,19,20,22,24,36,37,38,39,40,43,46");
			// sprintf will take the message, put ^D in front, then 3 digit length, then msg ex. "^D0052,230"
			parseType->dataLen = strlen((char *) str);
			strcpy((char *) responseStr,"^D");
			strcat((char *) responseStr,itoa3(parseType->dataLen));
			strcat((char *) responseStr,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case AP2:
			//				usart_putstr("^D07447,51,54,55,56,58,59,60,61,62,63,65,68,69,72,73,76,77,80,82,83,84,85,88,89",SNMP_PORT);
			strcpy((char *) str,"47,51,54,55,56,58,59,60,61,62,63,65,68,69,72,73,76,77,80,82,83,84,85,88,89");
			// sprintf will take the message, put ^D in front, then 3 digit length, then msg ex. "^D0052,230"
			parseType->dataLen = strlen((char *) str);
			sprintf((char *) responseStr,"^D%03d%s", parseType->dataLen,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case ATR:		// Autostart Poll and Set
			if (upsData->bank1 & BIT1) {	// see if autoStart bit is set
				strcpy((char *) str,"1");
			}
			else {
				strcpy((char *) str,"2");
			}
			// sprintf will take the message, put ^D in front, then 3 digit length, then msg ex. "^D0052,230"
			parseType->dataLen = strlen((char *) str);
			strcpy((char *) responseStr,"^D001");
			strcat((char *) responseStr,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case MAN:		// Manufacturer
			strcpy((char *) str,(char *) upsData->man);
			// sprintf will take the message, put ^D in front, then 3 digit length, then msg ex. "^D0052,230"
			parseType->dataLen = strlen((char *) str);
			//	sprintf(responseStr,"^D%03d%s", parseType->dataLen,str);
			strcpy((char *) responseStr,"^D");
			strcat((char *) responseStr,itoa3(parseType->dataLen));
			strcat((char *) responseStr,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case MOD:		// Model
			//strcpy((char *) str,(char *) upsData->model);
			strcpy((char *) str,UPS_MODEL);
			parseType->dataLen = strlen((char *) str);
			//	sprintf(responseStr,"^D%03d%s",parseType->dataLen, str);
			strcpy((char *) responseStr,"^D");
			strcat((char *) responseStr,itoa3(parseType->dataLen));
			strcat((char *) responseStr,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case NOM:	// Nominal values for parameters
			//							Vin,Fin,Vo, Fo, VA,	 Po,  LoBatTime,Alm,LoV,HiV (,BatInst,BatLife)
			//	strcpy(str,"230,600,230,600,5000,4000,1,2,170,280");
			strcpy((char *) str,itoa((int) upsBoss.voltInNom));
			strcat((char *) str,",");
			strcat((char *) str,itoa((int) upsBoss.freqInNom*10));
			strcat((char *) str,",");
			strcat((char *) str,itoa((int) upsBoss.voltOutNom));
			strcat((char *) str,",");
			strcat((char *) str,itoa((int) upsBoss.freqOutNom*10));
			strcat((char *) str,",");
			strcat((char *) str,itoa((int) upsBoss.vaOutNom));
			strcat((char *) str,",");
			strcat((char *) str,itoa((int) upsBoss.powOutNom));
			#if (defined SNMP_NOM_LOW_BAT_TIME)
				strcat((char *) str,SNMP_NOM_LOW_BAT_TIME);
			#else
				strcat((char *) str,",1");
			#endif
			// Audible Alarm Status 0=disabled, 1=Enabled, 2=muted, 3=low battery
			strcat((char *) str,",1");
			// Override low/high input voltage transfer in configuration file
			#if (defined SNMP_NOM_INPUT_TRANSFER)
				strcat((char *) str,SNMP_NOM_INPUT_TRANSFER);
			#else
				strcat((char *) str,",100,140");
			#endif
			parseType->dataLen = strlen((char *) str);
			//	sprintf(responseStr,"^D%03d%s", parseType->dataLen,str);
			strcpy((char *) responseStr,"^D");
			strcat((char *) responseStr,itoa3(parseType->dataLen));
			strcat((char *) responseStr,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case PSD:		// Timed Shutdown, Set command unique
			if (parseType->type == POLL) {
				strcpy((char *) str,itoa(upsBoss.secShutdownDelay));
				parseType->dataLen = strlen((char *) str);
				strcpy((char *) responseStr,"^D");
				strcat((char *) responseStr,itoa3(parseType->dataLen));
				strcat((char *) responseStr,(char *) str);
				usart_putstr(responseStr,parseType->snmpPort);
			}
			else if (parseType->type == SET) {					// UPS timed shutdown command
				#ifdef COM_SNMP
					scanSnmpParams(pSnmp);						// scan for parameters of command
				#endif
				#ifdef COM_UPSILON
					scanSnmpParams(pUpsilon);					// scan for parameters of command
				#endif
				ltemp1 = atol((char *) &StrParams[0][0]);
				if (ltemp1 != -1) {								// if it isn't an abort command
					upsBoss.secShutdownDelay = ltemp1;			// process in UPS state machine
					delayedShutdownTime = getTime();			// start timer
					delayedShutdown = TRUE;					// activate delayed startup
				} else {
					delayedShutdown = FALSE;					// cancel delayed startup
				}
				strcpy((char *) responseStr,"^1");
				//responseStr[2] = NULL;
				usart_putstr(responseStr,parseType->snmpPort);
			}
			parseType->snmpComState = SNMP_IDLE;
			break;
		case STD:		// Timed Startup, Set command unique
			if (parseType->type == POLL) {
				strcpy((char *) str,itoa(upsBoss.secStartupDelay));
				parseType->dataLen = strlen((char *) str);
				strcpy((char *) responseStr,"^D");
				strcat((char *) responseStr,itoa3(parseType->dataLen));
				strcat((char *) responseStr,(char *) str);
				usart_putstr(responseStr,parseType->snmpPort);
			}
			else if (parseType->type == SET) {					// UPS timed shutdown command
				#ifdef COM_SNMP
					scanSnmpParams(pSnmp);						// scan for parameters of command
				#endif
				#ifdef COM_UPSILON
					scanSnmpParams(pUpsilon);					// scan for parameters of command
				#endif
				ltemp1 = atol((char *) &StrParams[0][0]);
				if (ltemp1 != -1) {								// if it isn't an abort command
					upsBoss.secStartupDelay = ltemp1;			// process in UPS state machine
					delayedStartupTime = getTime();			// start timer
					delayedStartup = TRUE;						// activate delayed startup
				} else {
					delayedStartup = FALSE;					// cancel delayed startup
				}
				strcpy((char *) responseStr,"^1");
				//responseStr[2] = NULL;
				usart_putstr(responseStr,parseType->snmpPort);
			}
			parseType->snmpComState = SNMP_IDLE;
			break;
		case ST1:	// Battery Status
			if (parseType->parser == SNMP) {
				temp1 = (int) (upsData->batChgPct);
			}
			else {
				temp1 = (int) fMin(((upsData->batChgPct + 0.5)*10), 100.0);
			}
			/*
			sprintf(str,"%d,%d,%d,%05d,%d,%d,%d,%d,%d",
				upsData->batCond,upsData->batSts,upsData->batChgMode,
				(int) upsData->secOnBat,
				(int) upsData->estSecBat,
				temp1,
				(int) ((upsData->voltBat + 0.5)*10),
				(int) ((upsData->ampBat + 0.5)*10),
				(int) (upsData->tAmb+0.5));
			*/
			// if Model F sent over voltage (3), send fault (2), otherwise send number for condition
			#if (!defined SNMP_MIMIC)
				#if (!defined THALES_3KVA)
					switch (upsData->batCond) {
					case NORMAL:
						temp2 = 0;
						break;
					case WARN:
						temp2 = 1;
						break;
					case FAULT:
						temp2 = 2;
						break;
					case OVER_VOLTAGE:
						temp2 = 3;
						break;
					default:
						temp2 = 2;								// fault
						break;
					}
				#else		// defined THALES_3KVA
					// If Charger has power but not communicating, indicate battery fault
					if ((upsTwo.comOkay == FALSE) && (upsBoss.upsState != UPS_ON_BAT)) {
						temp2 = 2;									// fault
					} else {										// Otherwise report last state reported by charger
						switch (upsData->batCond) {				// substituted in ups_com() with upsThree (charger) data through RS485
						case NORMAL:
							temp2 = 0;
							break;
						case WARN:
							temp2 = 1;
							break;
						case FAULT:
							temp2 = 2;
							break;
						case OVER_VOLTAGE:
							temp2 = 3;
							break;
						default:
							temp2 = 2;								// fault
							break;
						}
					}
				#endif		// !defined THALES_3KVA
			#else	// defined SNMP_MIMIC
				// If Charger has power but not communicating, indicate battery fault
				if ((charger.comOkay == FALSE) && (upsBoss.upsState != UPS_ON_BAT)) 
				{
					temp2 = 2;										// fault
				} 
				else												// Otherwise report last state reported by charger 
				{
									// substituted in ups_com() with upsThree (charger) data through RS485
					switch (upsData->batCond) 
					{
					case NORMAL:
						temp2 = 0;
						break;
					case WARN:
						temp2 = 1;
						break;
					case FAULT:
						temp2 = 2;
						break;
					case OVER_VOLTAGE:
						temp2 = 3;
						break;
					default:
						temp2 = 2;								// fault
						break;
					}
				}
			#endif	// !defined SNMP_MIMIC
			strcpy((char *) str,itoa(temp2));				// ST1-1, Battery Condition
			strcat((char *) str,",");
			strcat((char *) str,itoa(upsBoss.batSts));		// ST1-2, Battery Status
			strcat((char *) str,",");
			// in ups chg_status 1=fast, 2=slow, 3=off, reports => 
			// Battery Charge, 0=float, 1=Charging, 2=Resting, 3=Discharging
			strcat((char *) str,itoa(upsData->chgModeSnmp));// ST1-3, Battery Charge
			strcat((char *) str,",");
			strcat((char *) str,itoa((int) upsData->secOnBat));// Seconds since UPS switched to Battery
			strcat((char *) str,",");
			strcat((char *) str,itoa( (int) ((upsData->estSecBat/60.0) + 0.5)));// Estimated time left on Battery
			strcat((char *) str,",");
			strcat((char *) str,itoa(temp1));// Percent of Battery charge remaining
			strcat((char *) str,",");
			strcat((char *) str,itoa((int) ((upsData->voltBat + 0.5)*10)));// Battery voltage
			strcat((char *) str,",");
			#if !defined UCLASS_FA10002
				if ((upsBoss.dcMode == AUTO_ON) || (upsBoss.dcMode == MANUAL_ON)) {
                   strcat((char *) str,itoa((int) ((upsData->ampBat + 0.5)*10)));// Battery current
				} else {
					strcat((char *) str,"0");		// not on batteries, zero current
				}
			#else												// defined UCLASS_FA10002
				strcat((char *) str,itoa((int) (upsData->ampBat*10)));// Battery current
			#endif
			strcat((char *) str,",");
			strcat((char *) str,itoa((int) (upsData->tAmb+0.5)));// Battery temperature
			parseType->dataLen = strlen((char *) str);
			//	sprintf(responseStr,"^D%03d%s", parseType->dataLen,str);
			strcpy((char *) responseStr,"^D");
			strcat((char *) responseStr,itoa3(parseType->dataLen));
			strcat((char *) responseStr,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case ST2:
			// first param empty, input lines bad
			//	sprintf(str,",1,%d,%d",
			//		(int) (upsData->freqIn*10),		// average 60.3=>603
			//		(int) (upsData->voltIn*10));		// output in series so add together
			#if (!defined NOV_CAT_J)
				strcpy((char *) str,",1,");// Number of input lines
			#else										// #if defined NOV_CAT_J
				// use number of input lines to report the two Bender status
				// 0=No fault, 1=input ground fault, 2=output impedance fault, 3=both faulted
				// status reported by Master UPS (upsOne)
				strcpy((char *) str,",");	// item delimiter
				strcat((char *) str,itoa((int) upsOne.linesNumIn));// Number of input lines
				strcat((char *) str,",");	// item delimiter
			#endif										// #if !defined NOV_CAT_J
			strcat((char *) str,itoa((int) (upsOne.freqIn*10)));
			strcat((char *) str,",");
			#ifdef CRYSTAL_OLD							// Original Crystal can't sense input voltage
				if (upsBoss.upsState == UPS_ON_UTIL) {
					upsBoss.voltIn = upsOne.voltIn = upsTwo.voltIn = UPS_VOLTINNOM;
					upsBoss.freqIn = upsOne.freqIn = upsTwo.freqIn = UPS_FREQINNOM;
				} else {
					upsBoss.voltIn = upsOne.voltIn = upsTwo.voltIn = 0.0;
					upsBoss.freqIn = upsOne.freqIn = upsTwo.freqIn = 0.0;
				}
			#endif
			strcat((char *) str,itoa((int) (upsOne.voltIn*10)));
			strcat((char *) str,",,,");											// input current and power
			strcat((char *) str,itoa((int) (upsTwo.freqIn*10)));
			strcat((char *) str,",");
			strcat((char *) str,itoa((int) (upsTwo.voltIn*10)));
			parseType->dataLen = strlen((char *) str);
			strcpy((char *) responseStr,"^D");
			strcat((char *) responseStr,itoa3(parseType->dataLen));
			strcat((char *) responseStr,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case ST3:
			// Output Source: UPS on batteries, Utility, Bypass...
			if (parseType->parser == SNMP) 
			{
				if (upsBoss.bypassMode == ON)			    // if either board on bypass 
				{
					temp1 = 0;								// don't report load on bypass
				} 
				else 
				{
                    #if (!defined ZERO_POWER_REPORTING_WATTS)
					temp1 = (int)((upsData->ampOut + 0.5)*10);
                    #else   // #if (defined ZERO_POWER_REPORTING_WATTS)
                    if (upsData->powOut >= ZERO_POWER_REPORTING_WATTS)
                    {
                        temp1 = (int)((upsData->ampOut + 0.5)*10);
                    }
                    else
                    {
                        temp1 = (int) 0;
                    }
                    #endif  // #if (!defined ZERO_POWER_REPORTING_WATTS)
				}
			} 
			else // if (parseType->parser != SNMP) UPSILON
			{
				//temp1 = (int) ((((upsData->powOut/UPS_POWOUTNOM) * 100) + 0.5)*10);
				if (upsBoss.bypassMode == ON)			    // if either board on bypass
				{
					temp1 = 0;								// don't report load on bypass
				} 
				else 
				{
                    #if (!defined ZERO_POWER_REPORTING_WATTS)
					temp1 = (int) (upsData->loadPctOut + 0.5)*10;
                    #else   // #if (defined ZERO_POWER_REPORTING_WATTS)
                    if (upsData->powOut >= ZERO_POWER_REPORTING_WATTS)
                    {
                        temp1 = (int) (upsData->loadPctOut + 0.5)*10;
                    }
                    else
                    {
                        temp1 = (int) 0;
                    }
                    #endif  // #if (!defined ZERO_POWER_REPORTING_WATTS)
				}
			}
			if (parseType->parser == SNMP) 
			{
				//temp2 = (int) (((upsData->powOut/UPS_POWOUTNOM) * 100) + 0.5);
				#ifdef SNMP_MIMIC

                #if (!defined ZERO_POWER_REPORTING_WATTS)
                temp2 = (int) (upsData->loadPctOut + 0.5)*10;
                #else   // #if (defined ZERO_POWER_REPORTING_WATTS)
                if (upsData->powOut >= ZERO_POWER_REPORTING_WATTS)
                {
                    temp2 = (int) (upsData->loadPctOut + 0.5)*10;
                }
                else
                {
                    temp2 = (int) 0;
                }
                #endif  // #if (!defined ZERO_POWER_REPORTING_WATTS)

				#else   // #ifndef SNMP_MIMIC - if either board on bypass
                
                if (upsBoss.bypassMode == ON) 
                {
                    temp2 = 0;							// don't report load on bypass
                } 
                else 
                {

                    #if (!defined ZERO_POWER_REPORTING_WATTS)
                    temp2 = (int) (upsData->loadPctOut + 0.5)*10;
                    #else   // #if (defined ZERO_POWER_REPORTING_WATTS)
                    if (upsData->powOut >= ZERO_POWER_REPORTING_WATTS)
                    {
                        temp2 = (int) (upsData->loadPctOut + 0.5);
                    }
                    else
                    {
                        temp2 = (int) 0;
                    }
                    #endif  // #if (!defined ZERO_POWER_REPORTING_WATTS)

                }

				#endif  // #ifdef SNMP_MIMIC
			} 
			else // if (parseType->parser != SNMP) UPSILON
			{
				if (upsBoss.bypassMode == ON) 			    // if either board on bypass
				{
					temp2 = 0;								// don't report load on bypass
				} 
				else 
				{
					temp2 = (int)(upsData->ampOut + 0.5);
				}
			}
			switch (upsBoss.upsState) 
			{
			case UPS_OFF:
				// UPS on "Other" then just drops down to UPS_INIT, unless only master has control
				// and then if there is overload bypass. The unit is in bypass but upsState is UPS_OFF
				// Note: this comment is no longer accurate and somewhat confusing. TODO:
				// Note: added the "#else" for ComTech since overload causes unit to go on bypass
				//     and to UPS_OFF state, SNMP was not reporting the Bypass state
				#if defined DUAL_BOARD_MASTER_BYPASS_CONTROL
					if (upsBoss.bypassMode == ON) 
					{
						strcpy((char *) str, "2");  // in bypass
					} 
					else 
					{
						strcpy((char *) str, "5");  // "Other"
					}
				#else // !defined DUAL_BOARD_MASTER_BYPASS_CONTROL
                    if (upsBoss.bypassMode == ON) 
                    {
                        strcpy((char *) str, "2");  // in bypass
                    } 
                    else 
                    {
                        //TODO - CANES 6.6KVA Output Source debug 
                        strcpy((char *) str, "5");  // "Other"
                    }
				#endif
				break;
			case UPS_INIT:                          // UPS on "Other"
				strcpy((char *) str, "5");
				break;
			case UPS_ON_BAT:
			case UPS_SHUTDOWN:
				strcpy((char *) str, "1");		    // UPS on battery
				break;
			case UPS_ON_UTIL:
				if (upsBoss.bypassMode == ON)       // ON, OFF or AUTO
				{
					strcpy((char *) str, "2");	    // UPS on bypass
				} 
				else if ((upsOne.dcMode == AUTO_ON) || (upsTwo.dcMode == AUTO_ON)) 
				{
					strcpy((char *) str, "1");	    // UPS on battery (shutting down)
				} 
				else 
				{
					strcpy((char *) str, "0");	    // UPS on utility "normal"
				}
				break;
			#ifdef BYPASS_RECOVER                   // Will try to recover from overload bypass
			case UPS_BYPASS:
				strcpy((char *) str, "2");		    // UPS on bypass
				break;
			#endif
			case UPS_FAULT:
				strcpy((char *) str, "2");		    // UPS on bypass
				break;
			}
			//	    output source, Fout, num lines out, Vout, Iout, Pout, Load%out
			//	sprintf(responseStr,"%s,%d,1,%d,%d,%d,%d",
			//		str,(int)(upsData->freqOut*10),
			//		(int)((upsData->voltOut + 0.5)*10),
			//		temp1,(int)(upsData->powOut + 0.5),temp2);
			// ST3-1 Output Source
			strcpy((char *) responseStr,(char *) str);			// output source, set above
			strcat((char *) responseStr,",");
			// ST3-2 Output Frequency
			strcat((char *) responseStr,itoa((int)(upsData->freqOut*10)));
			// ST3-3 Output number of lines
			strcat((char *) responseStr,",1,");
			// ST3-4 Output Voltage 1
			strcat((char *) responseStr,itoa((int)((upsData->voltOut + 0.5)*10)));
			strcat((char *) responseStr,",");
			// ST3-5 Output Current 1
			strcat((char *) responseStr,itoa(temp1));
			strcat((char *) responseStr,",");
			// ST3-6 Output Power 1
			if (upsBoss.bypassMode == ON)			            // if either board on bypass
			{
				strcat((char *) responseStr,itoa((int) 0 ));	// no power out
			} 
			else 
			{
                #if (!defined ZERO_POWER_REPORTING_WATTS)
				strcat((char *) responseStr,itoa((int)(upsData->powOut + 0.5)));
                #else   // #if (defined ZERO_POWER_REPORTING_WATTS)
                if (upsData->powOut >= ZERO_POWER_REPORTING_WATTS)
                {
				    strcat((char *) responseStr,itoa((int)(upsData->powOut + 0.5)));
                }
                else
                {
				    strcat((char *) responseStr,itoa(0));
                }
                #endif  // #if (!defined ZERO_POWER_REPORTING_WATTS)
			}
			strcat((char *) responseStr,",");
			// ST3-7 Output Load 1
			strcat((char *) responseStr,itoa(temp2));
			strcpy((char *) str,(char *) responseStr);
			// CORES2 extended MIB
			/*
			strcat(responseStr,",");
			// ST3-8 Output Voltage 2
			strcat(responseStr,itoa((int)((upsOne.voltOut + 0.5)*10)));
			strcat(responseStr,",");
			// ST3-9 Output Current 2
			strcat(responseStr,itoa((int)((upsOne.ampOut + 0.5)*10)));
			strcat(responseStr,",");
			// ST3-10 Output Power 2
			strcat(responseStr,itoa((int)(upsOne.powOut + 0.5)));
			strcat(responseStr,",");
			// ST3-11 Output Load 2
			strcat(responseStr,itoa((int)((((upsOne.powOut/2400.0) * 100) + 0.5)*10)));
			strcat(responseStr,",");
			// ST3-8 Output Voltage 3
			strcat(responseStr,itoa((int)((upsTwo.voltOut + 0.5)*10)));
			strcat(responseStr,",");
			// ST3-9 Output Current 2
			strcat(responseStr,itoa((int)((upsTwo.ampOut + 0.5)*10)));
			strcat(responseStr,",");
			// ST3-10 Output Power 2
			strcat(responseStr,itoa((int)(upsTwo.powOut + 0.5)));
			strcat(responseStr,",");
			// ST3-11 Output Load 2
			strcat(responseStr,itoa((int)((((upsTwo.powOut/2400.0) * 100) + 0.5)*10)));
			*/			
			parseType->dataLen = strlen((char *) str);
			strcpy((char *) responseStr,"^D");
			strcat((char *) responseStr,itoa3(parseType->dataLen));
			strcat((char *) responseStr,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case ST4:	// Bypass information
			//	strcpy((char *) responseStr, "^D001,");	// send message that feature is not supported
			// message above was used to prevent a problem with the SNMP
			// now using the unknown command response
			strcpy((char *) responseStr, "^D001,");	// send message that feature is supported but send nothing
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case ST5:	// Alarms
			if (upsBoss.tAmbMode == ON_ALARM) { 			// Temperature alarm, ST5-1
				strcpy((char *)str, "1,");
			}
			else {
				strcpy((char *)str, "0,");
			}
			if ( (upsBoss.dcMode == AUTO_ON)				// Input Bad Alm = on battery, ST5-2
			        || (upsData->dcMode == MANUAL_ON) ) {
				strcat((char *) str, "1,");
			}
			else {
				strcat((char *) str, "0,");
			}
			if (upsBoss.invFaultAlm == ON_ALARM) {		// Output Bad Alm = Inverter fault, ST5-3
				strcat((char *) str, "1,");
			}
			else {
				strcat((char *) str, "0,");
			}
			if ((upsBoss.loadPctOut >= 105.0) || (upsTwo.loadPctOut >= 105.0)) {
				strcat((char *) str, "1,");		// Overload Alm = Inverter fault, ST5-4
			}
			else {
				strcat((char *) str, "0,");
			}
			strcat((char *) str, ",");			// Bypass bad alarm, not implemented, ST5-5
			if ((upsBoss.invMode == AUTO_OFF) || (upsBoss.invMode == MANUAL_OFF)) {
				strcat((char *) str, "1");		// Output Bad Alm = Inverter off, ST5-6
			}
			else {
				strcat((char *) str, "0");
			}
			strcat((char *) str, ",,");	// UPS Shutdown, not implemented, ST5-7
			#if (!defined SNMP_MIMIC)
				#if (!defined THALES_3KVA)
					if (upsBoss.batCond != FAULT) {
						strcat((char *) str, "0");	// Charger Failure ST5-8, charger communication okay
					} else {
						strcat((char *) str, "1");	// Charger Failure, loss of charger communication
					}
				#else	// defined THALES_3KVA
					if ( ((upsTwo.comOkay == TRUE) && (upsBoss.batCond != FAULT))	// charger on, no fault
						|| ((upsBoss.upsState == UPS_ON_BAT) && (	upsBoss.batCond != FAULT)) ) 					// or charger off with no faults
					{
						strcat((char *) str, "0");	// Charger Failure ST5-8, charger communication okay
					} 
					else 
					{
						if ((upsTwo.comOkay == TRUE) && (upsBoss.batCond == FAULT)) 
						{
							strcat((char *) str, "1");	// Charger Failure, charger communication okay
						} 
						else 
						{
							if ((upsTwo.comOkay == FALSE) && (upsBoss.upsState != UPS_ON_BAT)) 
							{
								strcat((char *) str, "1");	// Charger Failure, loss of charger communication with utility on
							}
						}
					}
				#endif	// !defined THALES_3KVA
			#else	// defined SNMP_MIMIC
				if ((charger.comOkay == TRUE) && (	upsBoss.batCond != FAULT)) {
					strcat((char *) str, "0");	// Charger Failure ST5-8, charger communication okay
				} else {
					strcat((char *) str, "1");	// Charger Failure, loss of charger communication
				}
			#endif	// !defined SNMP_MIMIC
			/*
			strcat((char *) str, ",");				// UPS Shutdown, not implemented, ST5-7
			strcat((char *) str, ",");				// Charger Failure, not implemented, ST5-8
			strcat((char *) str, ",");				// System Off, not implemented, ST5-9
			if (upsData->fanFail == TRUE) {					// Cabinet Fan Status, ST5-10
				strcat((char *) str, "1,");
			}
			else {
				strcat((char *) str, "0,");
			}
			strcat((char *) str, ",");				// Fuse Failure, not implemented, ST5-11
			if ( (powerConditioner1Fault == TRUE) || 			// General Fault, ST5-12
				(powerConditioner2Fault == TRUE) ) {
				strcat((char *) str, "1,");
			} 
			else {
				strcat((char *) str, "0,");
			}
			strcat((char *) str, ",");				// Awaiting Power, not implemented, ST5-13
			strcat((char *) str, ",");				// Shutdown Pending, not implemented, ST5-14
			strcat((char *) str, ",");				// Shutdown Imminent, not implemented, ST5-15
			strcat((char *) str, "0,");				// PDU Status, ST5-16
			if ( (upsOne.invFaultAlm == ON_ALARM) || 	// Power Conditioner1 Status, ST5-17
				(upsOne.tAmbMode == ON_ALARM) ||
			//	(upsOne.tSinkMode == ON_ALARM) ||
				(powerConditioner1Fault == TRUE) ){
				strcat((char *) str, "1,");
			}
			else {
				strcat((char *) str, "0,");
			}
			if ( (upsTwo.invFaultAlm == ON_ALARM) || 	// Power Conditioner2 Status, ST5-18
				(upsTwo.tAmbMode == ON_ALARM) ||
				//	(upsTwo.tSinkMode == ON_ALARM) ||
				(powerConditioner2Fault == TRUE) ){
				strcat((char *) str, "1,");
			}
			else {
				strcat((char *) str, "0,");
			}
			strcat((char *) str,itoa(upsOne.batSts));// Battery1 Status, ST5-19
			strcat((char *) str, ",");
			strcat((char *) str,itoa(upsTwo.batSts));// Battery2 Status, ST5-20
			strcat((char *) str, ",");
			if (upsData->battleshort == TRUE) {		// Cabinet Battleshort, ST5-21
				strcat((char *) str, "1");
			}
			else {
				strcat((char *) str, "0");
			}
			*/
			parseType->dataLen = strlen((char *) str);
			strcpy((char *) responseStr,"^D");
			strcat((char *) responseStr,itoa3(parseType->dataLen));
			strcat((char *) responseStr,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case SDA:		// Poll and Set, shutdown type 1=UPS System
			usart_putstr("^D0011",parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case STR:		// Test Results Summary, Summary = 1 (passed), results detail (not implemented)
			usart_putstr("^D0011",parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case UBR:		// Poll and Set, Baud rate 1200, 2400, 4800, 9600, 19200
			strcpy((char *) str, "9600");
			parseType->dataLen = strlen((char *) str);
			//	sprintf(responseStr,"^D%03d%s", parseType->dataLen,str);
			strcpy((char *) responseStr,"^D");
			strcat((char *) responseStr,itoa3(parseType->dataLen));
			strcat((char *) responseStr,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case UID:		// UPS Identification, if you had more than one you could number them 1-xx
			//usart_putstr("^D00200",SNMP_PORT);
			if (parseType->type == POLL) {
				if (!iRange(upsBoss.snmpUid,0,999)) {			// if value not in range
					upsBoss.snmpUid = 0;						// set to default
				}
				strcpy((char *) str,itoa(upsBoss.snmpUid));
				parseType->dataLen = strlen((char *) str);
				strcpy((char *) responseStr,"^D");
				strcat((char *) responseStr,itoa3(parseType->dataLen));
				strcat((char *) responseStr,(char *) str);
				usart_putstr(responseStr,parseType->snmpPort);
			}
			else if (parseType->type == SET) {					// UPS timed shutdown command
				scanSnmpParams(pSnmp);							// scan for parameters of command
				ltemp1 = atol((char *) &StrParams[0][0]);
				upsBoss.snmpUid = (int) ltemp1;				// process in UPS state machine
				if (!iRange (upsBoss.snmpUid, 0, 999)) {		// If value not in valid range
					upsBoss.snmpUid = 0;						// then set to default
				}
				strcpy((char *) responseStr,"^1");
				usart_putstr(responseStr,parseType->snmpPort);
				flashPointer = (unsigned int *) 0x1880;			// Start of Information Flash Segment C *** Moved to avoid conflict with Battery Joules ***
				flashPointer += 2;								// Keep the offset from the older system but look in the new segment
				__disable_interrupt();					  		// 5xx Workaround: Disable global
				// interrupt while erasing. Re-Enable
				// GIE if needed
				FCTL3 = FWKEY;							  		// Clear Lock bit
				FCTL1 = FWKEY+ERASE;					  		// Set Erase bit
				*(unsigned int *)flashPointer = 0;				// Dummy write to erase Flash seg
				//FCTL1 = FWKEY+BLKWRT;						// Enable long-word write
				FCTL1 = FWKEY+WRT;						  		// Set WRT bit for write operation
				*flashPointer = upsBoss.snmpUid;				// Write UID
				//*(unsigned long *)Flash_ptrD = value;	  	// Write to Flash
				FCTL1 = FWKEY;							  		// Clear WRT bit
				FCTL3 = FWKEY+LOCK;						  		// Set LOCK bit
				__enable_interrupt();					 		// 5xx Workaround: Disable global
			}
			parseType->snmpComState = SNMP_IDLE;
			break;
		case VER:
			if (strlen((char *) upsData->verSoftware) < (SNMP_STR_MAX-1)) {
				strcpy((char *) str, (char *) upsData->verSoftware);
			} else {
				strcpy((char *) str, "Unknown, string length error");
			}
			//	strcat(str, ", PC1 ");
			//	strcat(str, upsOne.verSoftware);
			//	strcat(str, ", PC2 ");
			//	strcat(str, upsTwo.verSoftware);
			parseType->dataLen = strlen((char *) str);
			//	sprintf(responseStr,"^D%03d%s", parseType->dataLen,str);
			strcpy((char *) responseStr,"^D");
			strcat((char *) responseStr,itoa3(parseType->dataLen));
			strcat((char *) responseStr,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case XD1:
			temp[1] = (int) ((upsData->voltBus + 0.5)*10);	// average Vbus
			temp[2] = (int) (upsData->tSink + 0.5)	;			// max tSink
			temp[4] = (int) ((upsData->voltSupply + 0.5)*10);	// max Vsupply
			//	sprintf(str,"%1.3f,%d,%d,%1.2f,%d",((upsData->ampChg + 0.5)*10),temp[1],temp[2],upsData->pfOut,temp[4]);
			strcpy((char *) str,itoa((int) ((upsData->ampChg + 0.5)*10)));
			strcat((char *) str,",");
			strcat((char *) str,itoa(temp[1]));
			strcat((char *) str,",");
			strcat((char *) str,itoa(temp[2]));
			strcat((char *) str,",");
			strcat((char *) str,itoa((int) upsData->pfOut));
			strcat((char *) str,",");
			strcat((char *) str,itoa(temp[4]));
			parseType->dataLen = strlen((char *) str);
			//	sprintf(responseStr,"^D%03d%s", parseType->dataLen,str);
			strcpy((char *) responseStr,"^D");
			strcat((char *) responseStr,itoa3(parseType->dataLen));
			strcat((char *) responseStr,(char *) str);
			usart_putstr(responseStr,parseType->snmpPort);
			parseType->snmpComState = SNMP_IDLE;
			break;
		case XD2:
			/*	// code from charger_com()
				//pString = &StrParams [0][0];
				//upsOne.voltBat = atof((char*) pString);
				pString = &StrParams [1][0];
				upsOne.ampChg = atof((char*) pString);
				//pString = &StrParams [2][0];
				//upsOne.ampBat = atof((char*) pString);
				//pString = &StrParams [3][0];
				//upsTwo.voltBat = atof((char*) pString);
				pString = &StrParams [4][0];
				upsTwo.ampChg = atof((char*) pString);
				//pString = &StrParams [5][0];
				//upsTwo.ampBat = atof((char*) pString);
			*/
			if (parseType->type == POLL) {
			    //sprintf((char *) str, "%1.2f,%1.3f,%1.2f,%1.2f,%1.3f,%1.2f,%ld,%ld"
			    //,upsOne.voltBat,upsOne.ampChg,upsOne.ampBat,upsTwo.voltBat,upsTwo.ampChg,upsTwo.ampBat
			    //,upsOne.batJoule,upsTwo.batJoule);
				tmpfval1 = upsOne.ampChg;
				tmpfval2 = upsOne.ampBat;
				tmpfval3 = upsTwo.voltBat;
				tmpfval4 = upsTwo.ampChg;
				tmpfval5 = upsTwo.ampBat;
				tmplval1 = upsOne.batJoule;
				tmplval2 = upsTwo.batJoule;
				sprintf((char *) str, "%1.2f,%1.3f,%1.2f,%1.2f,%1.3f,%1.2f,%ld,%ld",
  								upsOne.voltBat,tmpfval1, tmpfval2, tmpfval3, tmpfval4, tmpfval5, tmplval1, tmplval2);    

				parseType->dataLen = strlen((char *) str);
				strcpy((char *) responseStr,"^D");
				strcat((char *) responseStr,itoa3(parseType->dataLen));
				strcat((char *) responseStr,(char *) str);
				usart_putstr(responseStr,parseType->snmpPort);
			} else if (parseType->type == SET) {	
				scanSnmpParams(pSnmp);							// scan for parameters of command
				//pString = &StrParams [0][0];
				//upsOne.voltBat = atof((char*) pString);
				//pString = &StrParams [1][0];
				//upsOne.ampChg = atof((char*) pString);
				//pString = &StrParams [2][0];
				//upsOne.ampBat = atof((char*) pString);
				//pString = &StrParams [3][0];
				//upsTwo.voltBat = atof((char*) pString);
				//pString = &StrParams [4][0];
				//upsTwo.ampChg = atof((char*) pString);
				//pString = &StrParams [5][0];
				//upsTwo.ampBat = atof((char*) pString);
				#if (defined CHARGER_MONSTER)					// Dual Model F charger
					pString = &StrParams [6][0];
					upsOne.batJoule = atol((char*) pString);
					pString = &StrParams [7][0];
					upsTwo.batJoule = atol((char*) pString);
				#endif
				strcpy((char *) responseStr,"^1");
				usart_putstr(responseStr,parseType->snmpPort);
			}
			parseType->snmpComState = SNMP_IDLE;
			break;
		case NO_SNMP_CMD:
		default:
			usart_putstr("^0",parseType->snmpPort);									// cmd not understood or not implemented
			parseType->snmpComState = SNMP_IDLE;
			break;
		}
		// after responses have finished, print to monitor
		#if defined COMM_MONITOR_PORT				// communication monitoring enabled
			usart_putstr("To SNMP:     ", COMM_MONITOR_PORT); // send string to indicated port
			usart_putstr(responseStr, COMM_MONITOR_PORT); // send string to indicated port
			usart_putstr("\r\n", COMM_MONITOR_PORT); // send string to indicated port
		#endif										// defined COMM_MONITOR_PORT
		break;
	default:
		parseType->snmpComState = SNMP_IDLE;
		parseType->lastSnmpComState = SNMP_RESPONSE;
		break;
	}	// end switch
	//__enable_interrupt();	// TODO - interrupt enable snmp_com
	//DEBUG_PORT2_0;	// ###########################

}


operatingModesT inverterControl(void) {

	static volatile int invOverloadAlmFlag = FALSE /*, invOverloadTripFlag = FALSE */;
	static volatile int invOvertempAlmFlag = FALSE, invOvertempTripFlag = FALSE;
	volatile operatingModesT invCmd;


	// starting point, when leaving function this will contain command for state machine
	invCmd = AUTO_ON;

	if (upsBoss.invFaultAlm == ON_ALARM) 
	{
		invCmd = FAULT;
		return invCmd;
	}
	if ((upsBoss.loadPctOut >= OVERLOAD_TRIP) || (upsBoss.invOverloadTrip == FAULT)) 
	{
		//if (invOverloadTripFlag == FALSE) {
			//invOverloadTripFlag = TRUE;
			updateAlm(SONALERT_ON);
		//}
	}
	/*
	else {	// this is done in ups state controller fault
		if (invOverloadTripFlag == TRUE) {
			if ( (upsBoss.loadPctOut <= OVERLOAD_TRIP_CLEAR) && (upsOne.invMode == AUTO_ON) && (upsTwo.invMode == AUTO_ON) ) {
				invOverloadTripFlag = FALSE;
				upsBoss.invOverloadTrip = NORMAL;
				updateAlm(SONALERT_OFF);
			}
		}
	}
	*/
	if (upsBoss.loadPctOut >= OVERLOAD_ALARM) 
	{
		if (invOverloadAlmFlag == FALSE) 
		{
			invOverloadAlmFlag = TRUE;
			updateAlm(SONALERT_BEEP);
		}
	} 
	else 
	{
		if (invOverloadAlmFlag == TRUE) 
		{
			if ( (upsBoss.loadPctOut <= OVERLOAD_ALARM_CLEAR) && (upsOne.invMode == AUTO_ON) && (upsTwo.invMode == AUTO_ON) ) 
			{
				invOverloadAlmFlag = FALSE;
				updateAlm(SONALERT_OFF);
			}
		}
	}
	/*
	if (invOverloadTripFlag == TRUE) 
	{
		invCmd = FAULT;								// turn off, ups state to fault
		upsBoss.invOverloadTrip = FAULT;
		return invCmd;
	}
	*/
	if (invOverloadAlmFlag == TRUE) 
	{
		invCmd = WARN;								// turn off, flash load bar
		return invCmd;
	}
	if (upsBoss.tSink >= TEMP_HS_TRIP_ON)		    // only trip on heatsink, not ambient over temp
	{
		if (invOvertempTripFlag == FALSE) 
		{
			invOvertempTripFlag = TRUE;
			if (upsBoss.battleShort == 0) 		    // don't sound alarm or shut down if battleshort is on
			{
				updateAlm(SONALERT_ON);
				invCmd = OVER_TEMPERATURE;			// turn off inverter
			}
		}
		return invCmd;
	} 
	else if (upsBoss.tSink <= TEMP_HS_TRIP_OFF) 
	{
		if (invOvertempTripFlag == TRUE) 
		{
			invOvertempTripFlag = FALSE;
			if (upsBoss.battleShort == 0)		    // don't sound alarm if battleshort is on
			{
				updateAlm(SONALERT_BEEP);
			}
		}
	}
	if ( (upsBoss.tSink >= TEMP_HS_ALM_ON) || (upsBoss.tAmb >= TEMP_AMB_ALM_ON) ) 
	{
		if (invOvertempAlmFlag == FALSE) 
		{
			invOvertempAlmFlag = TRUE;
			if (upsBoss.battleShort == 0)		    // don't sound alarm if battleshort is on
			{
				updateAlm(SONALERT_BEEP);
			}
		}
		invCmd = WARN;								// don't turn off, flash load bar
	} 
	else if ( (upsBoss.tSink <= TEMP_HS_ALM_OFF) && (upsBoss.tAmb <= TEMP_AMB_ALM_OFF) ) 
	{
		if (invOvertempAlmFlag == TRUE) 
		{
			invOvertempAlmFlag = FALSE;
			updateAlm(SONALERT_OFF);
		}
	}
	return invCmd;
}

void init_ups_state_controller(void)
{
	upsBoss.upsState = UPS_INIT;
	upsBoss.lastUpsState = UPS_OFF;
	upsBoss.invMode = AUTO_OFF;
	upsBoss.dcMode = AUTO_OFF;
	upsBoss.chgMode = AUTO_FAST;
	upsBoss.bypassMode = AUTO;
	upsBoss.syncMode = AUTO_ON;
}

void ups_state_controller(void)
{
	static volatile struct timeT time1, time2, time3, timeSlave, timeInvReg, timeCharger, timeInvOc;
	volatile float fTemp, tempMsgFloat;
	//volatile char tempMsgStr[80];	// TODO - check the length of this string *****
	static volatile int iTemp, invOcCount = 0;
	volatile int invCmd;
	static volatile operatingModesT batCondLast = NORMAL, invOcMode = OFF, invOcModeLast = OFF;
	static volatile operatingModesT bypassModeLast = NORMAL;
	// do local calculation for battery capacity
	#if ((BAT_CAP_METHOD==BAT_CAP_JOULE) && (!defined CHARGER_MONSTER))
		volatile long *FlashPointer, ltemp;
	#endif
	//	operatingModesT invCmd;

    int tmpival1, tmpival2;
    long tmplval1;
    ups_states_t tmpState;
    float tmpfval1, tmpfval2, tmpfval3;
    struct timeT tmptshutdwn, tmptstartup;
    #if (defined NOV_CAT_J)
        struct timeT chargerSumryCmdTimer;
    #endif
    operatingModesT	tmpmMode;	
                
	invCmd = inverterControl();							// check Inverter, see if action needs to be taken
	if (upsBoss.notifyMsg) 
    {
        //sprintf((char *)eventStrTemp,"ups_state_controller received message, mess=[%s], notify=%d, notify flag=%d"
        //  ,upsBoss.msgStr,upsBoss.upsNumber,upsBoss.notifyMsg);
		tmpival1 = upsBoss.upsNumber;
		tmpival2 = upsBoss.notifyMsg;
		sprintf((char *)eventStrTemp,"ups_state_controller received message, mess=[%s], notify=%d, notify flag=%d"
		        ,upsBoss.msgStr,tmpival1,tmpival2);                
		addEvent(eventStrTemp,4);
	}
	switch(upsBoss.upsState) {
    case UPS_INIT:                                          // Initial state when router/system powers up
	    //if (upsBoss.upsState != upsBoss.lastUpsState) {	// state entry code  
		tmpival1 = upsBoss.upsState;
		if (tmpival1 != upsBoss.lastUpsState)               // state entry code
		{
			upsBoss.lastUpsState = upsBoss.upsState;
			time1 = time2 = time3 = getTime();	            // reset timers
            #if (defined NOV_CAT_J)
                chargerSumryCmdTimer = getTime();           // charger timeout check timer 
            #endif
			upsBoss.SubState = 1;
			upsBoss.SubStateLast = 0;
			rotatingCmdHold = TRUE;                         // hold rotating commands
			#if ((BAT_CAP_METHOD==BAT_CAP_JOULE) && (!defined CHARGER_MONSTER))
				// get joules from flash
				// first check to see if this is a virgin systems
				FlashPointer = (long *) 0x1800;			    // start of Seg D
				ltemp = *FlashPointer;
				if (ltemp == -1) 							// uninitialized flash is all ones = -1
				{
					// start with max joules first time assuming new charged batteries
					upsOne.batJoule = upsTwo.batJoule = BAT_MAX_JOULE;
				} 
				else 
				{											// get old info
					upsOne.batJoule = *FlashPointer++;
					upsTwo.batJoule = *FlashPointer;
				}
			#endif
			addEvent("UPS State entered: UPS_INIT",9);
			updateAlm(UNBLANK_ALL);						// Put Display in normal state
		}
		switch(upsBoss.SubState) {
		case 1:
		    //if (upsBoss.SubState != upsBoss.SubStateLast) {	// first time through?
			tmpival1 = upsBoss.SubState;
			if (tmpival1 != upsBoss.SubStateLast) 	// state entry code
			{
				upsBoss.SubStateLast = upsBoss.SubState;		// set so all other passes bypass this condition
			}
			// Notification that Lucy has started or Timeout
			if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT30") == 0)) {
				time2 = getTime();
				upsBoss.SubState++;
				addEvent("UPS_INIT:1, Lucy Start notification from UPS",9);
			}
			// TODO - set to 12000 for normal, 60000 for development
			if ( (timer(time1, 12000)) || (upsStateRS485turnon == TRUE) ) {
				time2 = getTime();
				upsBoss.SubState++;
				addEvent("UPS_INIT, substate 1, Startup Timeout",9);
			}
			break;
		case 2:
            //if (upsBoss.SubState != upsBoss.SubStateLast) {			// first time through?
            tmpival1 = upsBoss.SubState;
            if (tmpival1 != upsBoss.SubStateLast) 	// state entry code
            {
                upsBoss.SubStateLast = upsBoss.SubState;				// set so all other passes bypass this condition
                masterCmdAdd(&upsCmd[RDAT20_SLAVE_ON][0], &upsTwo);	// Set Slave UPS into Slave Mode
                if (upsBoss.checksumMode == CHKSUM_ON) 
                {
                    masterCmdAddBoth(&upsCmd[RDAT22_CHKSUM_ON][0]);
                } 
                else 
                {
                    masterCmdAddBoth(&upsCmd[RDAT23_CHKSUM_OFF][0]);
                }
                upsOne.timeCmdMade = getTime();						// initialize rotating command timers
                upsTwo.timeCmdMade = getTime();
                upsOne.timeOutStart = getTime();
                upsTwo.timeOutStart = getTime();
                rotatingCmdHold = FALSE;								// remove hold on rotating commands
			}
			// Notification that lightshow is done or timeout expired
			if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT31") == 0)) {
				addEvent("UPS_INIT:2, RBUT31 Lightshow done",9);
				time2 = getTime();
				upsBoss.SubState++;
			}
			if (timer(time2, 10000)) {
				time2 = getTime();
				addEvent("UPS_INIT:2, Timeout",9);
				upsBoss.SubState++;
			}
			break;
		case 3:
		    //if (upsBoss.SubState != upsBoss.SubStateLast) {	// first time through?
			tmpival1 = upsBoss.SubState;
			if (tmpival1 != upsBoss.SubStateLast) 	// state entry code
			{
              	upsBoss.SubStateLast = upsBoss.SubState;		// set so all other passes bypass this condition
			}
			#if ((!defined UCLASS_FA10002) || (defined NAVFAC_FA10241))
				#if ((LCD_DISPLAY==FALSE) || (UPS_STATE_CONTROL==TRUE))
					if (autoStart == 1) {								// Auto Restart enabled, turn on
						addEvent("UPS_INIT:3, Auto Restart",9);
						// see if either ups started on batteries
						if ( (upsOne.dcMode == AUTO_ON) || (upsTwo.dcMode == AUTO_ON) ) {
							masterCmdAddBoth("^RMODIEA");				// Set both UPS inverters to On/Auto
							upsBoss.invMode = AUTO_ON;
							masterCmdAddBoth("^RMODDEA");				// Set both UPS DC/DC to On/Auto
							upsBoss.upsState = UPS_ON_BAT;				// if one or both UPS on batteries, startup in battery mode
						} else {
							masterCmdAddBoth("^RMODIEA");				// Set both UPS inverters to On/Auto
							upsBoss.invMode = AUTO_ON;
							//masterCmdAddBoth("^RMODDEA");				// Set both UPS DC/DC to On/Auto
							upsBoss.upsState = UPS_ON_UTIL;			// otherwise, start up on utility
						}
					} else {
						upsBoss.upsState = UPS_OFF;					// no autostart, just idle on utility power
					}
				#else
					if (timer(time2,10000)) {							// timeout
							upsBoss.upsState = UPS_OFF;				// no autostart, just idle on utility power
					}
					if (upsOne.bank1 != -1) {							// bank1 updated from UPS
						if (upsOne.bank1 & BIT1) {				// bit 1 = switch 2, 1= autostart
							addEvent("UPS_INIT:3, Auto Restart",9);
							// see if either ups started on batteries
							if ( (upsOne.dcMode == AUTO_ON) || (upsTwo.dcMode == AUTO_ON) ) {
								masterCmdAddBoth("^RMODIEA");				// Set both UPS inverters to On/Auto
								upsBoss.invMode = AUTO_ON;
								masterCmdAddBoth("^RMODDEA");				// Set both UPS DC/DC to On/Auto
								upsBoss.upsState = UPS_ON_BAT;				// if one or both UPS on batteries, startup in battery mode
							} else {
								masterCmdAddBoth("^RMODIEA");				// Set both UPS inverters to On/Auto
								upsBoss.invMode = AUTO_ON;
								//masterCmdAddBoth("^RMODDEA");				// Set both UPS DC/DC to On/Auto
								upsBoss.upsState = UPS_ON_UTIL;			// otherwise, start up on utility
							}
						} else {
							upsBoss.upsState = UPS_OFF;					// no autostart, just idle on utility power
						}
					}
				#endif
			#else													// defined UCLASS_FA10002
				if ( (timer(time2, 10000)) || (upsBoss.optoOnOff != 3) ) {
					switch (upsBoss.optoOnOff) {					// initialized to 3
					case 0:											// Remote off
						upsBoss.upsState = UPS_OFF;				// no remote on, just idle on utility power
						addEvent("UPS_INIT:3, Remote Off command",9);
						break;
					case 1:											// Remote on
						if ( (upsOne.dcMode == AUTO_ON) || (upsTwo.dcMode == AUTO_ON) ) {
							masterCmdAddBoth("^RMODIEA");			// Set both UPS inverters to On/Auto
							upsBoss.invMode = AUTO_ON;
							masterCmdAddBoth("^RMODDEA");			// Set both UPS DC/DC to On/Auto
							upsBoss.upsState = UPS_ON_BAT;			// if one or both UPS on batteries, startup in battery mode
						} else {
							masterCmdAddBoth("^RMODIEA");			// Set both UPS inverters to On/Auto
							upsBoss.invMode = AUTO_ON;
							//masterCmdAddBoth("^RMODDEA");			// Set both UPS DC/DC to On/Auto
							upsBoss.upsState = UPS_ON_UTIL;		// otherwise, start up on utility
						}
						addEvent("UPS_INIT:3, Remote On command",9);
						break;
					}
				}
			#endif
			break;
		}
		break;
	case UPS_ON_BAT:									// On battery, inverter on
	    //if (upsBoss.upsState != upsBoss.lastUpsState) {	// state entry code
        tmpState = upsBoss.upsState;
		if (tmpState != upsBoss.lastUpsState) 	// state entry code
		{
			upsBoss.lastUpsState = upsBoss.upsState;
			time1 = time2 = timeInvReg = getTime();	// reset timers
			addEvent("UPS State entered: UPS_ON_BAT",9);
			masterCmdAdd(&upsCmd[RDAT20_SLAVE_ON][0], &upsTwo);// Set Slave UPS into Slave Mode
			if (upsBoss.checksumMode == CHKSUM_ON) 
			{
			   masterCmdAddBoth(&upsCmd[RDAT22_CHKSUM_ON][0]);
			} 
			else 
			{
			   masterCmdAddBoth(&upsCmd[RDAT23_CHKSUM_OFF][0]);
			}
			masterCmdAddBoth("^RMODCNM");				// Set charger to Off/Manual
			// Micro valid commands (x=don't care) BxA = Auto, BEx = Bypass, Bxx = off if not like others (ex.BDM)
			#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
				if (upsOne.bypassMode != AUTO) 
				{
					masterCmdAdd("^RMODBDA",&upsOne);	// Set Bypass to Auto, should turn off if inverter on
				}
			#else
				if ((upsOne.bypassMode != AUTO) || (upsTwo.bypassMode != AUTO)) 
				{
					masterCmdAddBoth("^RMODBDA");		// Do the same for both systems
				}
			#endif
			timeSlave = getTime();					// remember last time it was done
			if (upsBoss.battleShort == 0) 			        // not on battleshort
			{
                          updateAlm(SONALERT_BEEP);				// Turn on master Beep alarm
			}
			masterCmdAddBoth("^RMODDEA");				// Make sure both systems on battery
			upsBoss.dcMode = AUTO_ON;					// Reflect this in virtual UPS
			upsBoss.batWarnFlag = FALSE;				// used to see if alarm is active
			updateAlm(UNBLANK_ALL);					// Put Display in normal state
	        }
		// calculate battery setpoints, they change at low power levels
		if ( ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT25") == 0))
			  || (upsBoss.loadPctOut >= OVERLOAD_TRIP) )  // Over current or overpower shutdown condition
		{
			upsBoss.invOverloadTrip = FAULT;
			upsBoss.upsState = UPS_OFF;
			addEvent("UPS reports overcurrent - RBUT25",9);
		}
        #if DUAL_BOARD==TRUE
          //if (fMin(upsOne.powOut,upsTwo.powOut) <= 40.0)        // if either is less than 40 watts, use higher setpoint
          tmpfval1 = upsOne.powOut;
          if (fMin(tmpfval1,upsTwo.powOut) <= 40.0)
        #else
          if (upsBoss.powOut <= 40.0) 						// if UPS is less than 40 watts, use higher setpoint
        #endif
        {
            #if !defined BAT_SETPOINT_CHANGE				// use default values
                upsBoss.vBatShutdown = NUM_CELLS * 1.75 * 1.01;	// shutdown at a little higher voltage than UPS
                upsBoss.vBatWarn = NUM_CELLS * 1.83;
            #else											// use custom voltages defined in config file
                upsBoss.vBatShutdown = NUM_CELLS * BAT_SET_BAT_SD_LOW_PWR * 1.01;	// shutdown at a little higher voltage than UPS
                upsBoss.vBatWarn = NUM_CELLS * BAT_SET_BAT_WARN_LOW_PWR;
            #endif
        } 
        else 
        {
            #if !defined BAT_SETPOINT_CHANGE				// use default values
                upsBoss.vBatShutdown = NUM_CELLS * 1.67 * 1.01;	// shutdown at a little higher voltage than UPS
                upsBoss.vBatWarn = NUM_CELLS * 1.75;
            #else											// use custom voltages defined in config file
                upsBoss.vBatShutdown = NUM_CELLS * BAT_SET_BAT_SD_NORM_PWR * 1.01;	// shutdown at a little higher voltage than UPS
                upsBoss.vBatWarn = NUM_CELLS * BAT_SET_BAT_WARN_NORM_PWR;
            #endif
        }
        #if DUAL_BOARD==TRUE
            //if (fMin(upsOne.voltBat,upsTwo.voltBat) <= upsBoss.vBatShutdown)
            tmpfval1 = upsOne.voltBat;
            tmpfval2 = upsBoss.vBatShutdown;
            if (fMin(tmpfval1,upsTwo.voltBat) <= tmpfval2)
        #else	// just one board parameters in upsBoss
            tmpfval1 = upsBoss.vBatShutdown;
            if (upsBoss.voltBat <= tmpfval1)
        #endif
            {
            upsBoss.upsState = UPS_SHUTDOWN;			// "Time to die, Bladerunner"
            //tempMsgFloat = fMin(upsOne.voltBat,upsTwo.voltBat);
            //tmpfval1 = upsOne.voltBat;
            //tempMsgFloat = fMin(tmpfval1,upsTwo.voltBat);
            //sprintf((char *)tempMsgStr,"Low Battery Voltage Shutdown = %fVDC",tempMsgFloat);
            //addEvent(tempMsgStr,9);
            } 
            else 
            {
            // If battery voltage below warning level or calculated time left less than 3 minutes
            // or Battery Low status from either Model F, sound warning
            tmpfval1 = upsOne.voltBat;
            tmpfval2 = upsTwo.voltBat;
            tmpfval3 = upsBoss.vBatWarn;
            if ( (
                #if DUAL_BOARD==TRUE
                    //(fMin(upsOne.voltBat,upsTwo.voltBat) <= upsBoss.vBatWarn)
                    (fMin(tmpfval1,tmpfval2) <= tmpfval3)
                #else	// just one board parameters in upsBoss
                    //(upsBoss.voltBat <= upsBoss.vBatWarn)
                    (upsBoss.voltBat <= tmpfval3)				
                #endif
                || (upsBoss.estSecBat < 180)
                || (upsBoss.batSts != 0) )			
                    && (upsBoss.batWarnFlag == FALSE) ) 
            {
                upsBoss.batWarnFlag = TRUE;			// used to see if alarm is active
                updateAlm(SONALERT_ON);				// Turn on master alarm	steady, can be turned off
                //tempMsgFloat = fMin(upsOne.voltBat,upsTwo.voltBat);
                //tempMsgFloat = fMin(tmpfval1,tmpfval2);
                //sprintf((char *)tempMsgStr,"Low Battery Voltage Warning = %fVDC",tempMsgFloat);
                //addEvent(tempMsgStr,9);
            }
        }
        if (upsBoss.batWarnFlag == TRUE) 
        {
            upsBoss.batSts = 1;						// show battery low condition
        } 
        else 
        {
            upsBoss.batSts = 0;						// show battery Normal condition
        }
        // Now utility is available, both ups are in DC Auto, when they turn the DC/DC off both are back on utility
        #if DUAL_BOARD==FALSE											// single board
          if (upsBoss.dcMode == AUTO_OFF) 
        #else
          if ( (upsOne.dcMode == AUTO_OFF) && (upsTwo.dcMode == AUTO_OFF) ) 
        #endif
          {    
            upsBoss.batSts = 0;							// show battery Normal condition
            updateAlm(SONALERT_OFF);					// Turn off master Beep alarm
            upsBoss.upsState = UPS_ON_UTIL;
            addEvent("Utility Returned",9);
          }
        #if LCD_DISPLAY==TRUE
          if (fakeButton(BUTTON1) || ((upsBoss.notifyMsg) && (strcmp((char *) upsBoss.msgStr, "^RBUT06") == 0))) // "Off" button
        #else
          if ((upsBoss.notifyMsg) && (strcmp((char *) upsBoss.msgStr, "^RBUT06") == 0))  // "Off" button
        #endif
          {
            upsBoss.upsState = UPS_OFF;
            addEvent("UPS OnBAT: Off Button Pressed",9);
          }
        #if LCD_DISPLAY==TRUE
            if (fakeButton(BUTTON3+FB_CLEAR)) 		// "Silence" button then clear button history
            {  
                updateAlm(SONALERT_OFF); 			// Turn off sonalert
                masterCmdAddBoth(almStrOne); 		// Update UPS, string may be different between the two
            }
        #endif
        #if DUAL_BOARD==TRUE
            if ( ((upsOne.invMode != AUTO_ON) || (upsTwo.invMode != AUTO_ON)) && (timer(time2, 20000)) ) // if state didn't take, keep trying
            {
                upsBoss.upsState = UPS_OFF;
                addEvent("UPS OnBAT: Both Inverters didn't turn on => UPS_OFF",9);
            }
        #endif
        if (invCmd == OVER_TEMPERATURE)  // inverterControl routine commands the UPS off
        {
            upsBoss.upsState = UPS_OFF;
            addEvent("UPS_ON_BAT: => UPS_OFF, invCmd = Overtemperature",9);
        }
        //if (invCmd == FAULT) { // inverterControl routine commands fault state
        // invFaultAlm is reported if inverter gain outside operational range
        // invOverloadTrip is set when inverter overload higher than trip setting in inverterControl()
        if (upsBoss.invFaultAlm == ON_ALARM) 
        {
            upsBoss.upsState = UPS_FAULT;
            addEvent("invCmd = FAULT",9);
        }
        if (delayedShutdown == TRUE) 					// delayed shutdown active
        {
            if (upsBoss.secShutdownDelay == -1)			// startup timer abort command
            {
                delayedShutdown = FALSE;				// deactivate startup
            } 
            //else										// decomposed else-if to make IAR workaround
            {
                tmptshutdwn = delayedShutdownTime;
                tmplval1 = upsBoss.secShutdownDelay;
                //if (timer(delayedShutdownTime, (long) upsBoss.secShutdownDelay * 1000))
                if (timer(tmptshutdwn, (long) tmplval1 * 1000))                          
                {
                  upsBoss.secShutdownDelay = -1;			// reset delay
                  delayedShutdown = FALSE;			// set delayed shutdown state
                  upsBoss.upsState = UPS_SHUTDOWN;		// light this sucker up
                  addEvent("UPS OnBAT: Delayed Shutdown",9);
                }
            }
        }
        break;
	case UPS_ON_UTIL:								// On Utility, inverter on
		tmpState = upsBoss.upsState;
		//if (upsBoss.upsState != upsBoss.lastUpsState) {	// state entry code
		if (tmpState != upsBoss.lastUpsState) 
		{
			upsBoss.lastUpsState = upsBoss.upsState;
			time1 = time2 = time3 = timeInvReg = timeCharger = getTime();	// reset timers
			addEvent("UPS State entered: UPS_ON_UTIL",9);
			masterCmdAdd(&upsCmd[RDAT20_SLAVE_ON][0], &upsTwo);             // Set Slave UPS into Slave Mode
			if (upsBoss.checksumMode == CHKSUM_ON) 
			{
				masterCmdAddBoth(&upsCmd[RDAT22_CHKSUM_ON][0]);
			} 
			else 
			{
				masterCmdAddBoth(&upsCmd[RDAT23_CHKSUM_OFF][0]);
			}
			timeSlave = getTime();						// remember last time it was done
			masterCmdAddBoth("^RMODIEA");				// set both inverters to On/Auto
			upsBoss.invMode = AUTO_ON;
			masterCmdAddBoth("^RMODDDA");				// set both DC/DC to Off/Auto
			upsBoss.dcMode = AUTO_OFF;					// reset this if coming off battery run
			masterCmdAddBoth("^RMODCFA");				// set both chargers to Fast/Auto
			upsBoss.batSts = 0;							// show battery Normal condition
			upsBoss.invOverloadTrip = NORMAL;			// allow for overload bypass recovery
			updateAlm(UNBLANK_ALL);					    // Put Display in normal state
			//upsBoss.invOverloadTrip = NORMAL;			// if recovering from overload, reset flag
			#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
				if (upsOne.bypassMode != AUTO) 
				{
					masterCmdAdd("^RMODBDA",&upsOne);	// Set Bypass to Auto, should turn off if inverter on
				}
			#else
				if ((upsOne.bypassMode != AUTO) || (upsTwo.bypassMode != AUTO)) 
				{
					masterCmdAddBoth("^RMODBDA");		// Do the same for both systems
				}
			#endif
		}
		#if ((UPS_STATE_CONTROL==TRUE) || (defined LCD_MANAGER_BOSS))
			#if ((!defined UCLASS_FA10002) || (defined NAVFAC_FA10241) || (defined CANES_FA10249))
				if ( ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT25") == 0))
					  || (upsBoss.loadPctOut >= OVERLOAD_TRIP) )
				{ // Over current or overpower shutdown condition
					upsBoss.invOverloadTrip = FAULT;
					upsBoss.upsState = UPS_OFF;
					addEvent("UPS reports overcurrent - RBUT25 or overload",9);
				}
			#else										// defined UCLASS_FA10002
				if ( ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT25") == 0))
					  || (upsBoss.loadPctOut >= OVERLOAD_TRIP) )
				{ // Over current or overpower shutdown condition
					upsBoss.invOverloadTrip = FAULT;
					upsBoss.upsState = UPS_OFF;
					addEvent("UPS reports overcurrent - RBUT25 or overload",9);
					#if (defined FORCE_BYPASS_OVERLOAD_RBUT25)	// Put in Bypass when Model F reports overload
						masterCmdAddBoth("^RMODBDM");			// Put both systems bypass off
					#endif
				}
			#endif										// !defined UCLASS_FA10002
		#endif
		/*
		if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT25") == 0)) { // Over current or overpower shutdown condition
			upsBoss.upsState = UPS_FAULT;
			addEvent("UPS reports overcurrent - RBUT25",9);
		}
		*/
		if ( ((upsOne.bypassMode == ON) || (upsTwo.bypassMode == ON)) && (bypassModeLast != ON) ) {
			bypassModeLast = ON;
			addEvent("UPS_ON_UTILITY: One or both UPS on Bypass",9);
		} else if ( (upsOne.bypassMode != ON) && (upsTwo.bypassMode != ON) && (bypassModeLast == ON) ) {
			bypassModeLast = AUTO;
			addEvent("UPS_ON_UTILITY: Both UPS Bypass => Auto or Off",9);
		}
		// Handle Inverter Overcurrent transfer to Bypass, try to recover if overload clears
		#if defined BYPASS_RECOVER							// Will try to recover from overload bypass
			// 2nd level overload, next stage beyond "^RBUT25"
			if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT26") == 0)) {
				#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
					masterCmdAdd("^RMODBDM",&upsOne);		// Put master bypass off
				#else
					masterCmdAddBoth("^RMODBDM");			// Put both systems bypass off
				#endif
				upsBoss.invOverloadTrip = FAULT;
				upsBoss.upsState = UPS_OFF;
				addEvent("2nd Level Overload",9);
			}
			switch (invOcMode) {
			case OFF:
				if (invOcMode != invOcModeLast) {
					invOcModeLast = invOcMode;
				}
				// Over current or overpower condition
				if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT25") == 0)) {
					invOcMode = ON;
					addEvent("UPS reports overcurrent - RBUT25",9);
				}
				break;
			case ON:
				if (invOcMode != invOcModeLast) {
					invOcModeLast = invOcMode;
					invOcCount++;
					timeInvOc = getTime();					// reset timer for mode
					// Micro valid commands (x=don't care) IxA = Auto, IEx = Inv on,
					//INx = inv on - bypass no change, IDx = inv and bypass off,
					//IFx = inverter off bypass no change 
					masterCmdAddBoth("^RMODINA");			// set both inverters to On/Auto, stay in bypass
				}
				if (invOcCount >= 3) {
					invOcCount = 0;							// reset for next overload
					upsBoss.upsState = UPS_BYPASS;
				}
				if (timer(timeInvOc,5000)) {
					// Micro valid commands (x=don't care) BxA = Auto, BEx = Bypass, Bxx = off if not like others (ex.BDM)
					#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
						masterCmdAdd("^RMODBDA",&upsOne);	// Set Bypass to Auto, should turn off if inverter on
					#else
						masterCmdAddBoth("^RMODBDA");		// Do the same for both systems
					#endif
					invOcMode = DELAY;
				}
				break;
			case DELAY:
				if (invOcMode != invOcModeLast) {
					invOcModeLast = invOcMode;
					timeInvOc = getTime();					// reset timer for mode
				}
				if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT25") == 0)) {
					invOcMode = ON;
					addEvent("UPS reports overcurrent - RBUT25",9);
				}
				if (timer(timeInvOc,15000)) {
					if (upsBoss.loadPctOut <= 100.0) {		// Inverter overload over?
						invOcCount = 0;						// reset for next overload
						invOcMode = OFF;					// everything fine, leave OverCurrent Mode
					}
				}
				break;
			}
		#endif // BYPASS_RECOVER
		// "Off" button pressed
		#if LCD_DISPLAY==TRUE
		if (fakeButton(BUTTON1) || ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT06") == 0))) { // "Off" button
		#else
		if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT06") == 0)) { // "Off" button
		#endif
			upsBoss.upsState = UPS_OFF;
			addEvent("UPS_ON_UTILITY: Off Button Pressed",9);
		}
		#if LCD_DISPLAY==TRUE
			if (fakeButton(BUTTON3+FB_CLEAR)) {		// "Silence" button then clear button history
				updateAlm(SONALERT_OFF); 			// Turn off sonalert
				masterCmdAddBoth(almStrOne); 		// Update UPS, string may be different between the two
			}
		#endif
		if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT12") == 0)) { // "Bypass" button
			// Micro valid commands (x=don't care) BxA = Auto, BEx = Bypass, Bxx = off if not like others (ex.BDM)
			if (upsBoss.bypassMode == ON) {			// if either board on bypass
				#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
					masterCmdAdd("^RMODBDA",&upsOne);		// Set Bypass to Auto, should turn off if inverter on
				#else
					masterCmdAddBoth("^RMODBDA");			// Do the same for both systems
				#endif
			} else {
				#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
					masterCmdAdd("^RMODBEM",&upsOne);		// Put master bypass on
				#else
					masterCmdAddBoth("^RMODBEM");			// Put both systems bypass on
				#endif
			}
			addEvent("Bypass Toggle Button Pressed",9);
		}
		#ifdef OVERLOAD_BYPASS
			if ((upsBoss.bypassMode == ON) && (upsBoss.invMode != AUTO_ON)) {
				masterCmdAddBoth("^RMODINA");				// set both inverters to On/Auto, stay in bypass
			}
		#endif
		// "On" button pressed when in bypass
		if ( (upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT08") == 0)
			&& (upsBoss.bypassMode == ON) ) {				// On button pressed
			#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
				masterCmdAdd("^RMODBDA",&upsOne);			// Set Bypass to Auto, should turn off if inverter on
			#else
				masterCmdAddBoth("^RMODBDA");				// Do the same for both systems
			#endif
			addEvent("UPS_ON_UTILITY: On bypass, ON button pressed",9);
		}
		// one or both UPS are running on batteries, switch to Auto so they will be able to transfer to Utility
		#if DUAL_BOARD==TRUE
			if ( (upsOne.dcMode == MANUAL_ON) || (upsTwo.dcMode == MANUAL_ON) ) {
				if (timer(time1, 5000)) {					// don't leave until unit stable and data updated
					time1 = getTime();
					masterCmdAddBoth("^RMODDEA");
				}
			} else {
				// one or both UPS are running on batteries
				if ((upsOne.dcMode == AUTO_ON) || (upsTwo.dcMode == AUTO_ON)) {
					if (timer(time1, 10000)) {				// don't leave until unit stable and data updated
						upsBoss.upsState = UPS_ON_BAT;		// so switch to battery mode
					}
				}
			}
		#else												// single board
			if (upsBoss.dcMode == MANUAL_ON) {
				if (timer(time1, 5000)) {					// don't leave until unit stable and data updated
					time1 = getTime();
					masterCmdAddBoth("^RMODDEA");
				}
			} else {
				// UPS is running on batteries
				if (upsBoss.dcMode == AUTO_ON) {
					if (timer(time1, 10000)) {				// don't leave until unit stable and data updated
						upsBoss.upsState = UPS_ON_BAT;		// so switch to battery mode
					}
				}
			}
		#endif
		#if DUAL_BOARD==TRUE
			// if state didn't take, give up
			if ( ((upsOne.invMode != AUTO_ON) || (upsTwo.invMode != AUTO_ON)) && (timer(time2, 20000)) ) {
				upsBoss.upsState = UPS_OFF;
				addEvent("Timeout, one or both inverters not Auto/On after 20 seconds",9);
			}
		#else											// single board
			if ( (upsBoss.invMode != AUTO_ON) && (timer(time2, 10000)) ) {
				upsBoss.upsState = UPS_OFF;
				addEvent("UPS Inverter off",9);
			}
		#endif
		if (invCmd == OVER_TEMPERATURE) { // inverterControl routine commands the UPS off, heatsink overtemp
			upsBoss.upsState = UPS_OFF;
			addEvent("UPS_ON_UTILITY: => UPS_OFF, invCmd = Overtemperature",9);
		}
		//if (invCmd == FAULT) { // inverterControl routine commands fault state
		// invFaultAlm is reported if inverter gain outside operational range
		// invOverloadTrip is set when inverter overload higher than trip setting in inverterControl()
		if (upsBoss.invFaultAlm == ON_ALARM) {
			upsBoss.upsState = UPS_FAULT;
			addEvent("invCmd = FAULT",9);
		}
		if (upsBoss.invOverloadTrip == FAULT) {		// overcurrent shutdown
			upsBoss.upsState = UPS_OFF;
			addEvent("Inverter Overload trip",9);
		}
		if (delayedShutdown == TRUE)                    // delayed shutdown active
		{					
			if (upsBoss.secShutdownDelay == -1) 	// startup timer abort command
			{
				delayedShutdown = FALSE;				// deactivate startup
			} 
			else 										// decomposed else-if for IAR workaround
			{
				tmptshutdwn = delayedShutdownTime;
				tmplval1 = upsBoss.secShutdownDelay;
				//if (timer(delayedShutdownTime, (long) upsBoss.secShutdownDelay * 1000))
				if (timer(tmptshutdwn, (long) tmplval1 * 1000))                          
				{
				  upsBoss.secShutdownDelay = -1;		// reset delay
				  delayedShutdown = FALSE;				// set delayed shutdown state
				  upsBoss.upsState = UPS_SHUTDOWN;		// light this sucker up
				}
			}
		}
		if (timer(upsBoss.timeStarted, 60000)) 			// wait until system is up and stable
		{
			tmpmMode = upsBoss.batCond;
			//if (upsBoss.batCond != batCondLast) {
			if (tmpmMode != batCondLast) 
			{
				batCondLast = upsBoss.batCond;
				if ((upsBoss.upsState == UPS_ON_UTIL) || (upsBoss.upsState == UPS_OFF)){
					switch (upsBoss.batCond) {
					case NORMAL:
						updateAlm(SONALERT_OFF);
						if (upsBoss.upsState == UPS_ON_UTIL) {
							masterCmdAddBoth("^RMODCFA");	// set both UPS chargers to Fast/Auto
						}
						break;
					case WARN:
						updateAlm(SONALERT_BEEP);
						break;
					case OVER_VOLTAGE:
						masterCmdAddBoth("^RMODCNM");		// set both UPS chargers to Off/Manual
					case FAULT:
						updateAlm(SONALERT_ON);
						break;
					}
				}
			}
		}
		if ((upsOne.batCond == NORMAL) && (timer(timeCharger, 10000))){// not in battery fault condition
			timeCharger = getTime();					// reset timer
			switch(upsOne.chgMode) {					// check to see if charger in wrong mode
			case MANUAL_FAST:							// manual/fast => auto/fast
				masterCmdAdd("^RMODCFA", &upsOne);
				break;
			case MANUAL_SLOW:							// manual/slow => auto/slow
				masterCmdAdd("^RMODCSA", &upsOne);
				break;
			case AUTO_FAST:								// both of these okay, don't change
			case AUTO_SLOW:
				break;
			default:									// anything else bad, go to fast/auto
				masterCmdAdd("^RMODCFA", &upsOne);
				break;
			}
			switch(upsTwo.chgMode) {					// check to see if charger in wrong mode
			case MANUAL_FAST:							// manual/fast => auto/fast
				masterCmdAdd("^RMODCFA", &upsTwo);
				break;
			case MANUAL_SLOW:							// manual/slow => auto/slow
				masterCmdAdd("^RMODCSA", &upsTwo);
				break;
			case AUTO_FAST:								// both of these okay, don't change
			case AUTO_SLOW:
				break;
			default:									// anything else bad, go to fast/auto
				masterCmdAdd("^RMODCFA", &upsTwo);
				break;
			}
		}
        #if (defined NOV_CAT_J)
            if (timer(chargerSumryCmdTimer, 5000))
            {
                chargerSumryCmdTimer = getTime();
                if (charger.comOkay == FALSE)       // TODO - NOV CAT-J
                {
                    masterCmdAddBoth("RMODTE");     // Summary Alarm Enabled
                }
                else
                {
                    masterCmdAddBoth("RMODTD");     // Summary Alarm Disabled
                }
            }
        #endif
		break;
	case UPS_SHUTDOWN:
		tmpState = upsBoss.upsState;										// Shutdown, transition state
		//if (upsBoss.upsState != upsBoss.lastUpsState) {	// state entry code
		if (tmpState != upsBoss.lastUpsState) 
		{
			upsBoss.lastUpsState = upsBoss.upsState;
			upsBoss.SubState = 1;							// set to substate
			upsBoss.SubStateLast = 0;						// set to different substate for state entry test
			time1 = time2 = getTime();	// reset timers
			#if ((BAT_CAP_METHOD==BAT_CAP_JOULE) && (!defined CHARGER_MONSTER))
				// save joules to flash
				FlashPointer = (long *) 0x1800;			// start of Seg D
				__disable_interrupt();						// 5xx Workaround: Disable global
				// interrupt while erasing. Re-Enable
				// GIE if needed
				FCTL3 = FWKEY;								// Clear Lock bit
				FCTL1 = FWKEY+ERASE;						// Set Erase bit
				*(unsigned int *)FlashPointer = 0;			// Dummy write to erase Flash seg
				//FCTL1 = FWKEY+BLKWRT;					// Enable long-word write
				FCTL1 = FWKEY+WRT;							// Set WRT bit for write operation
				*FlashPointer++ = upsOne.batJoule;			// write to flash
				*FlashPointer = upsTwo.batJoule;
				//*(unsigned long *)Flash_ptrD = value;	// Write to Flash
				FCTL1 = FWKEY;								// Clear WRT bit
				FCTL3 = FWKEY+LOCK;							// Set LOCK bit
				__enable_interrupt();						// 5xx Workaround: Disable global
			#endif
			addEvent("UPS State entered: UPS_SHUTDOWN",9);
			//updateAlm(BLANK_ALL);						// Turn off display
			#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
				masterCmdAdd("^RMODBDM",&upsOne);			// Put master bypass off
			#else
				masterCmdAddBoth("^RMODBDM");				// Put both systems bypass off
			#endif
			#if defined OLD_SHUTDOWN
				masterCmdAddBoth("^RMODIDM");					// Turn off Inverters (Off/Manual)
				masterCmdAddBoth("^RMODIDM");					// Turn off Inverters (Off/Manual)
				masterCmdAddBoth("^RMODGD");					// Special shutdown for CANES4 (SNMP shutdown didn't work)
			#endif
		}
		#if !defined OLD_SHUTDOWN
			switch (upsBoss.SubState) 
			{
			case 1:
 				tmpival1 = upsBoss.SubState;             
 				//if (upsBoss.SubState != upsBoss.SubStateLast)
				if (tmpival1 != upsBoss.SubStateLast) 
				{
					upsBoss.SubStateLast = upsBoss.SubState;
					masterCmdAdd("^RMODIDM", &upsTwo);			// Turn off Inverters (Off/Manual), slave first otherwise it loses PWM from master
					masterCmdAdd("^RMODGD", &upsTwo);					// Special shutdown for CANES4 (SNMP shutdown didn't work)
				}
				if (timer(time2, 5000)) 
				{
					time2 = getTime();
					upsBoss.SubState++;
				}
				break;
			case 2:
 				tmpival1 = upsBoss.SubState;             
 				//if (upsBoss.SubState != upsBoss.SubStateLast)
				if (tmpival1 != upsBoss.SubStateLast) 
				{
					upsBoss.SubStateLast = upsBoss.SubState;
					masterCmdAdd("^RMODIDM", &upsOne);			// Turn off Inverters (Off/Manual), slave first otherwise it loses PWM from master
					masterCmdAdd("^RMODGD", &upsOne);					// Special shutdown for CANES4 (SNMP shutdown didn't work)
				}
				break;
			}
		#endif                                                // !defined OLD_SHUTDOWN
		if (timer(time1, 10000)) {							// give it some time
			time1 = getTime();
			// Waiting until both UPS power off or if Utility back go to OFF mode
			if ( (upsOne.dcMode == AUTO_OFF) && (upsTwo.dcMode == AUTO_OFF) ) {
				upsBoss.upsState = UPS_OFF;
			}
		}
		break;
	case UPS_OFF:												// System powered but Inverter off
		tmpState = upsBoss.upsState;
		//if (upsBoss.upsState != upsBoss.lastUpsState) {		// state entry code
		if (tmpState != upsBoss.lastUpsState) 
		{
			upsBoss.lastUpsState = upsBoss.upsState;
			upsBoss.SubState = 1;
			upsBoss.SubStateLast = 0;
			time1 = time2 = time3 = timeCharger = getTime();	// reset timers
			addEvent("UPS State entered: UPS_OFF",9);
			if (upsBoss.invOverloadTrip != FAULT)               // if overload didn't send us here 
			{
				upsBoss.invMode = MANUAL_OFF;					// Flag to make sure inverter is turned off
				#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
					masterCmdAdd("^RMODBDM",&upsOne);			// Put master bypass off
				#else
					masterCmdAddBoth("^RMODBDM");				// Put both systems bypass off
				#endif
				masterCmdAddBoth("^RMODIDM");					// set both UPS inverters to Off/Manual
			} 
			else                                                // upsBoss.invOverloadTrip == FAULT
			{											        // Overload Trip = FAULT condition
				#if (!defined OVERLOAD_BYPASS_DISABLE)			// if not disabled and overload, go to bypass
					#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
						masterCmdAdd("^RMODBEM",&upsOne);		// Put master bypass on
					#else
                        masterCmdAddBoth("^RMODBEM");			// Put both systems bypass on
					#endif
				#else											// defined OVERLOAD_BYPASS_DISABLE
                    #if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
                        masterCmdAdd("^RMODBDM",&upsOne);       // Put master bypass off
                    #else
                        masterCmdAddBoth("^RMODBDM");           // Put both systems bypass off
                    #endif
					masterCmdAddBoth("^RMODIDM");				// set both UPS inverters to Off/Manual
				#endif
			}
			updateAlm(UNBLANK_ALL);							    // Put Display in normal state
		}
		switch(upsBoss.SubState) 
		{
		case 1:
 			tmpival1 = upsBoss.SubState;             
 			//if (upsBoss.SubState != upsBoss.SubStateLast) 	// first time through?
			if (tmpival1 != upsBoss.SubStateLast) 
			{
				upsBoss.SubStateLast = upsBoss.SubState;		// set so all other passes bypass this condition
			}
			if (timer(time3, 10)) 
			{
				time3 = getTime();
				// this is for overload bypass, stay on inverter
				//masterCmdAddBoth("^RMODIDM");				// set both UPS inverters to Off/Manual
				//addEvent("Startup Timeout",9);
				upsBoss.SubState++;
			}
			break;
		default:
			// one or both UPS are running on batteries, switch to Auto so they will be able to transfer to Utility
			if ( (upsOne.dcMode == MANUAL_ON) || (upsTwo.dcMode == MANUAL_ON) ) 
			{
				if (timer(time1, 5000)) 				// don't leave until unit stable and data updated
				{
					time1 = getTime();
					addEvent("UPS_OFF: DC/DC on manual, set to Auto",9);
					masterCmdAddBoth("^RMODDEA");					// set both DC/DC to On/Auto
				}
			} 
			else 
			{
				// one or both UPS are running on batteries
				if ( ((upsOne.dcMode == AUTO_ON) || (upsTwo.dcMode == AUTO_ON))
					  && (upsBoss.voltIn < (UPS_VOLTINNOM * 0.25)) )  // In case of Dual Input make sure no utility
				{
					  if (timer(time1, 10000)) 				// don't leave until unit stable and data updated
					  {
						addEvent("UPS_OFF: Inverter off, On batteries => Shutdown",9);
						upsBoss.upsState = UPS_SHUTDOWN;	// so shutdown
					  }
				}
			}
			#if (LCD_DISPLAY==TRUE)
			  if (fakeButton(BUTTON1) || ((upsBoss.notifyMsg) && (strcmp((char *) upsBoss.msgStr,"^RBUT08") == 0))) // "On" button
			#else
			  if ((upsBoss.notifyMsg) && (strcmp((char *) upsBoss.msgStr,"^RBUT08") == 0)) // "On" button
			#endif
			  {
				addEvent("UPS_OFF: ON button pressed",9);
				upsBoss.invOverloadTrip = NORMAL;		// Reset Flag
				upsBoss.upsState = UPS_ON_UTIL;
			  }
			#if LCD_DISPLAY==TRUE
			  if (fakeButton(BUTTON2) || ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT06") == 0)))  // "Off" button
			#else
			  if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT06") == 0))  // "Off" button
			#endif
			  {
				upsBoss.upsState = UPS_SHUTDOWN;
				addEvent("UPS_OFF: Off Button Pressed",9);
			  }
			#if LCD_DISPLAY==TRUE
				if (fakeButton(BUTTON3+FB_CLEAR)) {	// "Silence" button then clear button history
					updateAlm(SONALERT_OFF); 			// Turn off sonalert
					masterCmdAddBoth(almStrOne); 		// Update UPS, string may be different between the two
				}
			#endif 									// LCD_DISPLAY==TRUE
			/*
			// "On" button or Remote on (same cmd from Micro board)
			if (upsBoss.startupPending == TRUE) {
				upsBoss.startupPending = FALSE;
				addEvent("UPS_OFF: ON button pressed or Remote On",9);
				upsBoss.upsState = UPS_ON_UTIL;
			}
			*/
			if ((upsBoss.invMode == MANUAL_OFF) && (timer(time2, 5000))) {	// if state didn't take, keep trying
				time2 = getTime();
				// check to see if anyone is still running inverter
				if ( (upsOne.invMode == AUTO_ON) || (upsTwo.invMode == AUTO_ON)
						|| (upsOne.invMode == MANUAL_ON) || (upsTwo.invMode == MANUAL_ON) ) {
					masterCmdAddBoth("^RMODIDM");			// if so, de-energize and put in manual
				}
			}
			// started this state with systemShutdown= TRUE, if CB1 and Power Switch on when entering
			// then one needs to by cycled off then on to start UPS
			// no loads and one or both AC inputs failed, shut down completely
			if (delayedStartup == TRUE) {					// delayed startup active
				if (upsBoss.secStartupDelay == -1) 		// startup timer abort command
				{
					delayedStartup = FALSE;				// deactivate startup
				} 
				else
				{
					tmptstartup = delayedStartupTime;
					//if (timer(delayedStartupTime, (long) upsBoss.secStartupDelay * 1000)) 
					if (timer(tmptstartup, (long) upsBoss.secStartupDelay * 1000)) 
					{
					  delayedStartup = FALSE;				// set delayed shutdown state
					  upsBoss.secStartupDelay = -1;			// reset delay
					  upsBoss.upsState = UPS_ON_UTIL;		// light this sucker up
					}
				}
			}
			tmpmMode = upsBoss.batCond;
			//if (upsBoss.batCond != batCondLast) {
			if (tmpmMode != batCondLast) 
			{
				batCondLast = upsBoss.batCond;
				switch (upsBoss.batCond) {
				case NORMAL:
					updateAlm(SONALERT_OFF);
					if (upsBoss.upsState == UPS_ON_UTIL) {
						masterCmdAddBoth("^RMODCFA");		// set both UPS chargers to Fast/Auto
					}
					break;
				case WARN:
					updateAlm(SONALERT_BEEP);
					break;
				case OVER_VOLTAGE:
					masterCmdAddBoth("^RMODCNM");			// set both UPS chargers to Off/Manual
				case FAULT:
					updateAlm(SONALERT_ON);
					break;
				}
			}
			if ((upsOne.batCond == NORMAL) && (timer(timeCharger, 10000))){// not in battery fault condition
				timeCharger = getTime();					// reset timer
				switch(upsOne.chgMode) {					// check to see if charger in wrong mode
				case MANUAL_FAST:							// manual/fast => auto/fast
					masterCmdAdd("^RMODCFA", &upsOne);
					break;
				case MANUAL_SLOW:							// manual/slow => auto/slow
					masterCmdAdd("^RMODCSA", &upsOne);
					break;
				case AUTO_FAST:								// both of these okay, don't change
				case AUTO_SLOW:
					break;
				default:									// anything else bad, go to fast/auto
					masterCmdAdd("^RMODCFA", &upsOne);
					break;
				}
				switch(upsTwo.chgMode) {					// check to see if charger in wrong mode
				case MANUAL_FAST:							// manual/fast => auto/fast
					masterCmdAdd("^RMODCFA", &upsTwo);
					break;
				case MANUAL_SLOW:							// manual/slow => auto/slow
					masterCmdAdd("^RMODCSA", &upsTwo);
					break;
				case AUTO_FAST:								// both of these okay, don't change
				case AUTO_SLOW:
					break;
				default:									// anything else bad, go to fast/auto
					masterCmdAdd("^RMODCFA", &upsTwo);
					break;
				}
			}
			break;
		}
        #if (defined NOV_CAT_J)
            if (timer(chargerSumryCmdTimer, 5000))
            {
                chargerSumryCmdTimer = getTime();
                if (charger.comOkay == FALSE)       // TODO - NOV CAT-J
                {
                    masterCmdAddBoth("RMODTE");     // Summary Alarm Enabled
                }
                else
                {
                    masterCmdAddBoth("RMODTD");     // Summary Alarm Disabled
                }
            }
        #endif
		break;
	#ifdef BYPASS_RECOVER									// Will try to recover from overload bypass
	case UPS_BYPASS:
		if (upsBoss.upsState != upsBoss.lastUpsState) 	// state entry code
		{
			upsBoss.lastUpsState = upsBoss.upsState;
			time1 = time2 = getTime();	// reset timers
			upsBoss.SubState = 1;
			upsBoss.SubStateLast = 0;
			addEvent("UPS State entered: UPS_BYPASS",9);
			invOcMode = OFF;
		}
		switch(upsBoss.SubState) {
		case 0:
			if (upsBoss.SubState != upsBoss.SubStateLast) {	// first time through?
				upsBoss.SubStateLast = upsBoss.SubState;		// set so all other passes bypass this condition
			}
			//if (timer(time1,1000)) {
				// Micro valid commands (x=don't care) BxA = Auto, BEx = Bypass, Bxx = off if not like others (ex.BDM)
				#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
					masterCmdAdd("^RMODBEM",&upsOne);			// Put master bypass on
				#else
					masterCmdAddBoth("^RMODBEM");				// Put both systems bypass on
				#endif
				time1 = getTime();
				upsBoss.SubState++;
			//}
			break;
		case 1:
			if (upsBoss.SubState != upsBoss.SubStateLast) {	// first time through?
				upsBoss.SubStateLast = upsBoss.SubState;		// set so all other passes bypass this condition
			}
			if (timer(time1,1000)) {
				// Micro valid commands (x=don't care) IxA = Auto, IEx = Inv on,
				//INx = inv on - bypass no change, IDx = inv and bypass off,
				//IFx = inverter off bypass no change 
				masterCmdAddBoth("^RMODIFM");					// set both inverters to Off
				#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
					masterCmdAdd("^RMODBEM",&upsOne);			// Put master bypass on
				#else
					masterCmdAddBoth("^RMODBEM");				// Put both systems bypass on
				#endif
				time1 = getTime();
				upsBoss.SubState++;
			}
			break;
		case 2:
			if (upsBoss.SubState != upsBoss.SubStateLast) {	// first time through?
				upsBoss.SubStateLast = upsBoss.SubState;		// set so all other passes bypass this condition
			}
			if (timer(time1,1000)) {							// keep sending out command while in state
				time1 = getTime();
				#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
					masterCmdAdd("^RMODBEM",&upsOne);			// Put master bypass on
				#else
					masterCmdAddBoth("^RMODBEM");				// Put both systems bypass on
				#endif
			}
			break;
		default:
			break;
		}
		if ( (upsBoss.notifyMsg) && 
			( (strcmp((char *)upsBoss.msgStr,"^RBUT08") == 0)			// "On" button
			|| (strcmp((char *)upsBoss.msgStr,"^RBUT12") == 0) )  ) {	// "Bypass" button
			addEvent("UPS_BYPASS: ON or Bypass button pressed",9);
			masterCmdAddBoth("^RMODIEA");				// set both inverters to On/Auto
			masterCmdAddBoth("^RMODDDA");				// set both DC/DC to Off/Auto
			masterCmdAddBoth("^RMODCFA");				// set both Chargers to Fast/Auto
			upsBoss.invMode = AUTO_ON;
			upsBoss.dcMode = AUTO_OFF;					// reset this if coming off battery run
			upsBoss.batSts = 0;							// show battery Normal condition
			time1 = getTime();
			upsBoss.upsState = UPS_ON_UTIL;
		}
		// if we got here by customer pressing Bypass toggle buttons
		if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT12") == 0)) { // "Bypass" button
			addEvent("Bypass Button Pressed",9);
			upsBoss.upsState = UPS_ON_UTIL;
		}
		/*
		if ( (invOcMode == OFF) && (timer(time1, 5000)) ) {
			upsBoss.upsState = UPS_ON_UTIL;
		}
		*/
		#if LCD_DISPLAY==TRUE
		if (fakeButton(BUTTON1) || ((upsData->notifyMsg) && (strcmp(upsData->msgStr,"^RBUT08") == 0))) {// "On" button
		#else
		if ((upsData->notifyMsg) && (strcmp(upsData->msgStr,"^RBUT08") == 0)) {// "On" button
		#endif
			addEvent("UPS_OFF: ON button pressed",9);
			upsData->upsState = UPS_ON_UTIL;
		}
		#if LCD_DISPLAY==TRUE
		if (fakeButton(BUTTON2) || ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT06") == 0))) {// "Off" button
		#else
		if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT06") == 0)) {// "Off" button
		#endif
			#if ((DUAL_BOARD == TRUE) && (defined DUAL_BOARD_MASTER_BYPASS_CONTROL))
				masterCmdAdd("^RMODBDM",&upsOne);			// Put master bypass off
			#else
				masterCmdAddBoth("^RMODBDM");				// Put both systems bypass off
			#endif
			addEvent("UPS_OFF: ON button pressed",9);
			upsData->upsState = UPS_OFF;
		}
		#if LCD_DISPLAY==TRUE
			if (fakeButton(BUTTON3+FB_CLEAR)) {		// "Silence" button then clear button history
				updateAlm(SONALERT_OFF); 			// Turn off sonalert
				masterCmdAddBoth(almStrOne); 		// Update UPS, string may be different between the two
			}
		#endif
		break;
	#endif //BYPASS_RECOVER
	case UPS_FAULT:											// System powered, indicating fault
		tmpState = upsBoss.upsState;
		//if (upsBoss.upsState != upsBoss.lastUpsState) {		// state entry code
		if (tmpState != upsBoss.lastUpsState) 
		{
			upsBoss.lastUpsState = upsBoss.upsState;
			time1 = time2 = getTime();	// reset timers
			addEvent("UPS State entered: UPS_FAULT",9);
			updateAlm(UNBLANK_ALL);						// Put Display in normal state
			updateAlm(SONALERT_ON);						// the updateLedDisplay will turn fault led on
			masterCmdAddBoth("^RMODIDM");					// Turn off Inverters (Off/Manual)
		}
		#if LCD_DISPLAY==TRUE
		if (fakeButton(BUTTON2) || ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT06") == 0)) /* && (upsBoss.invFaultAlm == OFF_ALARM) */ ) { // "Off" button
		#else
		if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT06") == 0) /* && (upsBoss.invFaultAlm == OFF_ALARM) */ ) { // "Off" button
		#endif
			addEvent("Off Button Pressed in UPS_FAULT_STATE, no InvFault set",9);
			time1 = getTime();
			// recover from invOverloadTrip, not from invFaultAlm (inverter dead)
			if (upsBoss.invOverloadTrip == FAULT) {
				upsBoss.invOverloadTrip = NORMAL;
				updateAlm(SONALERT_OFF);
			}
			if (upsBoss.invFaultAlm == ON_ALARM) {
				upsBoss.invFaultAlm = OFF_ALARM;
			}
			masterCmdAddBoth("^RMODIDM");				// set both inverters to Off/Manual
			upsBoss.invMode = MANUAL_OFF;
			upsBoss.upsState = UPS_OFF;
		}
		#if LCD_DISPLAY==TRUE
			if (fakeButton(BUTTON3+FB_CLEAR)) {		// "Silence" button then clear button history
				updateAlm(SONALERT_OFF); 			// Turn off sonalert
				masterCmdAddBoth(almStrOne); 		// Update UPS, string may be different between the two
			}
		#endif
		/*
		if (timer(time2, 10000)) {						// give it some time
			time2 = getTime();
			masterCmdAddBoth("^RMODIDM");				// Turn off Inverters (Off/Manual)
			masterCmdAddBoth("^RMODDEA");				// switch DC/DC from On/Man to On/Auto
		}
		*/
		break;
	case UPS_COM_SHUTDOWN:								// System powered, indicating fault
		tmpState = upsBoss.upsState;
		//if (upsBoss.upsState != upsBoss.lastUpsState) {		// state entry code
		if (tmpState != upsBoss.lastUpsState)
		{ 
			upsBoss.lastUpsState = upsBoss.upsState;
			time1 = getTime();	// reset timers
			upsBoss.SubState = 1;							// Case sequence, start at 1
			addEvent("UPS State entered: UPS_COM_SHUTDOWN",9);
			updateAlm(UNBLANK_ALL);						// the updateLedDisplay will turn fault led on
			updateAlm(SONALERT_ON);						// Turn on master alarm	steady, can be turned off
		}
		// by sequencing one power supply off, then on, wait, then the other, if the Utility
		//   is available there will be continuous power to this board, if on batteries it will power down
		switch (upsBoss.SubState) {
		case 1:
			masterCmdAddBoth("^RMODIDM");				// set both inverters to Off/Manual
			upsBoss.SubState++;
			break;
		case 2:
			if (timer(time1, 10000)) {
				CUTTHROAT_MASTER_ON;					// Turn on FET that drives optoisolator for Master
				time1 = getTime();						// reset timer for next state
				upsBoss.SubState++;								// select next state
			}
			break;
		case 3:
			if (timer(time1, 100)) {					// hold signal long enough to reset UPS Microcontroller
				CUTTHROAT_MASTER_OFF;					// Turn off FET that drives optoisolator for Master
				time1 = getTime();						// reset timer for next state
				upsBoss.SubState++;								// select next state
			}
			break;
		case 4:
			if (timer(time1, 500)) {					// wait for master 12VDC to return and stabilize
				CUTTHROAT_SLAVE_ON;						// Turn on FET that drives optoisolator for Slave
				time1 = getTime();						// reset timer for next state
				upsBoss.SubState++;								// select next state
			}
			break;
		case 5:
			if (timer(time1, 100)) {					// hold signal long enough to reset UPS Microcontroller
				CUTTHROAT_SLAVE_OFF;					// Turn off FET that drives optoisolator for Slave
				time1 = getTime();						// reset timer for next state
				upsBoss.SubState++;								// select next state
			}
			break;
			//		default:										// do nothing
			//			break;
		}
		// "Off" button
		#if LCD_DISPLAY==TRUE
		if (fakeButton(BUTTON2) || ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT06") == 0))) {
		#else
		if ((upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT06") == 0)) {
		#endif
			upsOne.comErrors = upsTwo.comErrors = 0;	// reset errors
			addEvent("Off Button Pressed in UPS_COM_SHUTDOWN",9);
			masterCmdAddBoth("^RMODIDM");				// set both inverters to Off/Manual
			upsBoss.invMode = MANUAL_OFF;
			upsBoss.upsState = UPS_OFF;
		}
		break;
	default:
		tmpState = upsBoss.upsState;
		//if (upsBoss.upsState != upsBoss.lastUpsState) {		// state entry code
		if (tmpState != upsBoss.lastUpsState) 		
		{
			upsBoss.lastUpsState = upsBoss.upsState;
			time1 = time2 = getTime();	// reset timers
			addEvent("UPS State entered: default",9);
			updateAlm(UNBLANK_ALL);						// Put Display in normal state
		}
		upsBoss.upsState = UPS_FAULT;
		break;
	}
	#if (defined COM_ERROR_SHUTDOWN)
		if ( (upsOne.comErrors > 20) || (upsTwo.comErrors > 20) ) {
			if (upsBoss.upsState != UPS_COM_SHUTDOWN) {		// First time, or after reset
				upsBoss.upsState = UPS_COM_SHUTDOWN;
				addEvent("Communication Error Trip Level Reached",9);
			}
		}
	#endif
	if (timer(timeSlave, 15000)) {						// if state didn't take, keep trying
		timeSlave = getTime();
		masterCmdAdd(&upsCmd[RDAT20_SLAVE_ON][0], &upsTwo);// Set Slave UPS into Slave Mode
		if (upsBoss.checksumMode == CHKSUM_ON) {
			masterCmdAddBoth(&upsCmd[RDAT22_CHKSUM_ON][0]);
		} else {
			masterCmdAddBoth(&upsCmd[RDAT23_CHKSUM_OFF][0]);
		}
	}
	// Inverter Voltage Regulator adjustment to compensate for slave offset
	/*
		if ( (upsBoss.upsState == UPS_ON_UTIL) || (upsBoss.upsState == UPS_ON_BAT) ) {
			if (timer(timeInvReg, 10000)) {
				timeInvReg = getTime();
				fTemp = 230.0 - (upsOne.voltOut + upsTwo.voltOut);
				iTemp = (int) abs((fTemp * 10) + 0.5);
				//iTemp = iMin(iTemp, 10);
				iTemp = iMin(iTemp, 1);
				if (iTemp > 0) {		// don't send a correction if there isn't an error
					if (fTemp > 0) {
						sprintf((char *)upsBoss.StrTemp, "^RMODV%02d+", iTemp);
					}
					else {
						sprintf((char *)upsBoss.StrTemp, "^RMODV%02d-", iTemp);
					}
					masterCmdAdd(&upsBoss.StrTemp[0], &upsOne);
				}
			}
		}
	*/
	if ( (upsBoss.notifyMsg) && (strcmp((char *)upsBoss.msgStr,"^RBUT01") == 0) ) {	// "Test" button pressed
		addEvent("Test Button Pressed",9);
		if (lightshowFlag == FALSE) {		// only start new test if it isn't already running
			lightshowFlag = TRUE;
		}
	}
	if ( (upsOne.notifyMsg) && (strcmp((char *)upsOne.msgStr,"^RBUT31") == 0) ) {	// Master UPS Reports lightshow done
		addEvent("Master UPS Reports Lightshow done",9);
		lightshowFlag = lightshowLast = FALSE;
	}
	upsBoss.notifyMsg = upsOne.notifyMsg = upsTwo.notifyMsg = FALSE;

	//	__enable_interrupt();	// TODO - interrupt enable ups_state_controller
	DEBUG_PORT1_0;	// ####################
}

// returns a string conversion of an integer value, I've shortened it to 16 bit integer
// and added range check
char *itoa(volatile int i) {
	/* Room for INT_DIGITS digits, - and '\0' (19+2) */
	static volatile char buf[10];

//	ltoa((long) i, (char *) buf);
	sprintf( (char *) buf, "%ld", (long) i);

	return ((char *) &buf[0]);
}

// takes itoa and pads it to 3 characters with leading zeros
char *itoa3(volatile int i)
{
	static volatile char intBuf[10],buf[10];
	static volatile int bufLen;

	strcpy((char *) buf,NULL);					// clear previous stuff
	if ( (i >= 0)) {	// only process positive numbers, 3 numbers in length
		strcpy((char *) intBuf, itoa(i));
		bufLen = strlen((char *) intBuf);
		switch (bufLen) {
		case 1:
			strcpy((char *) buf, "00");
			break;
		case 2:
			strcpy((char *) buf, "0");
			break;
		case 3:										// buf is clear, no need to change for this
			break;
		default:
			strcpy((char *) intBuf, "000");		// number too big, reset to zero
			break;
		}
		strcat((char *) buf,(char *) intBuf);		// put pad in front of number
	} else {										// if negative
		strcpy((char *) buf, "000");				// return zero
	}

	return ((char *) buf);
}

// takes itoa and pads it to 3 characters with leading zeros
char *itoa6(volatile int i)
{
	static volatile char intBuf[10],buf[10];
	static volatile int bufLen;

	strcpy((char *) buf,NULL);					// clear previous stuff
	if ( (i >= 0)) {	// only process positive numbers, 3 numbers in length
		strcpy((char *) intBuf, itoa(i));
		bufLen = strlen((char *) intBuf);
		switch (bufLen) {
		case 1:
			strcpy((char *) buf, "00000");
			break;
		case 2:
			strcpy((char *) buf, "0000");
			break;
		case 3:										// buf is clear, no need to change for this
			strcpy((char *) buf, "000");
			break;
		case 4:
			strcpy((char *) buf, "00");
			break;
		case 5:
			strcpy((char *) buf, "0");
			break;
		case 6:										// buf is clear, no need to change for this
			break;
		default:
			strcpy((char *) intBuf, "000");		// number too big, reset to zero
			break;
		}
		strcat((char *) buf,(char *) intBuf);		// put pad in front of number
	} else {										// if negative
		strcpy((char *) buf, "000000");			// return zero
	}

	return ((char *) buf);
}

/*	// not done, may not be needed if sprintf works, but can't print '%' in sprintf
char *ftoa2(volatile float fval, volatile int decimal) {
	// Room for INT_DIGITS digits, - and '\0' (19+2)
	volatile char buf[10], buf2[10];
	volatile ltemp;

	ltemp = (long) (fval * 100.0);

//	ltoa(ltemp, (char *) buf);
        sprintf( (char *) buf, "%ld", ltemp);

	i = strlen((char *) buf);
	// xyz z		xyz
	buf[i+1] = buf[i-1];
	// xyz z		xyz
	buf[i+1] = buf[i-1];
	
	
	return ((char *) &buf[0]);
}
*/
#if BAT_CAP_METHOD==BAT_CAP_JOULE						// do local calculation for battery capacity
	void batCapJouleUpdate(volatile struct upsDataStrucT *upsData) {

        ups_states_t tmpState;
        float tmpfval1;
        long tmplval1;

		#if (!defined CHARGER_MONSTER)
			volatile float ftemp;
			volatile int itemp;
			static volatile ups_states_t lastUpsState = UPS_ON_UTIL;
			volatile long /* *FlashPointer, */ ltemp;
			static volatile struct timeT systemStartTime, systemStateTime;
			static volatile int systemStart = FALSE;
			
			if (systemStart == FALSE) {
				systemStart = TRUE;
				systemStartTime = systemStateTime = getTime();
			}
			
			#if UPS_STATE_CONTROL==TRUE							// this board controlling UPS modes
				switch(upsBoss.upsState) {
				case UPS_OFF:
				case UPS_ON_UTIL:
					tmpState = upsBoss.upsState;
					//if (upsBoss.upsState != lastUpsState) {		// state entry code
					if (tmpState != lastUpsState) 	
					{
						lastUpsState = upsBoss.upsState;
						systemStateTime = getTime();
					}
					#if (!defined SNMP_MIMIC)
						// accumulate charge joules
						//ftemp = upsData->voltBat * upsData->ampChg;
						tmpfval1 = upsData->voltBat;
						ftemp = tmpfval1 * upsData->ampChg;
						// used to balance charge/discharge Joule counts
						#if (defined BAT_JOULE_CHARGE_WEIGHTING)
							ftemp = ftemp * BAT_JOULE_CHARGE_WEIGHTING;
						#endif
					#else											// defined SNMP_MIMIC
						// If utility on, charger should be communicating
						if (charger.comOkay != FALSE) {			// charger is communicating, use info
							// accumulate charge joules

							//ftemp = upsData->voltBat * upsData->ampChg;
							tmpfval1 = upsData->voltBat;
							ftemp = tmpfval1 * upsData->ampChg;
							// used to balance charge/discharge Joule counts
							#if (defined BAT_JOULE_CHARGE_WEIGHTING)
								ftemp = ftemp * BAT_JOULE_CHARGE_WEIGHTING;
							#endif
						} else {
							ftemp = 0;                            // don't add anything
						}
					#endif											// !defined SNMP_MIMIC
					break;
				case UPS_ON_BAT:
					tmpState = upsBoss.upsState;
					//if (upsBoss.upsState != lastUpsState) {		// state entry code
					if (tmpState != lastUpsState) 	
					{
						lastUpsState = upsBoss.upsState;
						systemStateTime = getTime();
						// save joules to flash
					}
					// deaccumulate discharge joules, use negative joules
					//ftemp = -1.0 * (upsData->voltBat * upsData->ampBat);
					tmpfval1 = upsData->voltBat;
					ftemp = -1.0 * (tmpfval1 * upsData->ampBat);
					break;
				case UPS_INIT:
				    ftemp = 0;
				    break;
				}
			#else												// UPS1 controlling it's own modes
				switch(upsOne.dcMode) {
				case AUTO_OFF:
				case MANUAL_OFF:
					if (upsBoss.upsState != lastUpsState) {		// state entry code
						lastUpsState = upsBoss.upsState;
						systemStateTime = getTime();
					}
					// accumulate charge joules
					#if (!defined THALES_3KVA)
						ftemp = upsData->voltBat * upsData->ampChg;
					#else	// defined THALES_3KVA
						ftemp = upsTwo.voltBat * upsTwo.ampChg;
					#endif
					// used to balance charge/discharge Joule counts
					#if (defined BAT_JOULE_CHARGE_WEIGHTING)
						ftemp = ftemp * BAT_JOULE_CHARGE_WEIGHTING;
					#endif
					break;
				case AUTO_ON:
				case MANUAL_ON:
					if (upsBoss.upsState != lastUpsState) {		// state entry code
						lastUpsState = upsBoss.upsState;
						systemStateTime = getTime();
						// save joules to flash
					}
					// deaccumulate discharge joules, use negative joules
					#if (!defined THALES_3KVA)
						ftemp = -1.0 * (upsData->voltBat * upsData->ampBat);
					#else	// defined THALES_3KVA
						ftemp = -1.0 * (upsOne.voltBat * upsOne.ampBat);
					#endif
					break;
				case UPS_INIT:
				    ftemp = 0;
				    break;
				}
			#endif
			//upsData->batJouleFraction += ftemp;				// add or subtract joules this second
			tmpfval1 = ftemp;
			upsData->batJouleFraction += tmpfval1;				// add or subtract joules this second

			ltemp = (long) upsData->batJouleFraction;			// convert whole number
			//upsData->batJoule += ltemp;						// add or subtract whole joule
			tmplval1 = ltemp;
			upsData->batJoule += tmplval1;
			//upsData->batJouleFraction -= (float) ltemp;		// update fractional joules for next calc, remove int
			upsData->batJouleFraction -= (float) tmplval1;

			upsData->batJoule = lMin(upsData->batJoule,BAT_MAX_JOULE);// limit to maximum charge
			upsData->batJoule = lMax(upsData->batJoule,0);	// no negative joules
			// COMPILER FLAG for inclusion of the SNMP MIMIC functions getting info from charger model F
			#if (!defined SNMP_MIMIC)
				// if battery joules very low in Flash and charger goes to slow assume at least 80% charge
				if ( ((upsBoss.upsState == UPS_ON_UTIL) || (upsBoss.upsState == UPS_OFF))
					&& ((upsBoss.chgMode == AUTO_SLOW) || (upsBoss.chgMode == MANUAL_SLOW))
					&& (timer(systemStateTime, 300000))
					&& (upsData->dcMode != AUTO_ON) )			// prevent joule reset when shutting down from battery
				{
					systemStateTime = getTime();
					//upsData->batJoule = (long) fMax ((float)upsData->batJoule, (0.8 * (float)upsData->batJouleNom));
					tmplval1 = upsData->batJoule;
					upsData->batJoule = (long) fMax ((float)tmplval1, (0.8 * (float)upsData->batJouleNom));
				}
				//ftemp = ((float)upsData->batJoule/(float)upsData->batJouleNom)*100.0;
				tmplval1 = upsData->batJoule;
				ftemp = ((float)tmplval1/(float)upsData->batJouleNom)*100.0;
				// Only show 100% if fully charged and charger in slow mode
				if ( (upsBoss.chgMode == AUTO_FAST) || (upsBoss.chgMode == MANUAL_FAST) ){
					upsData->batChgPct = fMin(ftemp, 99.0);	// no higher than 99%
				} else {
					upsData->batChgPct = ftemp;
				}
			#else												// defined SNMP_MIMIC
				// if battery joules very low in Flash and charger goes to slow assume at least 80% charge
				if ( ((upsBoss.upsState == UPS_ON_UTIL) || (upsBoss.upsState == UPS_OFF))
					&& (upsData->ampChg <= (SNMP_MIMIC_CHARGER_THRESHOLD * 0.75))
					&& (timer(systemStateTime, 300000)) )
				{
					systemStateTime = getTime();
					// only allow this to happen if getting information from charger
					if (charger.comOkay == TRUE) 
					{
					    //upsData->batJoule = (long) fMax ((float)upsData->batJoule, (0.8 * (float)upsData->batJouleNom));
						tmplval1 = upsData->batJoule;
						upsData->batJoule = (long) fMax ((float)tmplval1, (0.8 * (float)upsData->batJouleNom));
					}
				}
				//ftemp = ((float)upsData->batJoule/(float)upsData->batJouleNom)*100.0;
				tmplval1 = upsData->batJoule;
				ftemp = ((float)tmplval1/(float)upsData->batJouleNom)*100.0;
				if (upsBoss.ampChg >= SNMP_MIMIC_CHARGER_THRESHOLD) 	// greater means fast charge
				{				
					upsData->batChgPct = fMin(ftemp, 99.0);	// no higher than 99%
				} 
				else 
				{
					upsData->batChgPct = ftemp;
				}
			#endif
			#if ((!defined THALES_3KVA) || (DUAL_BOARD==TRUE))
			    //upsBoss.batChgPct = fMin(upsOne.batChgPct,upsTwo.batChgPct);    // Older code

                //upsBoss.batJoule = upsOne.batJoule + upsTwo.batJoule;
				tmplval1 = upsOne.batJoule;
				upsBoss.batJoule = tmplval1 + upsTwo.batJoule;
				tmplval1 = upsBoss.batJoule;
				//upsBoss.batChgPct = ((float)upsBoss.batJoule/(float)upsBoss.batJouleNom)*100.0;  
				upsBoss.batChgPct = ((float)tmplval1/(float)upsBoss.batJouleNom)*100.0;
			#else // defined THALES_3KVA or single board
				upsBoss.batChgPct = upsOne.batChgPct;
			#endif
			if (upsData->powOut >= 50.0) 
			{
			    //upsData->estSecBat = (long)(upsData->batJoule / upsData->powOut);	// calculate remaining runtime in seconds
				tmplval1 = upsData->batJoule;
				upsData->estSecBat = (long)(tmplval1 / upsData->powOut);
			} 
			else 
			{										// don't divide by zero
				upsData->estSecBat = (long)(upsData->batJoule / 50.0);// calculate remaining runtime in seconds
			}
			#if ((!defined THALES_3KVA) || (DUAL_BOARD==TRUE))
			    //upsBoss.estSecBat = lMin(upsOne.estSecBat,upsTwo.estSecBat);	// min
				tmplval1 = upsOne.estSecBat;
				upsBoss.estSecBat = lMin(tmplval1,upsTwo.estSecBat);

			#else // defined THALES_3KVA
				upsBoss.estSecBat = upsOne.estSecBat;			// only use massaged upsOne joules
			#endif
		#else													// defined CHARGER_MONSTER
			volatile float ftemp;

			ftemp = ((float)upsData->batJoule/(float)upsData->batJouleNom)*100.0;
			// Only show 100% if fully charged and charger in slow mode
			if ( (upsBoss.chgMode == AUTO_FAST) || (upsBoss.chgMode == MANUAL_FAST) ){
				upsData->batChgPct = fMin(ftemp, 99.0);	// no higher than 99%
			} else {
				upsData->batChgPct = ftemp;
			}
			upsBoss.batChgPct = fMin(upsOne.batChgPct,upsTwo.batChgPct);
		#endif
	}
#endif

#if defined COM_ETI													// compiler switch for ETI data and functions
//*****************************************************************************************

/*
 * FILE: 		eti_check.c
 *
 * TARGET: 		MSP430x5419x
 *
 * AUTHOR: 		Argil Shaver
 *
 * DESCRIPTION:	Implements the ETI protocol which
 * 				will be used across the SNMP serial
 * 				port on the LCD Serial Router Board.
 *
 * 				This code will be merged into the
 * 				main.c code to fully run on the LCD
 * 				Serial Router Board.
 *
 * DATE:		21 May 2014
 *
 * VERSION:		1.0.2
 *
 * HISTORY:		1.0.0	Initial Version	18 April 2014
 * 				1.0.1	21 May 2014
 * 					Made modifications to use pointers like SNMP and UPSILON
 * 					Added a flag = FALSE statement to control erratic behavior
 * 					Cleaned up the formatting and fixed one error
 * 					Added several commands for EXTENDED operation parameter gathering over the "ETI" protocol
 * 				1.0.2	11 July 2014
 * 					Added requested User Command Extensions. Hid the extensions used by IntelliPower from
 * 					the help screen. Using RBUT for alarm relay testing.
 *
*/

/* A brief outline of the ETI Protocol
 *
 * This protocol converter will use the same serial hardware port as the SNMT and UPSILON links.
 * Communication occurs at 8-bit, 9600 baud, no parity, and one stop bit though a DE-9
 * connector on the back of the unit..
 *
 * If no alarms are present upon power up the interface will report the following Event Status:
 *
 * #IF0 OB0 LB0 CH1 BP0 100 30<CR>
 *
 * #   is the beginning of message frame
 * IF0 is the Input Fail 0, (AC input is OK, 1 indicates failure)
 * OB0 is the On Battery 0, (UPS is not running on battery, 1 indicates UPS on battery)
 * LB0 is the Low Battery 0, (UPS battery is not low, 1 indicates battery is low)
 * CH1 is the Battery CHarger 1, (battery is charging, 0 indicates no charging of battery)
 * BP0 is the ByPass not activated 0, (UPS bypass not activated, 1 indicates a Bypass condition)
 * 100 is the Battery percent available. Values are: 100, 075, 050, 025, 001, 000
 * 30  is the Approximate battery time remaining in minutes. Values are: 30, 22, 15, 12, 01, 00
 *
 * Any change in the Alarm Status or the Time Remaining (on battery) will produce an Event Status
 * report to be transmitted on the RS-232 channel.
 *
 * USER COMMANDS- User Commands (not case sensitive) and are 4 characters long. The commands below
 * are activated by issuing the first 3 characters of the command followed by a Carriage Return.
 *
 * USER INPUT RESULT
 * es1<CR>	will return:
 * 						#IFx OBx LBx CHx BPx yyy zz<CR>
 *
 * sd1<CR> will shut down the UPS, but only when on battery operation
 *
 * sw1<CR>	will return:
 * 							<model from boss> <software version from boss>
 * 							Copyright IntelliPower, Inc.
 *
 * help or HELP or ????		will return:
 * 						IntelliPower, Inc.
 * 						Voice: (714) 921-1580 Fax: (714) 921-4023
 * 						INTERNET: http://www.intellipower.com
 * 						ES1<CR> - Request Major Status Report
 * 						SD1<CR> - Remote Shutdown (only when on battery)
 * 						SW1<CR> - Request Software Version ID
 * 						GARN	- General Alarm Relay Normal
 * 						GART	- General Alarm Relay ON
 * 						BARN	- Battery Alarm Relay Normal
 * 						BART	- Battery Alarm Relay ON
 * 						HELP or ???? - Display this Menu
 *
 *					// These are internal to IntelliPower and not displayed in HELP menu
 * 						XALM- Extended Alarm Status (non standard command)
 * 						XBAT- Extended Battery Status (non standard command)
 * 						XINP- Extended Input Status (non standard command)
 * 						XOUT- Extended Output Status (non standard command)
 * 						XTMP- Extended Temperature Status (non standard command)
*/

// ETI definitions
	#define ETI_MSG_LEN 16
	#define ETI_STR_MAX 128
	#define MAX_ETI_COMMAND 4
	#define ETI_MODE_STR_MAX 18
	#define testTimeBatteryAlarmRelay (long) 3 * 60 * 1000		// Time to wait for the Battery Alarm Relay Test
	#define testTimeGeneralAlarmRelay (long) 3 * 60 * 1000		// Time to wait for the General Alarm Relay Test
	#define trayAlertTime  5000L								// Time to wait before announcing that the Battery Tray is pulled again (5 seconds)

	char *etiModeStr [ETI_MODE_STR_MAX] = {
											"ES1",				// Status Update request
											"SD1",				// Shutdown command for UPS
											"SW1",				// Software Version Request
											"HELP",				// Help
											"????",
											"XALM",				// ** Extended - report alarm   conditions
											"XBAT",				// ** Extended - report battery conditions
											"XBYP",				// ** Extended - Bypass Mode for Technicians
											"XINP",				// ** Extended - report input   conditions
											"XMOD",				// ** Extended - toggle output modes
											"XOUT",				// ** Extended - report output  conditions
											"XTMP",				// ** Extended - report temperatures
											"GARN",				// User Extension for General Alarm Relay Normal Operation
											"GART",				// User Extension for General Alarm Relay Test On
											"BARN",				// User Extension for Battery Alarm Relay Normal Operation
											"BART",				// User Extension for Battery Alarm Relay Test On
											"%",
											")"
                                   	   };

	typedef enum {
		STATUS,
		SHUTDOWN,
		SWVERSION,
		HELP,
		QUESTIONMARK,
		EXTENDED_ALARMS,
		EXTENDED_BATTERY,
		EXTENDED_BYPASSMODE,
		EXTENDED_INPUT,
		EXTENDED_MODE_TOGGLE,
		EXTENDED_OUTPUT,
		EXTENDED_TEMP,
		GENERAL_ALARM_RELAY_NORMAL,
		GENERAL_ALARM_RELAY_TEST,
		BATTERY_ALARM_RELAY_NORMAL,
		BATTERY_ALARM_RELAY_TEST,
		PERCENT_COMMAND,
		CLOSE_PARAN_COMMAND,
		NULL_ETI = -1		// Command not found or invalid
	} etiCommandT;
	
	typedef enum {
		ETI_BYPASS,			// Bypass Mode for Technicians
		ETI_INIT,			// First Start and must send Status Report
		ETI_IDLE,			// Waiting for something to happen
		ETI_WAITING,		// Waiting for message to complete
		ETI_RESPONSE,		// Decode and respond to the message
		ETI_HELP,			// First  part of help display
		ETI_HELP1,			// Second part of help display
		ETI_HELP2,			// Third  part of help display
		ETI_HELP3,			// Fourth part of the help display
		ETI_HELP4,			// Fifth  part of the help display
		ETI_HELP5,			// Finish off the help display
		ETI_AUTO			// Special case for Automatic Status Report
	} etiComStateT;
	
	// ETI Data Structure for the LCD Serial Router Board
	struct etiDataStruct {
		int  Port, etiBypassUps;							// Serial Port we connect to and a Bypass Mode Port
		char rcv [ETI_MSG_LEN], response [ETI_STR_MAX];		// Strings for protocol use both RX and TX
		int  posRCV;										// Position holders for strings
		int  dataCount;										// Number of Received characters
		bool Bootup;										// Boolean Flag for for Starting Condition infinite loop break
		bool setBatteryTime;								// Boolean Flag to force Battery Time Update to full until UPS_ON_BAT
		bool BypassMode;									// Boolean Flag to allow technicians to access individual Model F Boards for testing
		etiComStateT comState, lastComState;				// Communications State and Last State
		etiCommandT  cmd;									// Translated Command
		bool upsUtilityOK, upsBatteryOps;					// Flags to reflect calculated UPS values for ETI
		bool flagInputFail, flagOnBattery, flagLowBattery, flagCharging, flagBypass;	// Boolean Flags for state of Input, On Battery, Low Battery, Charging, and Bypass
		bool BatteryAlarmTest, GeneralAlarmTest;			// Boolean Flags for User Extension for Output Relay tests
		bool modeSelection;
		int  pctBat, lastPctBat, timeBat, lastTimeBat;		// Battery statistics that we use here
		struct timeT timeGeneralTest, timeBatteryTest;
	} ;
	
	// Our data structure in RAM
	volatile struct etiDataStruct eti, *pEti;				// Updated using pointers like SNMP and UPSILON
	
	/*
	 * Name:			etiSelectCommand
	 * Description:		Selects the proper decoded command
	 * Parameter(s):	string: A string of the received request
	 * Returns:			etiCommandT: The coded command response
	 */
	etiCommandT etiSelectCommand (char *string) {
		int i;
		etiCommandT result;
		result = NULL_ETI;								// Command not found
		for (i=0; i<ETI_MODE_STR_MAX; i++) {
			if (strcmp (string, etiModeStr [i]) == 0) {	// returns 0 when equal
				result = (etiCommandT) i;				// if same string, return index of match
			} 											// End of IF compare
		} 												// End of FOR search
		return result;									// and the winner is...
	} 													// End of etiSelectCommand
	
	/*
	 * Name:			initETIccom
	 * Description:		Sets up the ETI Data Structure Variable for Initial Use
	 * Parameter(s):	etiDataStruct *parseType
	 * Returns:			NONE
	 */
	void initETIcom (volatile struct etiDataStruct *parseType) {
		strcpy ((char *) parseType->response, NULL);				// Initialize Response string
		strcpy ((char *) parseType->rcv, NULL);						// Initialize Receive String
		parseType->Bootup = TRUE;									// Force the bootup message
		parseType->setBatteryTime = TRUE;
		parseType->BypassMode = FALSE;
		parseType->comState = ETI_INIT;								// Booting up the system
		parseType->lastComState = ETI_BYPASS;						// Force a state change action
		parseType->Port = ETI_PORT;									// Set our Serial Communications Port
		parseType->dataCount = 0;									// No data yet, so it must be zero
		parseType->posRCV = 0;
		parseType->flagInputFail = parseType->flagOnBattery = FALSE;
		parseType->flagLowBattery = FALSE;							// Preset conditions to FALSE
		parseType->flagCharging = parseType->flagBypass = FALSE;
		parseType->upsUtilityOK = parseType->upsBatteryOps = FALSE;
		parseType->timeBat = parseType->lastTimeBat = 0;			// Preset values to zero
		parseType->pctBat = parseType->lastPctBat = 0;
		parseType->cmd = NULL_ETI;									// Illegal command to start with
		parseType->BatteryAlarmTest = FALSE;
		parseType->GeneralAlarmTest = FALSE;
		parseType->modeSelection = MOCK_ETI_OUTPUT;
		usart_putstr ("^RDAT22", upsOne.port);						// Checksum on Master UPS
		usart_putstr ("^RDAT22", upsTwo.port);						// Checksum on Slave  UPS
		usart_putstr ("Model ", parseType->Port);
		usart_putstr ((char *) upsBoss.model, parseType->Port);
		usart_putstr (" Version ", parseType->Port);
		usart_putstr ((char *) upsBoss.verSoftware, parseType->Port);
		usart_putstr ("\r\nCopyright IntelliPower, Inc.\r\n", parseType->Port);
	} // End of initETIcom
	
	/*
	 * Name:			etiStatusChange
	 * Description:		Looks at monitored parameters and sets a flag if something has changed
	 * Parameter(s):	upsDataStrucT *upsData, to access the ups data with passed pointer
	 * 					etiDataStruct *parseType, to access and modify the ETI data structure
	 * Returns:			Boolean of if something change, TRUE is Yes and FALSE is no
	 */
	char etiStatusCheck (volatile struct upsDataStrucT *upsData, volatile struct etiDataStruct *parseType) {
		bool				notifyFlag, flag, test, test1, okUtil = FALSE;
		static bool			firstTray = TRUE;
		static float		notifyBatPercentage = 0.0;
		long				batTimeValue;
		float				tempBatPer;
		static long			notifyBatTime = 0L;

		flag = parseType->Bootup;																	// Had to add this here for some strange reason
		notifyFlag = test = test1 = FALSE;
		okUtil = FALSE;
		if (batteryTrayStatus) {																	// Battery Tray Pulled check
			parseType->flagInputFail = parseType->flagOnBattery = FALSE;							// Set Input Fault and not on battery (because there is none)
			parseType->flagCharging = FALSE;														// No Battery so no charging...
			parseType->flagLowBattery = TRUE;														// Set Low Battery Alarm
			parseType->pctBat = parseType->timeBat = 0;												// Set Battery Percent and Time Remaining to zero as there are no batteries
			if (firstTray) {																		// This is to prevent the compiler from bitching at us...
				firstTray = FALSE;																	// We are not virgin anymore
				flag = TRUE;																		// Announce this to the world
			}																						// End of do we announce check
		} else {																					// Otherwise...
			okUtil = (bool) (fRange (upsData->voltIn, 80.0, 145.0) && fRange (upsData->freqIn, 45.9, 69.9));	// Is AC Input within bounds?
			#if DUAL_BOARD																			// Dual board requires a little extra work...
				parseType->upsBatteryOps = ((upsOne.dcMode == AUTO_ON) || (upsOne.dcMode == MANUAL_ON) ||
											(upsTwo.dcMode == AUTO_ON) || (upsTwo.dcMode == MANUAL_ON));
				okUtil = (okUtil && ((upsOne.dcMode == AUTO_OFF) || (upsTwo.dcMode == AUTO_OFF)));
				parseType->upsUtilityOK = (((parseType->upsBatteryOps == FALSE) && (autoStart == 1)) || okUtil);
			#else
				parseType->upsBatteryOps = (bool) ((upsData->dcMode == AUTO_ON) || (upsData->dcMode == MANUAL_ON));
				okUtil = (okUtil && (upsData->dcMode == AUTO_OFF));
				parseType->upsUtilityOK = (bool) (((parseType->upsBatteryOps == FALSE) && (autoStart == 1)) || okUtil);
			#endif
			// If upsState and flagInputFail are different set flag. Update flagInputFail Status
			test = parseType->flagInputFail;														// Do we think we are on Utility?
			test1 = parseType->upsUtilityOK;														// Is the system on Utility?
			if (test1 ^= test) {																	// If we disagree, then update ourself and set update flag
				flag = TRUE;
				parseType->flagInputFail = parseType->upsUtilityOK;
			}																						// Exclusive-OR the results... a difference means update
			// If upsState and flagOnBattery are different set flag. Update flagOnBattery Status
			test = parseType->flagOnBattery;														// Do we think we are on Battery?
			test1 = parseType->upsBatteryOps;														// Is the system on Battery?
			if (test1 ^= test) {																	// If we disagree, then update ourself and set update flag
				flag = TRUE;
				parseType->flagOnBattery = parseType->upsBatteryOps;
			}																						// Exclusive-OR the results... a difference means update
			// If batSts and flagLowBattery are different set flag. Update flagLowBattery Status
			if ((upsOne.batSts == 1) || (upsTwo.batSts == 1)) {
				test = TRUE;
			} else {
				test = FALSE;
			}
			if (parseType->flagLowBattery != test) {
				flag = TRUE;
				parseType->flagLowBattery = test;
			}
			// Are we in any Charge Mode and are running on Utility?
			// If chgMode and flagCharging are different set flag. Update flagCharging Status
			test = parseType->flagCharging;															// Do we think we are Charging?
			#if DUAL_BOARD
				test1 = ((upsOne.chgModeSnmp == 1) || (upsOne.chgModeSnmp == 2)) && ((upsTwo.chgModeSnmp == 1) || (upsTwo.chgModeSnmp == 2));
			#else
				test1 = ((upsData->chgModeSnmp == 1) || (upsData->chgModeSnmp == 2));
			#endif
			test1 = test1 & (parseType->upsUtilityOK);												// Is the system Charging?
			parseType->flagCharging = test1;														// Because XOR is destructive
			if (test1 ^= test) {																	// If we disagree, then update ourself and set update flag
				flag = TRUE;
			}																						// Exclusive-OR the results... a difference means update
			// If UPS_BYPASS and sBP are different set flag. Update flagBypass Status
			test = parseType->flagBypass;															// Do we think we are on Bypass?
			test1 = upsData->bypassMode == ON;														// Is the system on Bypass?
			if (test1 ^= test) {																	// If we disagree, then update ourself and set update flag
				flag = TRUE;
				parseType->flagBypass = (upsData->bypassMode == ON);
			}																						// Exclusive-OR the results... a difference means update
			// Do the final adjustments based on dumb down or not...
			#if defined ETI_SCALE_BY_1
				tempBatPer = upsData->batChgPct;													// Re-scale for use based on micro reporting
			#else
				tempBatPer = upsData->batChgPct * 10.0;												// Re-scale for use based on micro reporting
			#endif
			if (parseType->modeSelection) {															// Do we dumb down?
				if (tempBatPer <= 5.0) {															// Are we at the low end of the pool?
					if (parseType->upsBatteryOps) {													// Are we on Battery?
						if (upsData->estSecBat <= 121L) {											// Check estimated seconds remaining to get things closer as battery percent is reported in 5% steps
							tempBatPer = 1.0;														// Less than two minutes estimated... give a one percent warning
							if (upsData->estSecBat <= 61L) {										// Less than a minute estimated?
								tempBatPer = 0.0;													// Announce over and out of capacity before it happens
							}																		// End of 0% check
						}																			// End of 1% check
					} else {																		// Otherwise we are charing up...
						tempBatPer = 1.0;															// Dumb and not on battery so hold at 1% until we increase battery charge
					}																				// End of on battery adjustment and check
				} else if (tempBatPer < 25.0) {														// 25% check
					tempBatPer = 25.0;																//
				} else if (tempBatPer < 50.0) {														// 50% check
					tempBatPer = 50.0;																//
				} else if (tempBatPer < 75.0) {														// 75% check
					tempBatPer = 75.0;																//
				} else {																			// Otherwise...
					tempBatPer = 100.0;																// 100%
				}																					// End of the dumb battery percentage checks
			}																						// End of Dumb It Down
			parseType->pctBat = (int) tempBatPer;													// Load value for the announcement
			notifyFlag  = (fabs (notifyBatPercentage - tempBatPer) >= 5.0);							// Test for a 5% Battery Percentage change since last time through
			notifyFlag |= ((tempBatPer != notifyBatPercentage) && (tempBatPer < 5.0));				// OR if less than 5% and the value changed
			if (notifyFlag) {																		// Check to see if we make an announcement
				notifyBatPercentage = tempBatPer;													// Yes, update the value
				notifyFlag = FALSE;																	// Clear this flag
				firstTray = TRUE;																	// Tray not pulled so we get to be a virgin again
				return TRUE;																		// To match ETI, we return and report... next up is time which is separate reporting
			}																						// End of Battery Percentage change reporting check
			if (parseType->upsBatteryOps) {															// Are we on Batteries?
				batTimeValue = upsData->estSecBat;													// Yes, so Estimated Seconds on Battery is valid... use it
				if (parseType->modeSelection) {														// Do we have to dumb ourselves down?
					parseType->timeBat = (int) (((float) parseType->pctBat * UPS_TIMELOBATNOM / 100.0) + 0.6);	// Make the gross adjustment
					batTimeValue = (long) parseType->timeBat * 60L;									// Override batTimeValue for this crap method
				} else {																			// Otherwise...
					parseType->timeBat = (int) (batTimeValue / 60L);								// Load the adjusted value into the memory structure for reporting
				}																					// End of battery time display selection
				notifyFlag  = (labs (notifyBatTime - batTimeValue) >= ((long) (UPS_TIMELOBATNOM * 0.05 * 60.0)));				// Has it changed by 5% of value?
				notifyFlag |= ((batTimeValue != notifyBatTime) && (batTimeValue < (long) (UPS_TIMELOBATNOM * 0.05 * 60.0)));	// Is it less than 5% but value changed?
				if (notifyFlag) {																	// Check to see if we make an announcement
					notifyBatTime = batTimeValue;													// Yes, update the value
					notifyFlag = FALSE;																// Clear this flag
					flag = TRUE;																	// Set the flag to report the change
				}																					// End of Battery Time Remaining change reporting check
			} else {																				// Otherwise...
				#if defined ETI_SCALE_BY_1
				batTimeValue = (long) (upsData->batChgPct * UPS_TIMELOBATNOM / 100.0);				// Link estimated time to the battery charge...
				#else
					batTimeValue = (long) (upsData->batChgPct * UPS_TIMELOBATNOM / 10.0);			// Link estimated time to the battery charge...
				#endif
				if (batTimeValue != notifyBatTime) {												// If the value has changed
					notifyBatTime = batTimeValue;													// Update the check value
					parseType->timeBat = (int) batTimeValue;										// Update the value to display
					flag = TRUE;																	// Set the announcement flag
				}																					// End of change check
			}																						// End of Battery or Normal Operations check for time indication
			firstTray = TRUE;																		// Tray not pulled so we get to be a virgin again
		}																							// End of Battery Tray Pulled check
		return flag;																				// Return the flag indicating further action or not
	}																								// End of etiStatusCheck
	
	/*
	 * Name:			extendedResponses
	 * Description:		Forms the protocol message for extended response and sends it
	 * Parameter(s):	upsDataStrucT *upsData, to access the ups data with passed pointer
	 * 					etiDataStruct *parseType, to access and modify the ETI data structure
	 * Returns:			NONE
	 */
	void extendedResponses (volatile struct upsDataStrucT *upsData, volatile struct etiDataStruct *parseType) {
		strcpy ((char *) parseType->response, NULL);										// Make sure we are clear
		// WARNING: Must be careful with the string length, both for response string and the UART buffer !!!!!!!!!!!!!!!!!!!!!!!!
		switch (parseType->cmd) {															// Process the command now
		case EXTENDED_BATTERY:																// Extended Status parameters
			sprintf ((char *) parseType->response, "BatVolt=%1.2f BatChgPct=%1.1f EstSecOnBat=%d SecOnBat=%d BatCond=",
					 upsData->voltBat, upsData->batChgPct * 10.0, (int) upsData->estSecBat, (int) upsData->secOnBat);
			switch (upsData->batCond) {
			case NORMAL:
				strcat ((char *) parseType->response, "NORMAL");
				break;
			case WARN:
				strcat ((char *) parseType->response, "WARN");
				break;
			case OVER_VOLTAGE:
				strcat ((char *) parseType->response, "OVER_VOLTAGE");
				break;
			case FAULT:
				strcat ((char *) parseType->response, "FAULT");
				break;
			default:
				strcat ((char *) parseType->response, "UNKNOWN");
				break;
			}
			break;
		case EXTENDED_INPUT:																// Extended Input parameters
			sprintf ((char *) parseType->response, "Volts=%1.1f Freq=%1.1f AmpChg=%1.2f ChgMode=%d",
					upsData->voltIn, upsData->freqIn, upsData->ampChg, upsData->chgModeSnmp);
			break;
		case EXTENDED_MODE_TOGGLE:
			parseType->modeSelection = !parseType->modeSelection;
			if (parseType->modeSelection) {
				strcpy ((char *) parseType->response, "Reduced Resolution Output");
			} else {
				strcpy ((char *) parseType->response, "Normal Output");
			}
			break;
		case EXTENDED_OUTPUT:																// Extended Output parameters
			sprintf ((char *) parseType->response, "Volts=%1.1f Amps=%1.1f Freq=%1.1f PF=%1.2f PctLoad=%1.1f",
					upsData->voltOut, upsData->ampOut, upsData->freqOut, upsData->pfOut, upsData->loadPctOut);
			break;
		case EXTENDED_TEMP:																	// Extended Temperature parameters
			sprintf ((char *) parseType->response, "Ambient=%1.1fC Heatsink=%1.1fC",
					upsData->tAmb, upsData->tSink);
			break;
		case EXTENDED_ALARMS:																// Extended Alarm parameters
			strcpy ((char *) parseType->response, "BatCond=");
			switch (upsData->batCond) {
			case NORMAL:
				strcat ((char *) parseType->response, "NORMAL ");
				break;
			case WARN:
				strcat ((char *) parseType->response, "WARN ");
				break;
			case OVER_VOLTAGE:
				strcat ((char *) parseType->response, "OVER_VOLTAGE ");
				break;
			case FAULT:
				strcat ((char *) parseType->response, "FAULT ");
				break;
			default:
				strcat ((char *) parseType->response, "UNKNOWN ");
				break;
			}																				// End SWITCH of batCond
			strcat ((char *) parseType->response, "InvFault=");
			if (upsData->invFaultAlm == ON_ALARM) {
				strcat ((char *) parseType->response, "ON_ALARM ");
			} else if (upsData->invFaultAlm == OFF_ALARM) {
				strcat ((char *) parseType->response, "OFF_ALARM ");
			}
			strcat ((char *) parseType->response, "AmbMode=");
			if ((upsOne.tAmbMode == ON_ALARM) || (upsTwo.tAmbMode == ON_ALARM)) {
				strcat ((char *) parseType->response, "ON_ALARM");
			} else  {
				strcat ((char *) parseType->response, "NORMAL");
			}
			break;
		default:																			// Should never get here, but catch it just in case
			strcpy ((char *) parseType->response, "ERROR IN COMMAND");
			break;
		} 																					// End of SWITCH on EXTENDED Commands
		strcat ((char *) parseType->response, "\r\n");										// <CR><LF> terminate the string for passage
		usart_putstr (parseType->response, parseType->Port);								// Transmit
		strcpy ((char *) parseType->response, NULL);										// Make sure we are clear
	} 																						// End of Extended Responses

	/*
	 * Name:			etiSatusResponse
	 * Description:		Forms the protocol message for status and sends it
	 * Parameter(s):	etiDataStruct *parseType
	 * Returns:			NONE
	 */
	void etiStatusResponse (volatile struct etiDataStruct *parseType) {
		strcpy ((char *) parseType->response, NULL);										// Make sure we are clear
		if (parseType->flagInputFail) {														// True if Input is good
			strcpy ((char *) parseType->response, "#IF0 ");
		} else {
			strcpy ((char *) parseType->response, "#IF1 ");
		}
		if (parseType->flagOnBattery) {														// True if On Battery
			strcat ((char *) parseType->response, "OB1 ");
		} else {
			strcat ((char *) parseType->response, "OB0 ");
		}
		if (parseType->flagLowBattery) {													// True if Low Battery
			strcat ((char *) parseType->response, "LB1 ");
		} else {
			strcat ((char *) parseType->response, "LB0 ");
		}
		if (parseType->flagCharging) {														// True if Charging
			strcat ((char *) parseType->response, "CH1 ");
		} else {
			strcat ((char *) parseType->response, "CH0 ");
		}
		if (parseType->flagBypass) {														// True if On Bypass
			strcat ((char *) parseType->response, "BP1 ");
		} else {
			strcat ((char *) parseType->response, "BP0 ");
		}
		strcat ((char *) parseType->response, (char *) itoa3 (parseType->pctBat));
		strcat ((char *) parseType->response, " ");											// Space delimiter
		if (parseType->timeBat < 10) {
			strcat ((char *) parseType->response, "0");										// If less than 10 (single digits) add a leading zero
		}
		strcat ((char *) parseType->response, (char *) itoa (parseType->timeBat));
		strcat ((char *) parseType->response, "\r\n");										// <CR><LF> terminate the string for passage
		usart_putstr (parseType->response, parseType->Port);								// Transmit
		strcpy ((char *) parseType->response, NULL);										// Reset communication parameters
	}
	
	/*
	 * Name:			ETIccom
	 * Description:		Processes the ETI Protocol and Interfaces with the UPS System
	 * 					through the defined serial port.
	 * Parameter(s):	upsDataStrucT *upsData, what UPS system data to use
	 * 					etiDataStruct *parseType, pointer to our own data structure
	 * Returns:			NONE
	 */
	void ETIcom (volatile struct upsDataStrucT *upsData, volatile struct etiDataStruct *parseType) {
		volatile char newChar;
		volatile long tempSOB1, tempSOB2;
		static volatile struct timeT	updateData;

		if ((parseType->BatteryAlarmTest) && (timer(parseType->timeBatteryTest, testTimeBatteryAlarmRelay))) {
			parseType->BatteryAlarmTest = FALSE;
			if (!parseType->flagLowBattery) {
				strcpy ((char *) parseType->response, "System BAR Normal\r\n");
				masterCmdAdd ("^RBUT23", &upsOne);
			} else {
				strcpy ((char *) parseType->response, "System BAR Alarm\r\n");
			}
			usart_putstr (parseType->response, parseType->Port);
			strcpy ((char *) parseType->response, NULL);															// Reset communication parameters
		}
		if ((parseType->GeneralAlarmTest) && (timer(parseType->timeGeneralTest, testTimeGeneralAlarmRelay))) {
			parseType->GeneralAlarmTest = FALSE;
			if (parseType->flagInputFail) {
				strcpy ((char *) parseType->response, "System GAR Normal\r\n");
				masterCmdAdd ("^RBUT21", &upsOne);
			} else {
				strcpy ((char *) parseType->response, "System GAR Alarm\r\n");
			}
			usart_putstr (parseType->response, parseType->Port);
			strcpy ((char *) parseType->response, NULL);															// Reset communication parameters
		}

		switch (parseType->comState) {
		case ETI_BYPASS:
			if (usart_tx_buffer_count (parseType->Port) != 0) {break;}												// Hold this state until TX Buffer empty
			if (parseType->comState != parseType->lastComState) {													// State entry code
				parseType->lastComState = parseType->comState;														// Update states
				parseType->etiBypassUps = UPS1_PORT;																// Select master
				usart_putstr  ("^RDAT23", upsOne.port);																// checksum off Master
				usart_putstr  ("^X",      upsOne.port);																// exit SNMP on Model F Board
				usart_putstr  ("^RDAT23", upsTwo.port);																// checksum off Slave
				usart_putstr  ("^X",      upsTwo.port);																// exit SNMP on Model F Board
				usart_putchar (13, parseType->etiBypassUps);														// pass on char to selected UPS
				usart_putchar (10, parseType->etiBypassUps);														// pass on char to selected UPS

				strcpy ((char *) parseType->response, "\r\nRouter Software Version: ");
				strcat ((char *) parseType->response, VERSION);
				strcat ((char *) parseType->response, "\r\n");
				usart_putstr (parseType->response, parseType->Port);
				strcpy ((char *) parseType->response, NULL);

				strcpy ((char *) parseType->response, "You are now connected directly to the Master Model F board\r\n");
				usart_putstr (parseType->response, parseType->Port);
				strcpy ((char *) parseType->response, NULL);

				strcpy ((char *) parseType->response, "1=Master,2=Slave,0=Exit\r\n");
				usart_putstr (parseType->response, parseType->Port);
				strcpy ((char *) parseType->response, NULL);
			}  																										// end state entry code
			if (usart_rx_buffer_count (parseType->Port)) {															// if char pending, get it
				newChar = usart_getchar (parseType->Port);
				switch (newChar) {
				case '0':
					strcpy ((char *) parseType->response, "You are now connected back to the ETI Protocol Emulator\r\n");
					usart_putstr (parseType->response, parseType->Port);
					strcpy ((char *) parseType->response, NULL);
					usart_putstr  ("^RDAT22", upsOne.port);															// checksum on Master
					usart_putstr  ("^RDAT22", upsTwo.port);															// checksum on Slave
					parseType->BypassMode = FALSE;
					parseType->comState = ETI_IDLE;																	// exit bypass
					break;
				case '1':
					parseType->etiBypassUps = UPS1_PORT;															// select master
					usart_putstr  ("You are now connected directly to the Master Model F board", parseType->Port);
					usart_putchar (13, parseType->Port);
					usart_putchar (10, parseType->Port);
					break;
				case '2':
					parseType->etiBypassUps = UPS2_PORT;															// select slave
					usart_putstr  ("You are now connected directly to the Slave Model F board", parseType->Port);
					usart_putchar (13, parseType->Port);
					usart_putchar (10, parseType->Port);
					break;
				case 'h': case 'H':
					usart_putstr  ("1=Master,2=Slave,0=Exit", parseType->Port);
					usart_putchar (13, parseType->Port);
					usart_putchar (10, parseType->Port);
					usart_putchar (newChar, parseType->etiBypassUps);												// pass on char to selected UPS
					break;
				default:
					usart_putchar (newChar, parseType->etiBypassUps);												// pass on char to selected UPS, ups side in Main
					break;
				}
			}
			if (usart_rx_buffer_count (parseType->etiBypassUps)) {													// Master selected
				newChar = usart_getchar (parseType->etiBypassUps);
				usart_putchar (newChar, parseType->Port);
			}
			break;
		case ETI_INIT:
			if (parseType->comState != parseType->lastComState) {parseType->lastComState = parseType->comState;}	// Update State Machine
			updateData = getTime ();
			if ((etiStatusCheck (upsData, parseType)) & (parseType->Bootup)) {										// Bootup flag prevents infinite loop
				parseType->Bootup = FALSE; parseType->comState = ETI_AUTO; break;}									// Force Initial Status Message
			parseType->comState = ETI_IDLE;
			break;
		case ETI_IDLE:
			if (parseType->comState != parseType->lastComState) {parseType->lastComState = parseType->comState;}	// Update State Machine
			if (etiStatusCheck (upsData, parseType)) {parseType->comState = ETI_AUTO; break;}						// Is there an automatic update? Yes, process
			if (usart_rx_buffer_count (parseType->Port)) {parseType->comState = ETI_WAITING;}						// If character go for Data Collection State
			if (timer (updateData, 60000L)) {
				updateData = getTime ();
				#if !defined BATTERY_TRAY_INDICATION
					batteryTrayStatus = (upsData->batCond == FAULT);
				#endif
				#if defined ETI_DEBUG_PRINT
					sprintf ((char *) parseType->response, "BatVolt=%1.2f BatChgPct=%1.1f EstSecOnBat=%d SecOnBat=%d",
							 upsData->voltBat, upsData->batChgPct * 10.0, (int) upsData->estSecBat, (int) upsData->secOnBat);
					strcat ((char *) parseType->response, "\r\n");													// <CR><LF> terminate the string for passage
					usart_putstr (parseType->response, parseType->Port);											// Transmit
					strcpy ((char *) parseType->response, NULL);													// Make sure we are clear
				#endif
			}
			break;
		case ETI_WAITING:
			if (parseType->comState != parseType->lastComState) {parseType->lastComState = parseType->comState;}	// Update State Machine
			if (etiStatusCheck (upsData, parseType) == TRUE) {parseType->comState = ETI_AUTO; break;}				// Is there an automatic update? Yes, process
			if (usart_rx_buffer_count (parseType->Port)) { 															// Do we have a character
				newChar = usart_getchar (parseType->Port);															// Get the character for processing
				if (newChar == '\r') {parseType->comState = ETI_RESPONSE;}											// Is it an message end character, Yes we are done
				else {
					parseType->rcv [parseType->posRCV++] = toupper (newChar);										// No, part of message make it upper case
					parseType->rcv [parseType->posRCV] = NULL;														// Null pad for ease of parsing
					parseType->dataCount++;}}																		// Bump the data counter
			if (parseType->dataCount == MAX_ETI_COMMAND) {parseType->comState = ETI_RESPONSE;}						// Do we have max command length? Yes, process
			if (parseType->dataCount > MAX_ETI_COMMAND) {															// Are we still valid in data length
				strcpy ((char *) parseType->rcv, NULL);																// Bad data count, reset communication parameters should never reach but just in case
				parseType->dataCount = parseType->posRCV = 0;
				parseType->comState = ETI_IDLE;}
			break;
		case ETI_RESPONSE:
			if (parseType->comState != parseType->lastComState) {parseType->lastComState = parseType->comState;}	// Update State Machine
			parseType->cmd = etiSelectCommand ((char *) parseType->rcv);											// Translate the command
			switch (parseType->cmd) {																				// Process the command now
			case HELP: case QUESTIONMARK:																			// Help display
				parseType->comState = ETI_HELP;																		// Start the HELP Display (fix for smaller buffers)
				break;
			case STATUS:																							// Major Status Report
				etiStatusResponse (parseType);																		// Send the response
				parseType->comState = ETI_IDLE;																		// Go back to waiting, idle communications
				break;
			case SHUTDOWN:																							// Shutdown the UPS
				if (parseType->upsBatteryOps) {																		// Are we on Batteries?
					usart_putstr  ("System RSD\r\n", parseType->Port);
					masterCmdAddBoth ("^RMODIDM");																	// Set both UPS inverters to Off/Manual
				} else {
					usart_putstr  ("No SD unless On Battery\r\n", parseType->Port);
				}
				parseType->comState = ETI_IDLE;																		// Go back to waiting for the end, idle communications
				break;
			case SWVERSION:																							// Software Version ID
				strcpy ((char *) parseType->response, NULL);
				strcpy ((char *) parseType->response, "Model ");
				strcat ((char *) parseType->response, (char *) upsBoss.model);
				strcat ((char *) parseType->response, " Version ");
				strcat ((char *) parseType->response, (char *) upsBoss.verSoftware);
				strcat ((char *) parseType->response, "\r\nCopyright IntelliPower, Inc.\r\n");
				usart_putstr (parseType->response, parseType->Port);
				parseType->comState = ETI_IDLE;																		// Go back to waiting, idle communications
				break;
			case EXTENDED_ALARMS:
			case EXTENDED_BATTERY:																					// Non ETI command to get responses not supported by ETI
			case EXTENDED_INPUT:																					// These are internal to IntelliPower
			case EXTENDED_MODE_TOGGLE:
			case EXTENDED_OUTPUT:
			case EXTENDED_TEMP:
				extendedResponses (upsData, parseType);																// Process the information
				parseType->comState = ETI_IDLE;
				break;
			case EXTENDED_BYPASSMODE:
				strcpy ((char *) parseType->response, "\r\nEntering Bypass Mode...\r\n");
				usart_putstr (parseType->response, parseType->Port);
				parseType->BypassMode = TRUE;
				parseType->comState = ETI_BYPASS;
				break;
			case GENERAL_ALARM_RELAY_NORMAL:
				if (parseType->flagInputFail) {																		// If AC Input OK then action, else skip because it is real
					parseType->GeneralAlarmTest = FALSE;
					masterCmdAdd ("^RBUT21", &upsOne);
					strcpy ((char *) parseType->response, "System GAR Normal\r\n");
				} else {
					strcpy ((char *) parseType->response, "System GAR Alarm\r\n");
				}
				usart_putstr (parseType->response, parseType->Port);
				parseType->comState = ETI_IDLE;																		// Go back to waiting, idle communications
				break;
			case GENERAL_ALARM_RELAY_TEST:
				if (parseType->flagInputFail) {																		// If AC Input OK then action, else skip because it is real
					parseType->GeneralAlarmTest = TRUE;
					masterCmdAdd ("^RBUT20", &upsOne);
					parseType->timeGeneralTest = getTime ();
					strcpy ((char *) parseType->response, "System GAR Test\r\n");
				} else {
					strcpy ((char *) parseType->response, "System GAR Alarm\r\n");
				}
				usart_putstr (parseType->response, parseType->Port);
				parseType->comState = ETI_IDLE;																		// Go back to waiting, idle communications
				break;
			case BATTERY_ALARM_RELAY_NORMAL:
				if (!(parseType->flagLowBattery)) {																	// IF not Low Battery then action, else it is real
					parseType->BatteryAlarmTest = FALSE;
					masterCmdAdd ("^RBUT23", &upsOne);
					strcpy ((char *) parseType->response, "System BAR Normal\r\n");
				} else {
					strcpy ((char *) parseType->response, "System BAR Alarm\r\n");
				}
				usart_putstr (parseType->response, parseType->Port);
				parseType->comState = ETI_IDLE;																		// Go back to waiting, idle communications
				break;
			case BATTERY_ALARM_RELAY_TEST:
				if (!(parseType->flagLowBattery)) {																	// IF not Low Battery then action, else it is real
					parseType->BatteryAlarmTest = TRUE;
					masterCmdAdd ("^RBUT22", &upsOne);
					parseType->timeBatteryTest = getTime ();
					strcpy ((char *) parseType->response, "System BAR Test\r\n");
				} else {
					strcpy ((char *) parseType->response, "System BAR Alarm\r\n");
				}
				usart_putstr (parseType->response, parseType->Port);
				parseType->comState = ETI_IDLE;																		// Go back to waiting, idle communications
				break;
			case PERCENT_COMMAND:
				tempSOB1  = upsData->secOnBat;
				tempSOB2  = (tempSOB1 / 60);
				tempSOB1 -= (tempSOB2 * 60);
				sprintf ((char *) parseType->response, "%1.1f,%1.1f,%1.2f,%1.1f,%1.1f,%1.2f,%1.1f,%1.1f,%1.1f,%1.0f,%1.0f,%1.2f,%1.1f,%d:%d\r\n",
						upsData->voltOut, upsData->freqOut, upsData->ampOut, upsData->powOut, upsData->vaOut, upsData->pfOut, upsData->loadPctOut,
						upsData->voltIn, upsData->freqIn, upsData->tAmb, upsData->tSink, upsData->voltBat, upsData->ampBat, (int) (tempSOB2), (int) (tempSOB1));
				usart_putstr (parseType->response, parseType->Port);
				parseType->comState = ETI_IDLE;																		// Go back to waiting, idle communications
				break;
			case CLOSE_PARAN_COMMAND:
				sprintf ((char *) parseType->response, "%1.1f\r\n",
						 upsData->batChgPct * 10);
				usart_putstr (parseType->response, parseType->Port);
				parseType->comState = ETI_IDLE;																		// Go back to waiting, idle communications
				break;
			case NULL_ETI:
				parseType->comState = ETI_IDLE;
				break;
			}																										// End of SWITCH on ETI command
			strcpy ((char *) parseType->rcv, NULL);
			strcpy ((char *) parseType->response, NULL);
			parseType->dataCount = parseType->posRCV = 0;
			break;																									// End of the Response functions
		case ETI_HELP:
			if (parseType->comState != parseType->lastComState) {parseType->lastComState = parseType->comState;}	// Update State Machine
			usart_putstr ("\r\nIntelliPower, Inc.\r\n", parseType->Port);
			usart_putstr ("Voice: (714) 921-1580 Fax: (714) 921-4023\r\n", parseType->Port);
			parseType->comState = ETI_HELP1;																		// Get Ready to do the next part of HELP
			break;
		case ETI_HELP1:
			if (usart_tx_buffer_count(parseType->Port) != 0) {break;}												// Hold this state until TX Buffer empty
			if (parseType->comState != parseType->lastComState) {parseType->lastComState = parseType->comState;}	// Update State Machine
			usart_putstr ("INTERNET: http://www.intellipower.com\r\n", parseType->Port);
			usart_putstr ("ES1<CR>      - Request Major Status Report\r\n", parseType->Port);
			parseType->comState = ETI_HELP2;																		// Get Ready to do the next part of HELP
			break;
		case ETI_HELP2:
			if (parseType->comState != parseType->lastComState) {parseType->lastComState = parseType->comState;}	// Update State Machine
			if (usart_tx_buffer_count(parseType->Port) != 0) {break;}												// Hold this state until TX Buffer empty
			usart_putstr ("SD1<CR>      - Remote Shutdown (only when on battery)\r\n", parseType->Port);
			usart_putstr ("SW1<CR>      - Request Software Version ID\r\n", parseType->Port);
			parseType->comState = ETI_HELP3;																		// Get Ready to do the next part of HELP
			break;
		case ETI_HELP3:
			if (parseType->comState != parseType->lastComState) {parseType->lastComState = parseType->comState;}	// Update State Machine
			if (usart_tx_buffer_count(parseType->Port) != 0) {break;}												// Hold this state until TX Buffer empty
			usart_putstr ("GARN         - General Alarm Relay Normal\r\n", parseType->Port);
			usart_putstr ("GART         - General Alarm Relay ON\r\n", parseType->Port);
			parseType->comState = ETI_HELP4;																		// Get Ready to do the next part of HELP
			break;
		case ETI_HELP4:
			if (parseType->comState != parseType->lastComState) {parseType->lastComState = parseType->comState;}	// Update State Machine
			if (usart_tx_buffer_count(parseType->Port) != 0) {break;}												// Hold this state until TX Buffer empty
			usart_putstr ("BARN         - Battery Alarm Relay Normal\r\n", parseType->Port);
			usart_putstr ("BART         - Battery Alarm Relay ON\r\n", parseType->Port);
			parseType->comState = ETI_HELP5;																		// Get Ready to do the next part of HELP
			break;
		case ETI_HELP5:
			if (parseType->comState != parseType->lastComState) {parseType->lastComState = parseType->comState;}	// Update State Machine
			if (usart_tx_buffer_count(parseType->Port) != 0) {break;}												// Hold this state until TX Buffer empty
			usart_putstr ("HELP or ???? - Display This Menu\r\n", parseType->Port);
			strcpy ((char *) parseType->rcv, NULL);
			strcpy ((char *) parseType->response, NULL);
			parseType->dataCount = parseType->posRCV = 0;															// Reset communication parameters
			parseType->comState  = ETI_IDLE;																		// Get Ready to do the next part of HELP
			break;
		case ETI_AUTO:
			etiStatusResponse (parseType);																			// Do the ETI Status Update
			parseType->comState = parseType->lastComState;															// Go back to where we were
			break;
		default:
			parseType->comState = ETI_IDLE;
			break;
		}																											// End of State Machine for ETI communications Protocol
	}																												// End of ETIcom
#endif	// defined COM_ETI

#ifdef SNMP_MIMIC						// COMPILER FLAG for inclusion of the SNMP MIMIC functions
//*****************************************************************************************

//*****************************************************************************************
// 	Thales Charger Communications Routines
//	Two functions which can be switched in and out of the main compiled code using the Compiler
//	Flag SNMP_MIMIC. The code described here performs the protocol communications link
//  between the Charger and the UPS in the Thales system. Not sure how the upsThree data is
//	being populated on the Charger side. On the UPS side, the values for upsThree are updated
// from the communications. Not sure how the data will be processed.
//
// init_charger_com()
// charger_com()
//*****************************************************************************************
/*
    #define chargerPollTime 500		// Time to wait for the next command poll sequence

	void init_charger_com (volatile struct snmpDataStruct *parseType) {
		parseType->aLen[0] = parseType->aData[0] = NULL;	// NULL them out
		parseType->pos = parseType->subPos = 0;			// Zero the position pointers
		parseType->snmpComState = SNMP_BYPASS;			// Do this one first
		parseType->lastSnmpComState = SNMP_RESPONSE;	// Will fore State Entry
		parseType->comOkay = TRUE;						// set flag to show com is working
		// debug, uses charger.comOkay set upsThree.comOkay to track in upsData structure
			upsThree.comOkay = TRUE;
		// debug end
		chargerData.cCmd = cST1;						// First command in poll
		chargerData.timePollCharger = getTime ();		// Update our time
		chargerData.pollingTime = chargerPollTime;		// Default polling talk time
		chargerData.cDataResponse = FALSE;				// No command override for now
		upsThree.comErrors = 0;
		usart_rx_buffer_flush(charger.snmpPort);
	}
	
	chargerCommandEnumT cCmd = cST1;
	snmpParsePollEnumT cLastCmd = XD1;
	char ccData [SNMP_STR_MAX];
	
    // debug
    typedef enum {
        NO_ERROR,
        TEMP_BUFFER_FULL,
        WHILE_CHECKING_MSG_CARROT,
        NON_NUMBER_LENGTH_CHAR,
        DATA_SECTION_BUFFER_FULL,
        SNMP_IDLE_TIMEOUT,
        COMMAND_TYPE_BAD,
        SNMP_WAITING_TIMEOUT,
        DATA_LENGTH_BAD,
        ST1_MSG_TEST_FAIL,
        XD1_MSG_TEST_FAIL,
        XD2_MSG_TEST_FAIL,
        COM_ERROR_FAULT
    } debugErrorTypeT;
    #define DEBUG_ERROR_MAX (100)
    volatile debugErrorTypeT debugErrorType[DEBUG_ERROR_MAX];
    volatile long debugErrorCount = 0, debugErrorCountLast = 0;
    // debug end

	void charger_com (volatile struct upsDataStrucT *upsData, volatile struct snmpDataStruct *parseType) {
		static int itemp1;
		static volatile float ftemp1;
		static unsigned int comErrors = 0;
		static volatile struct timeT nowMsgTime;
		static int debug3 = 0;
		static volatile long maxMsgTime = 0, minMsgTime = 6000, avgMsgTime = 0;
		static volatile long maxCmdTime = 0, minCmdTime = 6000, avgCmdTime = 0;
		static long diffMsgTime = 0;
		static int batCondLast=0, batCondCount=0;
		volatile char *pString;
		static int firstTime = TRUE;                        // indicates first pass for function
		static int lastMsgOkay = TRUE;
		snmpStates_t tmpsState;
		char tmpChar;
		int tmpival1;
        long tmplval1;

	    upsData->StrTemp[0] = NULL;
		switch (parseType->snmpComState) 
		{
		case SNMP_COMMAND:
		    //if (parseType->snmpComState != parseType->lastSnmpComState) {	// State entry code
			tmpsState = parseType->snmpComState;
			if (tmpsState != parseType->lastSnmpComState)
			{
			    parseType->lastSnmpComState = parseType->snmpComState;		// Updating...
			    nowMsgTime = getTime();
			    //diffMsgTime = nowMsgTime.msec - chargerData.timePollCharger.msec;
			    if (firstTime == TRUE) {                                    // ignore first pass, function doesn't run for the first 43 seconds
			        firstTime = FALSE;
			    } else {
                    tmplval1 = nowMsgTime.msec;                               
                    diffMsgTime = tmplval1 - chargerData.timePollCharger.msec;			    
                    maxCmdTime = lMax(maxCmdTime, diffMsgTime);
                    minCmdTime = lMin(minCmdTime, diffMsgTime);
                    tmplval1 = avgCmdTime;
                    avgCmdTime += (long) ((diffMsgTime - tmplval1) * 0.1);
			    }
			    chargerData.timePollCharger = getTime ();						// Reset time for the polling
				switch (cCmd) 
				{
				case cST1:
					usart_putstr ("^P003ST1", parseType->snmpPort);			// Poll for the data
					#if (defined COM_DEBUG)
						comDebug ("Sending: ^P003ST1");
					#endif // defined COM_DEBUG
					cCmd = cXD1;											// Next command please
					cLastCmd = ST1;											// Set our override command
					break;
				case cXD1:
					usart_putstr ("^P003XD1", parseType->snmpPort);			// Poll for the data
					#if (defined COM_DEBUG)
						comDebug ("Sending: ^P003XD1");
					#endif // defined COM_DEBUG
					cCmd = cXD2;											// Next command
					cLastCmd = XD1;											// Set our override command
					break;
				case cXD2:
					usart_putstr ("^P003XD2", parseType->snmpPort);      // Poll for the data
					#if (defined COM_DEBUG)
						comDebug ("Sending: ^P003XD2");
					#endif // defined COM_DEBUG
					#if (!defined CHARGER_MONSTER)							// Normal operation
						cCmd = cST1;                                     // Next command
						cLastCmd = XD2;                                  // Set our override command
					#else													// Sends XD3 Set command
						cCmd = cSXD2;										// Next command
						cLastCmd = XD2;										// Set our override command
					#endif
					break;
				#if (defined CHARGER_MONSTER)									// Sends XD3 Set command
					case cSXD2:													// Set instead of Poll XD2
						sprintf((char *) upsData->StrTemp, "XD2%1.2f,%1.3f,%1.2f,%1.2f,%1.3f,%1.2f,%ld,%ld"
							,upsOne.voltBat,upsOne.ampChg,upsOne.ampBat,upsTwo.voltBat,upsTwo.ampChg,upsTwo.ampBat
							,upsOne.batJoule,upsTwo.batJoule);
						parseType->dataLen = strlen((char *) upsData->StrTemp);
						strcpy((char *) responseStr,"^S");
						strcat((char *) responseStr,itoa3(parseType->dataLen));
						strcat((char *) responseStr,(char *) upsData->StrTemp);
						usart_putstr(responseStr,parseType->snmpPort);
						#if (defined COM_DEBUG)
							comDebug ("Sending: ^P003ST1");
						#endif // defined COM_DEBUG
						cCmd = cCMD_END;										// Next command
						cLastCmd = XD2;											// Set our override command
						break;
				#endif
				case cCMD_END:
				default:
					cCmd = cST1;												// Restart the command polling
					//cLastCmd = NO_SNMP_CMD;									// Set our override command
					break;
				} // End of commands
				parseType->snmpComState = SNMP_IDLE;							// Now watch for communications
			}	// End state entry code
			break; // End of SNMP_COMMAND
		case SNMP_IDLE:	// Waiting for timed command, state change cmd or ups generated notification (button press, etc.)
		    //if (parseType->snmpComState != parseType->lastSnmpComState) {	// state entry code
			tmpsState = parseType->snmpComState;
			if (tmpsState != parseType->lastSnmpComState)
			{
				parseType->lastSnmpComState = parseType->snmpComState;
				//parseType->timeOutStart = getTime();
				parseType->type = UNKNOWN;
			}  // end state entry code

			if (usart_rx_buffer_count(parseType->snmpPort)) 			// beginning of message
			{
				if (usart_peekchar(parseType->snmpPort) == '^') 		// if the beginning character
				{
					parseType->snmpComState = SNMP_WAITING;			// so go to wait for the rest
				} 
				else 												// starting character
				{
					tmpChar = usart_getchar(parseType->snmpPort);	// pull off buffer and trash
					// limit wait time in this state
				}
			}
			//if (timer (chargerData.timePollCharger, (chargerData.pollingTime+2000))) {
			tmplval1 = chargerData.pollingTime+2000;
			if (timer (chargerData.timePollCharger, tmplval1)) 
			{
			    init_charger_com (pCharger);
			    parseType->type = FAIL;
                debugErrorType[debugErrorCount] = SNMP_IDLE_TIMEOUT;
                debugErrorCount++;
                if (debugErrorCount >= DEBUG_ERROR_MAX) {
                    debugErrorCount = 0;
                }
			}
			break;
		case SNMP_WAITING:	// waiting for incoming msg to complete
		    //if (parseType->snmpComState != parseType->lastSnmpComState) {	// state entry code
			tmpsState = parseType->snmpComState;
			if (tmpsState != parseType->lastSnmpComState) {
				parseType->lastSnmpComState = parseType->snmpComState;

				upsData->StrTemp[0] = parseType->aLen[0] = parseType->aData[0] = NULL;
				parseType->pos = parseType->subPos = 0;
				parseType->phase = START;
				parseType->type = UNKNOWN;
			}  // end state entry code
			if (usart_rx_buffer_count(parseType->snmpPort)) 				// if char pending, get it
			{
				if (parseType->pos >= (TEMP_BUFFER_LENGTH-2)) 				// upsData->StrTemp buffer almost full
				{
					parseType->type = FAIL;									// dump message, start again
					debugErrorType[debugErrorCount] = TEMP_BUFFER_FULL;
					debugErrorCount++;
                    if (debugErrorCount >= DEBUG_ERROR_MAX) {
                        debugErrorCount = 0;
                    }
				} 
				else 
				{	// upsData->StrTemp buffer not full
					// get char, put in string, bump pointer
					//upsData->StrTemp[parseType->pos++] = parseType->snmpChar = usart_getchar(parseType->snmpPort);  // Mark's new code
						tmpChar = usart_getchar(parseType->snmpPort);
						parseType->snmpChar = tmpChar;
						upsData->StrTemp[parseType->pos++] = tmpChar;
						// debug
					if ( (usart_peekchar(parseType->snmpPort) == '^') && (parseType->pos > 1) ) 
					{
					    tmpChar = usart_getchar(parseType->snmpPort);    // eat '^'
					    parseType->type = FAIL;
					    debugErrorType[debugErrorCount] = WHILE_CHECKING_MSG_CARROT;
					    debugErrorCount++;
					    if (debugErrorCount >= DEBUG_ERROR_MAX) {
                           debugErrorCount = 0;
                       }
					}
					
					// debug end
					switch (parseType->phase) 
					{
					case START:
						// nothing to process
						break;
					case LEN:
						// make sure it's an ascii code within chars '0' and '9' and buffer not full
						if ((parseType->snmpChar >= '0') && (parseType->snmpChar <= '9') && (parseType->subPos < SNMP_STR2_MAX)) 
						{
						    //parseType->aLen[parseType->subPos++] = parseType->snmpChar;
							tmpChar = parseType->snmpChar;
							parseType->aLen[parseType->subPos++] = tmpChar;
							parseType->aLen[parseType->subPos] = NULL;						// terminate with null so string processes correctly atoi
						} 
						else 
						{
							parseType->type = FAIL;
                           debugErrorType[debugErrorCount] = NON_NUMBER_LENGTH_CHAR;
                           debugErrorCount++;
                            if (debugErrorCount >= DEBUG_ERROR_MAX) {
                                debugErrorCount = 0;
                            }
						}
						break;
					case DATA:
						// make sure buffer not full
						if (parseType->subPos < (SNMP_STR_MAX-2)) 							// at string max - 2, add char and null
						{
						    //parseType->aData[parseType->subPos++] = parseType->snmpChar;	// accumulate data string
							tmpChar = parseType->snmpChar;
							parseType->aData[parseType->subPos++] = tmpChar;
							parseType->aData[parseType->subPos] = NULL;						// easy to process, no matter when we stop
							//if (parseType->subPos >= parseType->dataLen) {					// when position+1 = length, stop
							tmpival1 = parseType->subPos;
							if (tmpival1 >= parseType->dataLen)
							{
								upsData->StrTemp[0] = NULL;									// reset, used for part of response msg
								parseType->snmpComState = SNMP_RESPONSE;					// message is done, now to respond to it
							}
						} 
						else 
						{
							parseType->type = FAIL;
                           debugErrorType[debugErrorCount] = DATA_SECTION_BUFFER_FULL;
                           debugErrorCount++;
                            if (debugErrorCount >= DEBUG_ERROR_MAX) {
                                debugErrorCount = 0;
                            }
						}
						break;
					default:
						break;
					}
                    switch (parseType->pos)  // since this is bumped after getting char, 2 means 2nd char in snmpChar
                    {
                    case 2:	// check after we have two characters
                        switch (parseType->snmpChar) 
                        {
                        case 'P':
                            parseType->type = POLL;
                            break;
                        case 'S':
                            parseType->type = SET;
                            break;
                        case 'D':
                            parseType->type = SET;						// Data coming back, we need to SET our UPS Data Structure
                            chargerData.cDataResponse = TRUE;			// Set flag to monkey with the command decoder in Response
                            break;
                        case 'X':
                        parseType->snmpComState = SNMP_COMMAND;		// "^X" same as UPS to exit SNMP
                            break;
                        default:
                           debugErrorType[debugErrorCount] = COMMAND_TYPE_BAD;
                           debugErrorCount++;
                            if (debugErrorCount >= DEBUG_ERROR_MAX) {
                                debugErrorCount = 0;
                            }
                            parseType->type = FAIL;
                            break;
                        }
                        if (parseType->snmpComState == SNMP_IDLE) {break;}
                        parseType->phase = LEN;	// next get number of char in message
                        parseType->subPos = 0;		// reset to fill sub field
                        break;
                    case 5:	// now aLen should have string with length number in it
                        parseType->dataLen = atoi((char *) parseType->aLen);
                        if ((parseType->dataLen > 0) && (parseType->dataLen < SNMP_STR_MAX)) 
                        {
                            parseType->phase = DATA;	// next get number of char in message
                            parseType->subPos = 0;		// reset to fill sub field
                        } 
                        else 
                        {
                            parseType->type = FAIL;
                            debugErrorType[debugErrorCount] = DATA_LENGTH_BAD;
                            debugErrorCount++;
                            if (debugErrorCount >= DEBUG_ERROR_MAX) {
                                debugErrorCount = 0;
                            }
                       }
                        break;
                    default:
                        break;
                    }
				}  // end of else clause of if (parseType->pos >= (TEMP_BUFFER_LENGTH-2)) 
			} // end of then clause of if (usart_rx_buffer_count(parseType->snmpPort))
			// message failed to conform to protocol in some way
			// message didn't complete within the time limit
			//if (timer (chargerData.timePollCharger, chargerData.pollingTime+3000)) {
            tmplval1 = chargerData.pollingTime+3000;
            if (timer (chargerData.timePollCharger, tmplval1))                        
			{
              init_charger_com (pCharger);
              parseType->type = FAIL;
              debugErrorType[debugErrorCount] = SNMP_WAITING_TIMEOUT;
              debugErrorCount++;
              if (debugErrorCount >= DEBUG_ERROR_MAX) {
                  debugErrorCount = 0;
              }
			}
			break;
		case SNMP_RESPONSE:	// Not used for timed updating, this is when question is first asked by Upsilon
		    //if (parseType->snmpComState != parseType->lastSnmpComState) {	// state entry code
			tmpsState = parseType->snmpComState;
			if (tmpsState != parseType->lastSnmpComState) 
			{
				parseType->lastSnmpComState = parseType->snmpComState;
			}  // end state entry code
			if (chargerData.cDataResponse) {
				switch (cLastCmd) {
				case ST1:
					strcpy ((char *) ccData, NULL);
					strcpy ((char *) ccData, "ST1");						// Insert command from data response
					break;
				case XD1:
					strcpy ((char *) ccData, NULL);
					strcpy ((char *) ccData, "XD1");						// Insert command from data response
					break;
				case XD2:
					strcpy ((char *) ccData, NULL);
					strcpy ((char *) ccData, "XD2");						// Insert command from data response
					break;
				default:
					strcpy ((char *) ccData, NULL);
					strcpy ((char *) ccData, "XXX");						// Insert illegal command from data response
					break;
				}
				strcat ((char *) ccData, (char *) parseType->aData);
				parseType->aData [0] = NULL;
				strcpy ((char *) parseType->aData, (char *) ccData);
				chargerData.cDataResponse = FALSE;
			}
			parseType->snmpCmd = selectSnmpCmd(parseType->aData);      // get enum value related to string command
			scanSnmpParams (parseType);                               // There is data so scan it now
            pString = &StrParams [0][0];                              // Battery Condition (batCond)
            itemp1 = atoi ((char*) pString);
            if ( (itemp1 >= 0) && (itemp1 <= 3) ) {                   // check for valid response
                parseType->snmpCmd = ST1;                             // select parse for command
            } else {
                pString = &StrParams [3][0];                          // Power Factor
                ftemp1 = atof ((char*) pString);
                if ( (ftemp1 <= 1.0) && (ftemp1 >= 0) ) {             // check for valid response
                    parseType->snmpCmd = XD1;                             // select parse for command
                } else {
                    pString = &StrParams [0][0];                // voltBat
                    ftemp1 = atof ((char*) pString);
                    if ( (ftemp1 > 72.0) && (ftemp1 < 150.0) ) {                        // check for valid response
                        parseType->snmpCmd = XD2;                             // select parse for command
                    }
                }
            }
			switch(parseType->snmpCmd) {
			case ST1:
				pString = &StrParams [0][0];
				itemp1 = atoi ((char*) pString);
				if (itemp1 > 3) {                                      // check for valid response
				    parseType->type = FAIL;                            // response failed
					debugErrorType[debugErrorCount] = ST1_MSG_TEST_FAIL;
					debugErrorCount++;
                    if (debugErrorCount >= DEBUG_ERROR_MAX) {
                        debugErrorCount = 0;
                    }
				    break;
				}
				//tmpival1 = itemp1;
				if (itemp1 == batCondLast) {
				    if (batCondCount++ <= 10) {
				        itemp1 = 0;                                    // set to normal until new state is constant
				    } else {
				        batCondCount = iMin(batCondCount,11);
				    }
				} else {                                               // different batCon
				    batCondLast = itemp1;
				    batCondCount = 0;                                  // reset count
				    itemp1 = 0;                                        // set to normal this time
				}
				switch (itemp1) {
				case 0:
					upsData->batCond = NORMAL;
					break;
				case 1:
					upsData->batCond = WARN;
					break;
				case 2:
					upsData->batCond = FAULT;
					break;
				case 3:
					upsData->batCond = OVER_VOLTAGE;
					break;
				default:
					upsData->batCond = NORMAL;	// Tricky double on the other side
					break;
				}
				pString = &StrParams [1][0];
				upsData->batSts = atoi ((char*) pString);
				pString = &StrParams [2][0];
				upsData->chgModeSnmp = atoi ((char*) pString);
				// chgModeSnmp 0=float, 1=Charging, 2=Resting, 3=Discharging
				// chgMode 1=fast, 2=slow, 3=off
				switch(upsData->chgModeSnmp) {							// convert from number to mode
				case 1:
					upsData->chgMode = AUTO_FAST;
					break;
				case 0:
					upsData->chgMode = AUTO_SLOW;
					break;
				case 2:
					upsData->chgMode = AUTO_OFF;
					break;
				}
				pString = &StrParams [3][0];
				upsData->secOnBat = atol ((char*) pString);
				pString = &StrParams [4][0];
				upsData->estSecBat = atol ((char*) pString);
				pString = &StrParams [5][0];
				if (parseType->parser == SNMP) {
					upsData->batChgPct = atof ((char*) pString);
				} else {
					upsData->batChgPct = (atof ((char*) pString) / 10.0 - 0.5);
				}
				upsData->batChgPct = atof ((char*) pString);
				pString = &StrParams [6][0];
				upsData->voltBat = ((atof ((char*) pString) / 10) - 0.5);
				pString = &StrParams [7][0];
				upsData->ampBat = (atof ((char*) pString) /10 - 0.5);
				pString = &StrParams [8][0];
				upsData->tAmb = atof ((char*) pString);
				parseType->snmpComState = SNMP_IDLE;
				parseType->type = OKAY;
				break;
			case XD1:
				pString = &StrParams [3][0];
				ftemp1 = atof ((char*) pString);
				if (ftemp1 <= 1.0) {                        // check for valid response
				    upsData->pfOut = ftemp1;
				} else {
				    parseType->type = FAIL;                 // response failed
					debugErrorType[debugErrorCount] = XD1_MSG_TEST_FAIL;
					debugErrorCount++;
                    if (debugErrorCount >= DEBUG_ERROR_MAX) {
                        debugErrorCount = 0;
                    }
				    break;                                  // wrong response, leave
				}
				pString = &StrParams [0][0];
				upsData->ampChg = (atof ((char*) pString) / 10 - 0.5);
				pString = &StrParams [1][0];
				upsData->voltBus = (atof ((char*) pString) / 10 - 0.5);
				pString = &StrParams [2][0];
				upsData->tSink = atof ((char*) pString);
				pString = &StrParams [3][0];
				upsData->pfOut = atof ((char*) pString);
				upsData->pfOut = ftemp1;
				pString = &StrParams [4][0];
				upsData->voltSupply = (atof ((char*) pString) / 10 - 0.5);
				parseType->snmpComState = SNMP_IDLE;
				parseType->type = OKAY;
				break;
			case XD2:
				pString = &StrParams [0][0];                // voltBat
				ftemp1 = atof ((char*) pString);
				if (ftemp1 < 72.0) {                        // check for valid response
				    parseType->type = FAIL;                 // response failed
					debugErrorType[debugErrorCount] = XD2_MSG_TEST_FAIL;
					debugErrorCount++;
                    if (debugErrorCount >= DEBUG_ERROR_MAX) {
                        debugErrorCount = 0;
                    }
				    break;                                  // wrong response, leave
				}
				//pString = &StrParams [0][0];
				//upsOne.voltBat = atof((char*) pString);
				upsOne.voltBat = ftemp1;
				pString = &StrParams [1][0];
				upsOne.ampChg = atof((char*) pString);
				//pString = &StrParams [2][0];
				//upsOne.ampBat = atof((char*) pString);
				//pString = &StrParams [3][0];
				//upsTwo.voltBat = atof((char*) pString);
				pString = &StrParams [4][0];
				upsTwo.ampChg = atof((char*) pString);
				//pString = &StrParams [5][0];
				//upsTwo.ampBat = atof((char*) pString);
				parseType->snmpComState = SNMP_IDLE;
				parseType->type = OKAY;
				break;
			case NO_SNMP_CMD:
			default:
				//usart_putstr("^0",parseType->snmpPort);					// cmd not understood or not implemented
				parseType->snmpComState = SNMP_IDLE;
				break;
			}
			break;
		default:
			parseType->snmpComState = SNMP_IDLE;
			parseType->lastSnmpComState = SNMP_RESPONSE;
			break;
		}	// end switch
		//if ( (timer (chargerData.timePollCharger, (chargerData.pollingTime + 1000))) || (parseType->type == FAIL) ) // Old code
		//if ( (timer (chargerData.timePollCharger, (chargerData.pollingTime + 4500))) || (parseType->type == FAIL) ) // New Code
		tmpival1 = (chargerData.pollingTime + 1000);
		if (timer (chargerData.timePollCharger, tmpival1) || parseType->type == FAIL ) 
		{
		    lastMsgOkay = FALSE;
			debug3++;
			parseType->type = UNKNOWN;
			upsThree.comErrors = comErrors;
			parseType->comErrors = comErrors;
			if (comErrors++ > 50) 
			{
				comErrors = iMin (comErrors, 60);					// Limit value so it doesn't roll over
				// othewise if comm fails on timeout this will erase the last type=OKAY
				parseType->comOkay = FALSE;
				// debug, uses charger.comOkay set upsThree.comOkay to track in upsData structure
				charger.comOkay = FALSE;
				upsThree.comOkay = FALSE;
				// debug end
				// bad idea to init_charger_com resets parseType->comOkay = TRUE
			    //init_charger_com (pCharger);                    // initialize communication pointers
                debugErrorType[debugErrorCount] = COM_ERROR_FAULT;
                debugErrorCount++;
                if (debugErrorCount >= DEBUG_ERROR_MAX) {
                    debugErrorCount = 0;
                }
            }
			parseType->snmpComState = SNMP_COMMAND;              // Done, start again
			parseType->lastSnmpComState = SNMP_IDLE;				// force it to run state entry code
		}
		if (parseType->type == OKAY) 
		{
		    lastMsgOkay = TRUE;
			// reset type so it will only branch here on next good message
			parseType->comOkay = TRUE;
			// debug, uses charger.comOkay set upsThree.comOkay to track in upsData structure
			charger.comOkay = TRUE;
			upsThree.comOkay = TRUE;
			// debug end
			comErrors = 0;
			upsThree.comErrors = comErrors;
			parseType->comErrors = comErrors;
			nowMsgTime = getTime();
			//diffMsgTime = nowMsgTime.msec - chargerData.timePollCharger.msec;
			tmplval1 = nowMsgTime.msec;
			diffMsgTime = tmplval1 - chargerData.timePollCharger.msec;
			maxMsgTime = lMax(maxMsgTime, diffMsgTime);
			minMsgTime = lMin(minMsgTime, diffMsgTime);
			tmplval1 = avgMsgTime;
			avgMsgTime += (long) ((diffMsgTime - tmplval1) * 0.1);
			//if (timer (chargerData.timePollCharger, chargerData.pollingTime)) 
		}
		if (lastMsgOkay == TRUE) {
			tmplval1 = chargerData.pollingTime;
            if (timer (chargerData.timePollCharger, tmplval1))
			{
			    lastMsgOkay = FALSE;                            // reset, wait for next okay
			    parseType->type = UNKNOWN;                      // so it will not come back until new command is sent
			    parseType->snmpComState = SNMP_COMMAND;         // Done, start again
			    parseType->lastSnmpComState = SNMP_IDLE;        // force it to run state entry code
			}
		}
        		
		if (comErrors > 5) 
		{
			__NOP;
		}
		tmplval1 = debugErrorCountLast;
		if (debugErrorCount != tmplval1) {
		    debugErrorCountLast = debugErrorCount;
		}
	} // End of charger_com routine
*/
	void init_charger_com (volatile struct snmpDataStruct *parseType) 
	{
		parseType->snmpComState = SNMP_IDLE;			// Do this one first
		parseType->lastSnmpComState = SNMP_RESPONSE;	// Will fore State Entry
		parseType->comOkay = TRUE;						// set flag to show com is working
		// debug, uses charger.comOkay set upsThree.comOkay to track in upsData structure
			upsThree.comOkay = TRUE;
		// debug end
		//chargerData.cCmd = cST1;						// First command in poll
		chargerData.timePollCharger = getTime ();		// Update our time
		chargerData.pollingTime = 2000;		            // Default polling talk time
		chargerData.cDataResponse = FALSE;				// No command override for now
		upsThree.comErrors = 0;
		usart_rx_buffer_flush(charger.snmpPort);
	}
	
	void charger_com (volatile struct upsDataStrucT *upsData, volatile struct snmpDataStruct *parseType) {
	    static int comError = 0, comErrorLast = 0;
	    char charTemp;
	    int iTemp1, iTemp2;
	    long lTemp1, lTemp2;
	    float fTemp1;
	    char *pString;
		snmpStates_t tmpsState;
	    
		switch (parseType->snmpComState) 
		{
		case SNMP_IDLE:
		    //if (parseType->snmpComState != parseType->lastSnmpComState) {	// state entry code
			tmpsState = parseType->snmpComState;
			if (tmpsState != parseType->lastSnmpComState)
			{
				parseType->lastSnmpComState = parseType->snmpComState;
				//parseType->timeOutStart = getTime();
				//parseType->type = UNKNOWN;
			}  // end state entry code
			if (usart_rx_buffer_count(parseType->snmpPort)) 	// char in buffer between router sourced cmds
			{
			    parseType->snmpComState = SNMP_WAITING;
			}
		    break;
		case SNMP_WAITING:	// waiting for incoming msg to complete
		    //if (parseType->snmpComState != parseType->lastSnmpComState) {	// state entry code
			tmpsState = parseType->snmpComState;
			if (tmpsState != parseType->lastSnmpComState) 
			{
				parseType->lastSnmpComState = parseType->snmpComState;

				upsData->StrTemp[0] = NULL;
				parseType->pos = 0;
			}  // end state entry code
			if (usart_rx_buffer_count(parseType->snmpPort))        	// if char pending, get it
			{
			    charTemp = usart_getchar(parseType->snmpPort);
			        
			    if (charTemp == 10)                                	// new line, end of message
			    {
			        parseType->snmpComState = SNMP_CHECK;
			    }
			    else if (parseType->pos <= (TEMP_BUFFER_LENGTH-5))
			    {
			        if (charTemp != 13)                            	// ignore carriage return, throw away
			        {
                        upsData->msgStr[parseType->pos++] = charTemp;
                        upsData->msgStr[parseType->pos] = NULL;   	// terminate for string functions
			        }
			    } 
			    else 
			    {    // if (parseType->pos > (TEMP_BUFFER_LENGTH-5))
			        parseType->snmpComState = SNMP_IDLE;          	// Failed, start again
			    }
			}
			/*
			// Note this could cause race conditon with timeout if there is noise
			// interpreted as characters when charger is off preventing or slowing alarm
			lTemp1 = chargerData.pollingTime;
			if (timer(chargerData.timePollCharger, lTemp1)) 
			{
			    comError++;
			    chargerData.timePollCharger = getTime ();			// Update our time
			}
			*/
		    break;
		case SNMP_CHECK:
			scanParams(upsData);	                                // get list of parameters
			if (upsData->lastParam != -1)                          	// -1 means no params found
			{
               iTemp2 = (strlen((char *)upsData->msgStr)-6);  		// checksum of characters not including checksum
               lTemp1 = 0;
               for (iTemp1=0;iTemp1<iTemp2;iTemp1++) 
               {
                   lTemp1 += (int) upsData->msgStr[iTemp1];
               }
			} 
			else 
			{
			    comError++;                                        	// failed
			    parseType->snmpComState = SNMP_IDLE;               	// go back and wait for next message
			}
            pString = &StrParams[upsData->lastParam][0];          	// charger checksum in last place
            lTemp2 = atol((char *) pString);
            if (lTemp1 == lTemp2)                                 	// checksums match
            {
                comError = 0;                                     	// reset errors
                parseType->comOkay = TRUE;
                charger.comOkay = TRUE;                           	// report any prior comOkay is cleared
				upsThree.comOkay = TRUE;
                chargerData.timePollCharger = getTime ();		 	// Update our time
                parseType->snmpComState = SNMP_RESPONSE;          	// get information from message
            } 
            else 
            {
			    comError++;                                        	// failed
			    parseType->snmpComState = SNMP_IDLE;               	// go back and wait for next message
            }
		    break;
		case SNMP_RESPONSE:	// Not used for timed updating, this is when question is first asked by Upsilon
            tmpsState = parseType->snmpComState;
            if (tmpsState != parseType->lastSnmpComState) 
            {
                parseType->lastSnmpComState = parseType->snmpComState;
            }  // end state entry code

            // item 0, Battery Condition
            pString = &StrParams [0][0];                            // Battery Condition (batCond)
            iTemp1 = atoi ((char*) pString);
            switch (iTemp1) 
            {
            case 0:
                upsData->batCond = NORMAL;
                break;
            case 1:
                upsData->batCond = WARN;
                break;
            case 2:
                upsData->batCond = FAULT;
                break;
            case 3:
                upsData->batCond = OVER_VOLTAGE;
                break;
            default:
                upsData->batCond = NORMAL;	// Tricky double on the other side
                break;
            }

            // item 1, Battery Status
			// Battery Low, 1=Low, 0=Normal
            pString = &StrParams [1][0];
            upsData->batSts = atoi ((char*) pString);
			
            // item 2, Charge Mode
            // chgModeSnmp 0=float, 1=Charging, 2=Resting, 3=Discharging
            pString = &StrParams [2][0];
            upsData->chgModeSnmp = atoi ((char*) pString);
            // chgModeSnmp 0=float, 1=Charging, 2=Resting, 3=Discharging
            // chgMode 1=fast, 2=slow, 3=off
            switch(upsData->chgModeSnmp)							// convert from number to mode
            {
            case 1:
                upsData->chgMode = AUTO_FAST;
                break;
            case 0:
                upsData->chgMode = AUTO_SLOW;
                break;
            case 2:
                upsData->chgMode = AUTO_OFF;
                break;
            }
            upsBoss.chgMode = upsData->chgMode;
            

            // item 3, Seconds on Battery
            pString = &StrParams [3][0];
            upsData->secOnBat = atol ((char*) pString);

			// item 4, Estimated Battery Runtime
			pString = &StrParams [4][0];
			upsData->estSecBat = atol ((char*) pString);

			// item 5, Charger Amps, UPS One
			pString = &StrParams [5][0];
			upsOne.ampChg = atof((char*) pString);

			// item 6, Charger Amps, UPS Two
			pString = &StrParams [6][0];
			upsTwo.ampChg = atof((char*) pString);

			// add together for Boss
			fTemp1 = upsTwo.ampChg;
			upsBoss.ampChg = upsOne.ampChg + fTemp1;

			// item 7, Ambient Temperature
			pString = &StrParams [7][0];
			upsData->tAmb = atof ((char*) pString);

			// item 8, Heatsink Temperature
			pString = &StrParams [8][0];
			upsData->tSink = atof ((char*) pString);

			parseType->snmpComState = SNMP_IDLE;
		default:
		    parseType->snmpComState = SNMP_IDLE;
		    break;
		}
		/*
		iTemp1 = (chargerData.pollingTime + 1000);
		if (timer (chargerData.timePollCharger, iTemp1)) 
		{
		    chargerData.timePollCharger = getTime();
		    comErrorLast = comError;
			upsThree.comErrors = comError;
			parseType->comErrors = comError;
			if (comError++ > 50) 
			{
				comError = iMin (comError, 60);					// Limit value so it doesn't roll over
				parseType->comOkay = FALSE;
				// debug, uses charger.comOkay set upsThree.comOkay to track in upsData structure
				charger.comOkay = FALSE;
				upsThree.comOkay = FALSE;
            }
		}
		*/
		// Timeout on loss of communication without depending on comError
		//iTemp1 = (chargerData.pollingTime + 1000);
		if (timer (chargerData.timePollCharger, 30000)) 
		{
            parseType->comOkay = FALSE;
            // debug, uses charger.comOkay set upsThree.comOkay to track in upsData structure
            charger.comOkay = FALSE;
            upsThree.comOkay = FALSE;
		}
		// debug
		if (comError != comErrorLast)
		{
		    comErrorLast = comError;
		}
		// debug end
	}
    
#endif  // ifdef SNMP_MIMIC

#if (defined THALES_CHARGER)
	void init_charger_info_com (volatile struct snmpDataStruct *parseType) {
	    
		chargerData.timePollCharger = getTime ();		// Update our time
		chargerData.pollingTime = 500;		            // Default polling talk time
	    parseType->snmpComState = SNMP_WAITING;
	}
	void charger_info_com (volatile struct upsDataStrucT *upsData, volatile struct snmpDataStruct *parseType) {
		static int firstTime = TRUE;                        // indicates first pass for function
		int iTemp1, iTemp2;
		long lTemp1;
		float fTemp1;
		snmpStates_t tmpsState;
		//operatingModesT opMode1, opMode2;                    // used for batCond and chgMode
		char stemp[10];


	    if (firstTime == TRUE) {
	    }
		switch (parseType->snmpComState) 
		{
		case SNMP_WAITING:	// waiting for incoming msg to complete
		    //if (parseType->snmpComState != parseType->lastSnmpComState) {	// state entry code
			tmpsState = parseType->snmpComState;
			if (tmpsState != parseType->lastSnmpComState) {
				parseType->lastSnmpComState = parseType->snmpComState;

				//parseType->phase = START;
				//parseType->type = UNKNOWN;
			}  // end state entry code
			lTemp1 = chargerData.pollingTime;
			if (timer (chargerData.timePollCharger, lTemp1)) {
			    chargerData.timePollCharger = getTime();
			    parseType->snmpComState = SNMP_RESPONSE;
			}
		    break;
		case SNMP_RESPONSE:                                        // send information
		    tmpsState = parseType->snmpComState;
            if (tmpsState != parseType->lastSnmpComState) 
            {
                parseType->lastSnmpComState = parseType->snmpComState;

                upsData->StrTemp[0] = parseType->aLen[0] = parseType->aData[0] = NULL;
                parseType->pos = parseType->subPos = 0;
            }  // end state entry code
            
            // item 0, Battery Condition
            switch (upsOne.batCond)
            {
            case NORMAL:
                iTemp1 = 0;
                break;
            case WARN:
                iTemp1 = 1;
                break;
            case FAULT:
                iTemp1 = 2;
                break;
            case OVER_VOLTAGE:
                iTemp1 = 3;
                break;
            default:
                iTemp1 = 0;
                break;
            }
            switch (upsTwo.batCond)
            {
            case NORMAL:
                iTemp2 = 0;
                break;
            case WARN:
                iTemp2 = 1;
                break;
            case FAULT:
                iTemp2 = 2;
                break;
            case OVER_VOLTAGE:
                iTemp2 = 3;
                break;
            default:
                iTemp2 = 0;
                break;
            }
            if ((iTemp1 == 2) || (iTemp2 == 2)) {   // 2=Fault
                iTemp1 = 2;                         // set to Fault
            } else {
                iTemp1 = iMax(iTemp1, iTemp2);      // 0=Normal, >0 fault conditions
            }
			strcpy((char *) upsData->StrTemp,itoa(iTemp1));
			strcat((char *) upsData->StrTemp,",");
			
			// item 1, Battery Status
			// Battery Low, 1=Low, 0=Normal
			if ((upsOne.batSts == 1) || (upsTwo.batSts == 1)) {
			    iTemp1 = 1;                                         // set to low
			} else {
			    iTemp1 = 0;
			}
			strcat((char *) upsData->StrTemp,itoa(iTemp1));	
			strcat((char *) upsData->StrTemp,",");
			
           // item 2, Charge Mode
           // chgModeSnmp 0=float, 1=Charging, 2=Resting, 3=Discharging
           iTemp1 = upsTwo.chgModeSnmp;
           iTemp1 = iMax(upsOne.chgModeSnmp, iTemp1);
           strcat((char *) upsData->StrTemp,itoa(iTemp1));	
           strcat((char *) upsData->StrTemp,",");
			
           // item 3, Seconds on Battery
           iTemp1 = upsTwo.secOnBat;
           iTemp1 = iMin(upsOne.secOnBat, iTemp1);
           strcat((char *) upsData->StrTemp,itoa(iTemp1));	
           strcat((char *) upsData->StrTemp,",");

           // item 4, Estimated Battery Runtime
           lTemp1 = upsTwo.estSecBat;
           lTemp1 = lMin(upsOne.estSecBat, lTemp1);
           sprintf( (char *) stemp, "%ld", lTemp1);
           strcat((char *) upsData->StrTemp,(char *) stemp);	
           strcat((char *) upsData->StrTemp,",");

           // item 5, Charger Amps, UPS One
           sprintf( (char *) stemp, "%4.3f", upsOne.ampChg);
           strcat((char *) upsData->StrTemp,(char *)stemp);
           strcat((char *) upsData->StrTemp,",");

           // item 6, Charger Amps, UPS Two
           sprintf( (char *) stemp, "%4.3f", upsTwo.ampChg);
           strcat((char *) upsData->StrTemp,(char *)stemp);
           strcat((char *) upsData->StrTemp,",");

           // item 7, Ambient Temperature
           fTemp1 = upsTwo.tAmb;
           fTemp1 = fMax(upsOne.tAmb, fTemp1);
           sprintf( (char *) stemp, "%4.3f", fTemp1);
           strcat((char *) upsData->StrTemp,(char *)stemp);
           strcat((char *) upsData->StrTemp,",");

           // item 8, Heatsink Temperature
           fTemp1 = upsTwo.tSink;
           fTemp1 = fMax(upsOne.tSink, fTemp1);
           sprintf( (char *) stemp, "%4.3f", fTemp1);
           strcat((char *) upsData->StrTemp,(char *)stemp);
           strcat((char *) upsData->StrTemp,",");
			
           // add checksum
           iTemp2 = strlen((char *)upsData->StrTemp);
           lTemp1 = 0;
           for (iTemp1=0;iTemp1<iTemp2;iTemp1++) {
               lTemp1 += (int) upsData->StrTemp[iTemp1];
           }
           strcat((char *) upsData->StrTemp,itoa6(lTemp1));
           strcat((char *) upsData->StrTemp,"\r\n");

           usart_putstr(upsData->StrTemp,parseType->snmpPort);

           parseType->snmpComState = SNMP_WAITING;
		default:
		    parseType->snmpComState = SNMP_WAITING;
		    break;
		}
	}
#endif  // defined THALES_CHARGER

#if ((defined UCLASS_FA10002) && (!defined NAVFAC_FA10241))	// NAVFAC doesn't use remote opto control
	void uclass_com(void) {									// some added communication between boards
		volatile ups_states_t lastUpsState = UPS_NULL;
		static volatile struct timeT timerModeset, timerRdat07;
		static volatile int optoEpoLast = 1, relayOverTempLast = 1, relayTempWarningLast = 1;
		
		switch(upsBoss.upsState) {
		case UPS_INIT:
			if (upsBoss.upsState != lastUpsState) {
				lastUpsState = upsBoss.upsState;
				upsBoss.relayOnBat = 0;						// not on battery
				timerModeset = getTime();
				timerRdat07 = getTime();
			}
			break;
		case UPS_ON_UTIL:
			if (upsBoss.upsState != lastUpsState) {
				lastUpsState = upsBoss.upsState;
				upsBoss.relayOnBat = 0;						// not on battery
			}
			if (upsBoss.optoEpo == 1) {						// EPO has priority
				upsBoss.upsState = UPS_OFF;
				if (upsBoss.optoEpo != optoEpoLast) {
					optoEpoLast = upsBoss.optoEpo;
					addEvent("uclass_com() - EPO Active",9);
				}
			} else if (upsBoss.optoOnOff != upsBoss.optoOnOffLast) {	// Remote state change
				upsBoss.optoOnOffLast =  upsBoss.optoOnOff;
				if (upsBoss.optoOnOff == 0) {
					upsBoss.upsState = UPS_OFF;
					addEvent("uclass_com() - opto Remote On/Off => Off",9);
				}
			}
			break;
		case UPS_ON_BAT:
			if (upsBoss.upsState != lastUpsState) {
				lastUpsState = upsBoss.upsState;
				upsBoss.relayOnBat = 1;						// on battery
			}
			if (upsBoss.optoEpo == 1) {						// EPO has priority
				upsBoss.upsState = UPS_SHUTDOWN;
			} else if (upsBoss.optoOnOff != upsBoss.optoOnOffLast) {	// Remote state change
				upsBoss.optoOnOffLast =  upsBoss.optoOnOff;
				if (upsBoss.optoOnOff == 0) {
					upsBoss.upsState = UPS_SHUTDOWN;
					addEvent("uclass_com() - opto Remote On/Off => Off",9);
				}
			}
			break;
		case UPS_OFF:
			if (upsBoss.upsState != lastUpsState) {
				lastUpsState = upsBoss.upsState;
				upsBoss.relayOnBat = 0;						// not on battery
			}
			if (upsBoss.optoEpo == 0) {						// EPO off
				if (upsBoss.optoOnOff != upsBoss.optoOnOffLast) {		// Remote state change
					upsBoss.optoOnOffLast =  upsBoss.optoOnOff;
					if (upsBoss.optoOnOff == 1) {
						upsBoss.upsState = UPS_ON_UTIL;
						addEvent("uclass_com() - opto Remote On/Off => On",9);
					}
				}
			}
			break;
		}
		
		if (timer(timerModeset, 4000)) {
			timerModeset = getTime();
			if ( (upsBoss.tSink >= TEMP_HS_ALM_ON) || (upsBoss.tAmb >= TEMP_AMB_ALM_ON) ) {
				upsBoss.relayTempWarning = 1;
				if (upsBoss.relayTempWarning != relayTempWarningLast) {
					relayTempWarningLast = upsBoss.relayTempWarning;
					addEvent("uclass_com() - relayTempWarning => On",9);
				}
			} else {
				if ( (upsBoss.tSink < TEMP_HS_ALM_OFF) && (upsBoss.tAmb < TEMP_AMB_ALM_OFF) ) {
					upsBoss.relayTempWarning = 0;
					if (upsBoss.relayTempWarning != relayTempWarningLast) {
						relayTempWarningLast = upsBoss.relayTempWarning;
						addEvent("uclass_com() - relayTempWarning => Off",9);
					}
				}
			}
			/*
			if ( (upsBoss.tSink >= TEMP_HS_TRIP_ON) || (upsBoss.tAmb >= TEMP_AMB_TRIP_ON) ) {
				upsBoss.relayOverTemp = 1;
				if (upsBoss.relayOverTemp != relayOverTempLast) {
					relayOverTempLast = upsBoss.relayOverTemp;
					addEvent("uclass_com() - relayOverTemp => On",9);
				}
			} else {
				if ( (upsBoss.tSink < TEMP_HS_ALM_OFF) && (upsBoss.tAmb < TEMP_AMB_TRIP_OFF) ) {
					upsBoss.relayOverTemp = 0;
					if (upsBoss.relayOverTemp != relayOverTempLast) {
						relayOverTempLast = upsBoss.relayOverTemp;
						addEvent("uclass_com() - relayOverTemp => Off",9);
					}
				}
			}
			*/
			// Put out Modeset string with no commas
			strcpy((char *) upsBoss.StrTemp,"^RMODR");
			strcat((char *) upsBoss.StrTemp,itoa(upsBoss.relayOnBat));
			strcat((char *) upsBoss.StrTemp,itoa(upsBoss.relayTempWarning));		// highest of master/slave
			strcat((char *) upsBoss.StrTemp,"22");									// not used, yet
			strcat((char *) upsBoss.StrTemp,itoa(upsBoss.battleShort));
			strcat((char *) upsBoss.StrTemp,itoa(upsBoss.relayTempWarning));		// this should be from slave
			masterCmdAdd(upsBoss.StrTemp, &upsOne);
		}
		if (timer(timerRdat07, 4000)) {
			timerRdat07 = getTime();
			masterCmdAdd("^RDAT07", &upsTwo);			// request optoisolator input status
			// debug
			#if defined RDAT07_DEBUG
			rdat07debug();
			#endif
			// debug end
		}
	}
#endif
	
//*****************************************************************************************

void main(void){

//      WDTCTL = WDTPW + WDTHOLD;       // Another IAR item; in their default main.c
//   Watchdog init now done in __low_level_init.c, runs before main and before
//   data is initialized.  This is the IAR method.  This code is loaded unly if the
//   __IAR compile option is defined <IAR_YES_NO.h>
	
        static volatile struct timeT timeDisplay, timeUpsStateRunning, timeVersionChecked;
	static volatile struct timeT timeComLed, timeLoop;
	// **debug
	static volatile struct timeT timeTemp;
	// **debug end
	#if BAT_CAP_METHOD==BAT_CAP_JOULE
		static volatile struct timeT timeBatCapJoule;
	#endif
	volatile char tempChar;
	static long loopCount = 0;

	char tmpIARchar;    

	// Initialize subsystems
	low_level_init();

	// Initialize PWM timer
	timera_init(); // LCD backlight PWM
	timerb_init(); // MSec timer

	__enable_interrupt();

	// If clock test passed, leave green LED on for one second
	// Start with all LEDs off

	ledTest();	// test local router board LEDs

	LED0_GRN; // Turn on board OK led

	// Initialize UARTS
	/*
	usart0_init();
	usart1_init();
	usart2_init();
	usart3_init();
	*/
	usart_init();

	// Initialize UPS Data structure
	initRs485com();
	init_ups_com();
	init_snmp_com(pSnmp);
	init_snmp_com(pUpsilon);
	#ifdef SNMP_MIMIC
		init_charger_com (pCharger);
		charger.snmpPort = UPSNET_PORT;				// Use the RS-485 Channel
	#endif
    #if (defined THALES_CHARGER)
		init_charger_info_com (pCharger);
		charger.snmpPort = UPSNET_PORT;				// Use the RS-485 Channel
    #endif

	// Use the ETI Protocol Translator, note it runs on the SNMP_PORT
	#ifdef COM_ETI
		pEti = &eti;
		initETIcom (pEti);
	#endif

	init_ups_state_controller();
	initUpdateDisplay();

	// set up SNMP and Upsilon
	pSnmp = &snmp;				// address of structure into pointer var
	pUpsilon = &upsilon;		// address of structure into pointer var

	// Addition of the second SNMP communications for debugging proposes on Thales
	#ifdef SNMP2						// FLAG for inclusion of second debug SNMP functions
		init_snmp_com (pSnmp2);
		pSnmp2 = &snmp2;				// Address of structure into pointer variable
		snmp2.snmpPort = SNMP2_PORT;	// Use the normal SNMP_PORT for communications
		snmp2.parser   = SNMP;			// Use the SNMP parser for this function
	#endif

	snmp.snmpPort = SNMP_PORT;
	snmp.parser   = SNMP;
	upsilon.snmpPort = SNMP_PORT;
	upsilon.parser   = UPSILON;
	#ifdef SNMP_MIMIC
		pCharger = &charger;		// Address of structure into pointer variable
		charger.snmpPort = UPSNET_PORT;	// Use the normal SNMP_PORT for communications
		charger.parser = SNMP;		// Use the SNMP parser for this function
	#endif
    #if (defined THALES_CHARGER)
		pCharger = &charger;		// Address of structure into pointer variable
		charger.snmpPort = UPSNET_PORT;	// Use the normal SNMP_PORT for communications
		charger.parser = SNMP;		// Use the SNMP parser for this function
    #endif

	initEvent();

	timeStarted = upsBoss.timeStarted = getTime();
	timeVersionChecked = getTime();
	timeDisplay = getTime();
	timeComLed = getTime();
	timeLoop = getTime();
	#if BAT_CAP_METHOD==BAT_CAP_JOULE
		timeBatCapJoule = getTime();
	#endif
	// ** debug
	timeTemp = getTime();
	// ** debug end
	
	pUpsOne = &upsOne; 				// address of structure into pointer var
	pUpsTwo = &upsTwo;
	pUpsBoss = &upsBoss;

	pUpsThree = &upsThree;			// Address of the structure into a pointer variable

	addEvent("Starting System",9);

	// TODO - Debug Setup
	// Reconfigure P4.7 for debug use
	//P4SEL &= ~BIT7;					// turn off Aux function
	//P4DIR |= BIT7;					// set as output
	//P4OUT &= ~BIT7;					// set low initially
	// Initialize LCD
	
	#if LCD_DISPLAY == TRUE
		// Initialize buttons
		buttons_init();
		
		// Setup LCD
		//BACKLIGHT_MED;						// TODO: setup backlight control Timer0 CCR1 duty cycle
		while (!timer(timeStarted, 100));		// wait for a while before initializing LCD
		LCD_init();
		LCD_putstr((ubyte *)UPS_MODEL, 0, 0);
		LCD_putstr((ubyte *)"Initializing", 1, 0);
	#endif
	
	// get UID from Flash, if outside of range in SNMP poll it will be set to 0
	// it can only be fixed if SNMP does a UID SET to number in range
	flashPointer = (unsigned int *) 0x1880;			// Start of Information Flash Segment C *** Moved to avoid conflict with Battery Joules ***
	flashPointer += 2;								// Keep the offset from the older system but look in the new segment
	upsBoss.snmpUid = *flashPointer;

	updateAlm(SONALERT_OFF);						// Put Display in normal state
	updateAlm(UNBLANK_ALL);							// Put Display in normal state
	
	#if defined TEST_COMM_RS485_TO_RS232			// calls function with Forever loop to sniff RS485 and put it on SNMP RS232
		testCommRs485toRs232 ();
	#endif

	Forever {	//main loop

	    #ifndef RS485LOCKOUT						// RS485 LOCKOUT FLAG because charger_com() and snmp_com() uses RS485 on Thales
			rs485com();
		#endif

		#if LCD_DISPLAY==TRUE
			lcdWriteStateController();
			if (timer(timeDisplay, 1000)) {			// no need to update too often
				timeDisplay = getTime();
				#if ((!defined LCD_MANAGER_BOSS) && (DUAL_BOARD==FALSE))
					lcdManager(pUpsOne);
				#else
					lcdManager(pUpsBoss);
				#endif
				updateAlm(UPDATE);						// updates alarm string
				refreshAlarmString ();					// Updates sonalert command to UPS, normally called by updateDisplay (for LED Display)
			}
		#else
			//updateComStatusLeds();					// doing it differently so they flash
			if (timer(timeDisplay, 1000)) {			// no need to update too often
				timeDisplay = getTime();
				updateDisplay();
				updateAlm(UPDATE);
			}
		#endif
		if (timer(timeStarted, 1000)) {
			#ifdef COM_SNMP
				#ifdef SNMP2
					if (snmp2.snmpComState == SNMP_BYPASS) {	// talking directly to ups, RS232 to Router in snmp_com()
				#else
					if (snmp.snmpComState == SNMP_BYPASS) {		// talking directly to ups, RS232 to Router in snmp_com()
				#endif
			#endif
			#ifdef COM_UPSILON
			if (upsilon.snmpComState == SNMP_BYPASS) {		// talking directly to ups, RS232 to Router in snmp_com()
			#endif
			#ifdef COM_ETI
			if (FALSE) {
			#endif
				if (usart_rx_buffer_count(snmpBypassUps)) {	// check selected ups receive
					tempChar = usart_getchar(snmpBypassUps);
					#ifdef SNMP2					// uses RS232 SNMP port for local test
						#ifdef RS485LOCKOUT			// RS485 LOCKOUT FLAG because charger_com()
													//  and snmp_com() uses RS485 on Thales
							usart_putchar(tempChar, snmp2.snmpPort);
						#endif
					#else
					    //usart_putchar(tempChar, snmp.snmpPort);
						tmpIARchar = tempChar;
						usart_putchar(tmpIARchar, snmp.snmpPort);
					#endif
				}
			} else {								// normal operation
				#if UPS_STATE_CONTROL==TRUE		// use router to control UPS operation
					ups_state_controller();
				#else // UPS_STATE_CONTROL==FALSE,	use router to control UPS operation
					rotatingCmdHold = FALSE;		// remove hold on rotating commands, normally done in UPS state control
				#endif
				#if defined COM_ETI
					if (eti.BypassMode) {
						// Hold off on UPS COM action
					} else {
						ups_com(pUpsOne);
						#if DUAL_BOARD
							ups_com(pUpsTwo);
						#endif
					}
				#else
					if (snmp.snmpComState != SNMP_BYPASS) {	// don't interrogate ups if in SNMP bypass
						ups_com(pUpsOne);
						#if DUAL_BOARD || defined THALES_3KVA
							ups_com(pUpsTwo);
						#endif
					}
				#endif
			}
		}
        #if (defined SNMP_IMMEDIATE_START)
            if (timer(timeStarted, 10)) {		    // start immediately with nominal values to prevent SNMP confusion (out of sync)
		#elif UPS_STATE_CONTROL==TRUE				    // use router to control UPS operation
			if (upsBoss.upsState == UPS_INIT) {
				timeUpsStateRunning = getTime();	// keep updating unit Boss state machine leaves init state
				timeDisplay = getTime();			// keep display from updating as well
			}
			//if (timer(timeUpsStateRunning, 20000)) {// wait until we have information from master and slave UPS
			if (timer(timeStarted, 10)) {// wait until we have information from master and slave UPS
        #else
            if (timer(timeStarted, 10000)) {		// wait until we have information from master and slave UPS
		#endif
			#ifdef COM_SNMP
				snmp_com(pUpsBoss,pSnmp);
			#endif
			#ifdef COM_UPSILON
				snmp_com(pUpsBoss,pUpsilon);
			#endif
			#ifdef COM_ETI
				#if DUAL_BOARD
					ETIcom (pUpsBoss, pEti);
				#else
					ETIcom (pUpsOne, pEti);
				#endif
			#endif
			#if (defined SNMP_MIMIC)
				charger_com (pUpsThree, pCharger);
			#endif
            #if (defined THALES_CHARGER)
				charger_info_com (pUpsThree, pCharger);
            #endif
			#ifdef SNMP2
				snmp_com (pUpsBoss, pSnmp2);	// This is the debug version of SNMP running on Thales Charger
			#endif
			}
		if (timer(timeVersionChecked, 15000)) {		// time to check
			timeVersionChecked = getTime();		// reset counter
			if (strcmp((char *)upsOne.verSoftware,"Unknown") == 0) { 	// need to get
				masterCmdAdd(&upsCmd[RDAT31_UPS_PROG_VER][0], &upsOne); // Request Version
			} else {									// must need Slave version
				if (strcmp((char *)upsTwo.verSoftware,"Unknown") == 0) {
					masterCmdAdd(&upsCmd[RDAT31_UPS_PROG_VER][0], &upsTwo); // Request Version
				}
			}
		}
		#if BAT_CAP_METHOD==BAT_CAP_JOULE
			if (timer(timeBatCapJoule, 1000)) {			// no need to update too often
				timeBatCapJoule = getTime();
				batCapJouleUpdate(pUpsOne);
				// if called twice doubles discharge/recharge joules
				#if ((!defined THALES_3KVA) || (DUAL_BOARD==TRUE))
					batCapJouleUpdate(pUpsTwo);
				#endif
			}
		#endif
		#if LCD_DISPLAY==FALSE
			if (timer(timeComLed, 500)) {		// turned on in UART interrupt red=xmit green=rcv
				timeComLed = getTime();		// reset counter
				RED0_OFF;						// usart3 - UPS2, User or Upsilon
				GRN0_OFF;
				RED1_OFF;						// usart2 - RS485 (UPS Net)
				GRN1_OFF;
				RED2_OFF;						// usart1 - SNMP
				GRN2_OFF;
				RED3_OFF;						// usart0 - UPS1
				GRN3_OFF;
			}
		#endif
		if (!timer(timeLoop, 10000)) {	
			//timeLoop = getTime();
			loopCount++;
		} 
		else 
		{
			__NOP;
		}
		/*
				if (usart2_rx_buffer_count()) {				// TODO - RS485 debug - loopback
					debugChar = usart2_getchar();
					usart_putchar(debugChar,UPSNET_PORT);
				}
		*/
		#if ((defined UCLASS_FA10002) && (!defined NAVFAC_FA10241))
			uclass_com();									// some added communication between boards
		#endif
		#if defined COMM_MONITOR_PORT				// communication monitoring enabled
			if (strlen((char *) upsOne.StrCommMonitor) != 0) {						// Master
				usart_putstr("To Master:   ", COMM_MONITOR_PORT); // send string to indicated port
				usart_putstr(upsOne.StrCommMonitor, COMM_MONITOR_PORT); // send string to indicated port
				usart_putstr("\r\n", COMM_MONITOR_PORT); // send string to indicated port
				upsOne.StrCommMonitor[0] = NULL;			// set string to zero characters
			}
			if (strlen((char *) upsTwo.StrCommMonitor) != 0) {						// Slave
				usart_putstr("To Slave:    ", COMM_MONITOR_PORT); // send string to indicated port
				usart_putstr(upsTwo.StrCommMonitor, COMM_MONITOR_PORT); // send string to indicated port
				usart_putstr("\r\n", COMM_MONITOR_PORT); // send string to indicated port
				upsTwo.StrCommMonitor[0] = NULL;			// set string to zero characters
			}
		#endif										// defined COMM_MONITOR_PORT
	}	// end main infinite while loop
}//main
