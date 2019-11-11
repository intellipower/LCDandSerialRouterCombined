#ifndef _main_h_    // inclusion guard
#define _main_h_

#include "types.h"
#include "timerb.h"

typedef enum _upsParsePollEnum_t {
    RDAT01_PARAMS,			// Request Parameters
    RDAT02_PARAMS2,			// Request more Parameters
    RDAT03_STATUS,			// Request Status
    RDAT04_NOTIFY_ON,		// Turn on Notify
    RDAT05_NOTIFY_OFF,		// Turn off Notify
    RDAT06_MISC_PARMS,		// Left over parameters
    RDAT07_OPTO_INPUTS,	    // Optoisolator states 4 fields, EPO, BatShort, On/Off, TBD - 0=off, 1=on, 2=not used
    RDAT10_COMBO_PARMS,	    // All of the others requested together
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
	RDAT32_BAT_LOW_RLY_ON,	// Sends command to turn on low battery signal relay
	RDAT33_BAT_LOW_RLY_OFF,// Sends command to turn off low battery signal relay
    //	The next commands just show the prefix, the rest varies so only test the prefix
    RMOD,					// Mode commands, returns "^1" for success, "^0" for failure
    RLOD,					// Display load bar set command
    RBAT,					// Display battery capacity bar set command
    RDIS,					// Display LEDs On, Bypass, Service Bat, Fault set command
    RALM,					// Sonalert and Aux 1, Aux 2 LED control
	RSET,					// Bank switch set command
    // The next set are unsolicited messages from the UPS, this is just the prefix ex. ^RBUT30
    RBUT,					// Indicates front panel button press or notification of event i.e. Lightshow done

    CMD_END					// special marker indicating no active command
} upsParsePollEnum_t;

#define SNMP_STR_MAX 100
#define SNMP_STR2_MAX 50

typedef enum {
    SNMP,
    UPSILON
} snmpParserT;


#define SNMP_CMD_MAX 25

typedef enum /* _snmpParsePollEnumT */ {
    AP1,		// Poll commands, some Set commands
    AP2,
    ATR,		// Poll and Set
    BTS,
    MAN,
    MOD,
    NOM,
    OTC,
    OTD,
    PSD,		// Timed shutdown
    ST1,
    ST2,
    ST3,
    ST4,
    ST5,
    ST6,
    ST7,
    SDA,		// Poll and Set
    STD,		// Timed startup
    STR,
    UBR,
    UID,
    VER,
    XD1,
    XD2,
    NO_SNMP_CMD
} snmpParsePollEnumT ;

//snmpParsePollEnumT snmpParsePoll;


typedef enum {
    START, LEN, DATA
} snmpPartT;

typedef enum {
    POLL,
    SET,
    FAIL,
    OKAY,
    UNKNOWN
} snmpTypeT;

struct snmpDataStruct {
	volatile int snmpPort;
	volatile snmpParserT parser;
	volatile snmpStates_t snmpComState, lastSnmpComState;
	volatile char aLen[SNMP_STR2_MAX], aData[SNMP_STR_MAX],/* responseStr[SNMP_STR_MAX], */ snmpChar;
	volatile int pos, subPos, dataLen, comOkay;
	// comOkay is normally used when SNMP command is received, if the command is not understood
	// it returns "^0".  When used with the SNMP mimic for getting information it will count the
	// number of times the charger does not respond.
	#ifdef SNMP_MIMIC
		volatile int comErrors;
	#endif
	volatile snmpTypeT type;
	volatile snmpPartT phase;
	volatile snmpParsePollEnumT snmpCmd;
	volatile struct timeT timeOutStart;						// timeOut for waiting for response
	volatile char StrParams[PARAM_NUM_MAX][PARAM_LEN_MAX];	//	when parsed each parameter will be in a string form up to 20 params
	volatile int lastParam;									//	last parameter location, -1 if none, 0 if 1
} ;

// Charger for Thales Items including our commands and a data structure
typedef enum _chargerCommandEnumT {
	cST1,					// Request Battery Volts, Charge Percent, and Temperatures
	cXD1,					// Request Battery Charging Current
	cXD2,					// Request Battery higher resolution charging current, adding bat volts and amps
	cSXD2,					// Set battery information for Monster Charger, just passing joules now
	cCMD_END				// End of the line marker a non-event
} chargerCommandEnumT;

struct chargerDataStruct {
	volatile struct timeT timePollCharger;
	volatile chargerCommandEnumT cCmd;
	volatile snmpParsePollEnumT cLastCmd;
	volatile int cDataResponse;
	volatile int pollingTime;						// This value may be modified runtime to alter polling speed
};

#define CMD_STR_MAX 15

// This structure is the central data hub of the program, all communication is done through string
// variables in this struction, all transactions are tracked, all parameters, errors, etc.
struct upsDataStrucT {
	volatile int upsNumber;
	volatile portName_t port;									// enumerated port defined in uscia_UART.h
	volatile ups_states_t upsState, lastUpsState;
	volatile comStates_t upsComState, lastUpsComState;
	volatile comWaitStates_t ComWaitState, lastComWaitState;
	volatile int SubState, SubStateLast;						// sequential states within states, breaking states down into phases
	volatile upsParsePollEnum_t currentCmd;					// either timedCmd or masterCmd
	volatile char msgStr [TEMP_BUFFER_LENGTH];
	volatile char msgChr;										// last character from buffer
	volatile int nextChrPos;									// position in string next character will take, empty = 0
	volatile char StrTemp [TEMP_BUFFER_LENGTH];				// use this to edit a string, ex. pulling comma delimited response apart
	volatile int nextTempChrPos;								// position in string next character will take, empty = 0
	//volatile char StrParams[PARAM_NUM_MAX][PARAM_LEN_MAX];	//	when parsed each parameter will be in a string form up to 20 params
	volatile int lastParam;									//	last parameter location, -1 if none, 0 if 1
	volatile char masterCmdStr[MAX_MASTER_CMDS][CMD_STR_MAX];//	This holds the commands sent to the UPS's in a circular buffer
	volatile upsParsePollEnum_t masterCmd[MAX_MASTER_CMDS];	// when masterCmdStr[x] gets sent, masterCmd[x] is used to process response
	volatile int masterCmdBot,masterCmdTop,masterCmdTot;		//	pointers and total of master commands in buffer
	volatile long checksumMsg, checksumCalc;					// checksum of parsed message, if active
	volatile checksum_t checksumMode;							// Indicates when we are receiving a checksum
	volatile int timedCmd;										// number of cmd pending from rotating list
	volatile int startupInvCmd;								// Flag indicating an inverter startup command has been issued
	volatile int notifyMsg;									// in process of receiving notification from ups, button press, status change, etc.
	volatile int snmpUid;										// SNMP ID number 0-999 so more than one UPS on network can be identified
	volatile char almMask[DISPLAY_LEN];						// alarm string sent to UPS, 9 char and null
	volatile int comOkay, comMsgOkay, comErrors;
	volatile struct timeT timeCmdMade, timeOutStart, timeStarted;			// timeCmdMade used for next time, timeOut for waiting for response
	volatile float ampBat, ampChg, voltBat, batChgPct, hiVoltTrans, freqIn, voltIn, lowVoltTrans;
	volatile float freqInNom, voltInNom, timeLoBatNom, freqOutNom, powOutNom, vaOutNom, voltOutNom;
	volatile long batJouleNom, batJoule;
	volatile float batJouleFraction, vBatWarn, vBatShutdown;
	volatile float freqOut, loadPctOut, powOut, voltOut, vaOut, pfOut, ampOut, tAmb, tSink;
	#ifdef CRYSTAL_FIX
		volatile float pfOutFiltered;
	#endif
	volatile float voltBus, voltSupply, ampSupply;
	volatile operatingModesT invMode, dcMode, chgMode, bypassMode, syncMode, fanMode, batCond;
	volatile operatingModesT tAmbMode, tSinkMode, powOutMode, ampOutMode;	// TODO get tAmbMode warn/fault, only have alarm
	volatile operatingModesT inBadAlm, outBadAlm, invFaultAlm, outOffAlm, invOverloadTrip;
	volatile operatingModesT overloadAlmtempAlm, audAlmMode, autoStartMode;
	volatile long durSecReboot, secOnBat, estSecBat, secStartupDelay, secShutdownDelay;// secShutdownDelay 1-999 sec, 0=off
	volatile char estTimeStr[10];										// formatted bat time remaining min:sec
	// chgModeSnmp takes snmp value from ups RDAT06 and passes it to SNMP, RDAT03 send mode ex. AUTO_FAST
	//volatile int batSts, batStsLast, chgModeSnmp, linesNumIn, batWarnFlag;
	int batSts, batStsLast, chgModeSnmp, linesNumIn, batWarnFlag;
	volatile int battleShort;											// 1=battleshort, 0=off
	volatile int linesNumOut, sourceOutMode, shutdownType, testType;
	volatile char upsId[IDENT_LEN_MAX], man[IDENT_LEN_MAX], model[IDENT_LEN_MAX], verSoftware[IDENT_LEN_MAX];
	volatile long baudrate, bank1, bank2, bank3, ledStatus[4];
	//volatile int startupPending;
	#if defined UCLASS_FA10002
		volatile int optoEpo, optoBattleShort, optoOnOff, optoTbd;
		volatile int optoOnOffLast;
		volatile int relayOnBat, relayOverTemp, relayTempWarning;
	#endif
	#if defined COMM_MONITOR_PORT				// communication monitoring enabled
		volatile char StrCommMonitor [TEMP_BUFFER_LENGTH];		// communication debuging
	#endif
    #if (defined NOV_CAT_J)
        // ORed summary relay status from Micro board
        int bossSummaryRelayStatus;
    #endif
};

extern volatile struct upsDataStrucT upsOne, upsTwo, upsThree, upsBoss, *pUpsOne, *pUpsTwo, *pUpsThree, *pUpsBoss, *pUps;

// The folllowing array has been moved to main -- IAR would not link due to duplicate instantiations w/ lcd.display.c if the the .h file was loaded
//                                                                                                             ...it included main.h which led to trouble
//char StrParams[PARAM_NUM_MAX][PARAM_LEN_MAX];	

// Function Prototypes
/*
int iMax(volatile int var1, volatile int var2);
int iMin(volatile int var1, volatile int var2);
int iRange(volatile int test, volatile int limit1, volatile int limit2);
long lMax(volatile long lvar1, volatile long lvar2);
long lMin(volatile long lvar1, volatile long lvar2);
int lRange(volatile long test, volatile long limit1, volatile long limit2);
float fMax(volatile float fvar1, volatile float fvar2);
float fMin(volatile float fvar1, volatile float fvar2);
int fRange(volatile float test, volatile float limit1, volatile float limit2);
*/
void all_tx_rx_LED_off(void);
void updateComStatusLeds(void);
void ups_rx_LED_on(void);
void ups_tx_LED_on(void);
void ledTest(void);
void initEvent(void);
void addEvent(volatile char *str, volatile int reportLevel);
/*
ubyte ups_rx_buffer_count(volatile int ups);
void ups_rx_buffer_flush(volatile int ups);
ubyte ups_putchar(volatile char cByte, volatile int ups);
ubyte ups_putString(volatile char *str, volatile int ups);
ubyte ups_getchar(volatile int ups);
ubyte ups_peekchar(volatile int ups);
ubyte snmp_rx_buffer_count(void);
ubyte snmp_getchar(void);
ubyte snmp_peekchar(void);
ubyte snmp_putchar(volatile char cByte);
ubyte snmp_putString(volatile char *str);
*/
int masterCmdPending(volatile struct upsDataStrucT *upsData);
int masterCmdAdd(volatile char *str, volatile struct upsDataStrucT *upsData);
int masterCmdAddBoth(volatile char *str);
int masterCmdLabelAdd(volatile upsParsePollEnum_t cmdLabel, volatile struct upsDataStrucT *upsData);
int masterCmdLabelAddBoth(volatile upsParsePollEnum_t cmdLabel);
int masterCmdSend(volatile struct upsDataStrucT *upsData);
void updateAlm(volatile alm_states_t state);
void updateLoadDisplay(void);
void updateBatDisplay(void);
void updateLedDisplay(void);
void initUpdateDisplay(void);
void updateDisplayRefresh(void);
void updateDisplay(void);
operatingModesT selectStrOpMode(volatile char *str);
upsParsePollEnum_t scanResponseHeader(volatile char *str);
long scanEstSecBat(volatile char *str);
int scanParams(volatile struct upsDataStrucT *upsData);
void scanSnmpParams(volatile struct snmpDataStruct *parseType);
void tab(volatile char *str, volatile int tabTo);
void rs485tabularData(void);
operatingModesT inverterControl(void);
void initRs485com(void);
void rs485com(void);
void init_ups_com(void);
void ups_com(volatile struct upsDataStrucT *upsData);
snmpParsePollEnumT selectSnmpCmd(volatile char *str);
void init_snmp_com(volatile struct snmpDataStruct *parseType);
void snmp_com(volatile struct upsDataStrucT *upsData, volatile struct snmpDataStruct *parseType);
void init_ups_state_controller(void);
void ups_state_controller(void);	// TODO - straighten out function prototypes
char *itoa(volatile int i);
char *itoa3(volatile int i);
void refreshAlarmString (void);
void init_charger_com (volatile struct snmpDataStruct *parseType);
void charger_com (volatile struct upsDataStrucT *upsData, volatile struct snmpDataStruct *parseType);
#if BAT_CAP_METHOD==BAT_CAP_JOULE
	void batCapJouleUpdate(volatile struct upsDataStrucT *upsData);
#endif
#if ((defined UCLASS_FA10002) && (!defined NAVFAC_FA10241))
	void uclass_com(void);						// some added communication between boards
#endif
void main(void);

#endif						// end inclusion guard
