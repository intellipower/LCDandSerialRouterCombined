#ifndef __TYPES_H // inclusion guard
#define __TYPES_H

#include "timera.h"
#include "uscia_UART.h"

#define uint8_t unsigned char
#define uint16_t unsigned int
#define uint32_t unsigned long
//#define uint64_t unsigned long long

#define int8_t char
#define int16_t int
#define int32_t long

#define ubyte 	unsigned char		// 8 bits
#define bool 	ubyte
#define bool_t  bool
#define ushort 	unsigned int		// 16 bits
#define sshort 	signed int			// 16 bits
#define uint 	unsigned int		// 32 bits
#define sint 	signed int			// 32 bits
#define ulong 	unsigned long		// 32 bits

// common values
#define TRUE 0x01
#define FALSE 0x00

// readability
#define Forever for(;;)			// Neat

typedef enum comStates_t {
    COM_IDLE,
    COM_WAITING,
    COM_RESPONSE
} comStates_t;

typedef enum comWaitStates_t {
    COM_WAIT_START,
    COM_WAIT_MESSAGE,
    COM_WAIT_CHECKSUM,
    COM_WAIT_DONE
} comWaitStates_t;

extern int Rs485busy;

//volatile portName_t snmpBypassUps;	// in snmp bypass, which ups is connected 1=Master, 2=Slave

typedef enum {
    SONALERT_ON,
    SONALERT_OFF,
    SONALERT_BEEP,
    BLANK_ALL,
    UNBLANK_ALL,
    AUX1LED_ON,
    AUX1LED_OFF,
    AUX1LED_FLASH,
    AUX2LED_ON,
    AUX2LED_OFF,
    AUX2LED_FLASH,
    UPDATE,
    TEST_LED
} alm_states_t;

//alm_states_t almStates;

// upsState definitions

typedef enum {
    UPS_INIT,			// Initial state when router/system powers up
    UPS_OFF,			// System powered but Inverter off
    UPS_ON_BAT,			// On battery, inverter on
    UPS_ON_UTIL,		// On Utility, inverter on
    UPS_BYPASS,			// On bypass
    UPS_SHUTDOWN,		// Shutdown, transition state
    UPS_FAULT,			// System powered, inverter off, indicating fault
    UPS_COM_SHUTDOWN,	// System cutthroat, inverter off, indicating fault
    UPS_NULL			// Used by lcdOptionScreen() lastUpsStateLcd not used for ups state tests
} ups_states_t;

#define DISPLAY_LEN 15
//volatile char loadDisp[DISPLAY_LEN],loadDispLast[DISPLAY_LEN],batDisp[DISPLAY_LEN],batDispLast[DISPLAY_LEN];
//volatile char almStrOne[DISPLAY_LEN],almStrOneLast[DISPLAY_LEN],almStrTwo[DISPLAY_LEN],almStrTwoLast[DISPLAY_LEN];
//volatile char ledDisp[DISPLAY_LEN],ledDispLast[DISPLAY_LEN];

typedef enum checksum_t {
    CHKSUM_ON,				// in process of getting and calculating checksum
    CHKSUM_OFF,				// in between...
    CHKSUM_CR				// getting checksum characters with carriage return termination
} checksum_t;

#define MAX_MASTER_CMDS 20
#define TEMP_BUFFER_LENGTH 100
#define PARAM_NUM_MAX 20
#define PARAM_LEN_MAX 15
#define IDENT_LEN_MAX 50

typedef enum {
    OFF,
    ON,
    AUTO,
    MANUAL,
    AUTO_OFF,
    AUTO_ON,
    AUTO_SLOW,
    AUTO_MED,
    AUTO_FAST,
    MANUAL_OFF,
    MANUAL_ON,
    MANUAL_SLOW,
    MANUAL_MED,
    MANUAL_FAST,
    ON_ALARM,		// from this point on there isn't Model F equivalent mode expressed in RDAT03
    OFF_ALARM,
    NORMAL,			// Note: leave Normal, Warn, Fault and Over_Voltage in this order, they
    WARN,			//	are used in ups_com() and need increasing int values for worse conditions
    FAULT,			//	for upsBoss.batCond testing
    OVER_VOLTAGE,
    OVER_TEMPERATURE,
    DELAY,
    NULL_MODE = -1		// mode not found or invalid
} operatingModesT;

// snmpParse definitions

typedef enum snmpStates_t {
    SNMP_IDLE,
    SNMP_WAITING,
    SNMP_RESPONSE,
    SNMP_BYPASS,
    SNMP2_RESPONSE_DELAY
} snmpStates_t;

//volatile snmpStates_t snmpComState, lastSnmpComState;

/*
#define SNMP_CMD_MAX 24

typedef enum {
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
    NO_SNMP_CMD
} snmpParsePollEnumT;

snmpParsePollEnumT snmpParsePoll;

typedef enum {
    SNMP,
    UPSILON
} snmpParserT;

volatile char snmpParsePollCmd[SNMP_CMD_MAX][5] = {
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
        };

typedef enum {
    START, LEN, DATA
} snmpPartT;

typedef enum {
    POLL,
    SET,
    FAIL,
    UNKNOWN
} snmpTypeT;


#define SNMP_STR_MAX 200
#define SNMP_STR2_MAX 50

struct snmpDataStruct {
	volatile int snmpPort;
	volatile snmpParserT parser;
	volatile snmpStates_t snmpComState, lastSnmpComState;
	volatile char str[SNMP_STR_MAX], aLen[SNMP_STR2_MAX], aData[SNMP_STR_MAX],responseStr[SNMP_STR_MAX], snmpChar;
	volatile int pos, subPos, dataLen, comOkay;
	volatile snmpTypeT type;
	volatile snmpPartT phase;
	volatile snmpParsePollEnumT snmpCmd;
	volatile struct timeT timeOutStart;			// timeOut for waiting for response
	volatile char StrParams[PARAM_NUM_MAX][PARAM_LEN_MAX];		//	when parsed each parameter will be in a string form up to 20 params
	volatile int lastParam;										//	last parameter location, -1 if none, 0 if 1
} snmpDataStruct;
*/

//volatile struct snmpDataStruct snmp, upsilon, *pSnmp, *pUpsilon;


#endif


