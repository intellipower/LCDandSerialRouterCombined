//**************************************************************************************
//
//	System Configuration information
//
// Version 1.08
// 	Added LCD_OPTION_DISABLED to completely disable option select screen, other two
//	option switches not needed
// Version 1.07
//	Added BAT_JOULE_CHARGE_WEIGHTING, changed OVERLOAD_TRIP and OVERLOAD_TRIP_CLEAR
//	settings.
// Version 1.06
//	Added LCD_AUTOSTART_OPTION and LCD_BYPASS_OPTION_DISABLED to keep LCD from showing
//	output frequency selection, it also doesn't show bypass options.
// Version 1.05
//	Changed Ambient temperature trip to 70C, it didn't shut down the inverter before.
// Now it does.
//
//
//
//
//
//**************************************************************************************

// UPS system configuration

#ifndef _system_UPS_h_    // inclusion guard
#define _system_UPS_h_

#include "types.h"

//#define TRUE 1
//#define FALSE 0

#define TEST_COMM_RS485_TO_RS232

#define DESCRIPTION "NOV DRAGON 6KVA Router "
#define autoStart 1						// 0=no auto start, 1=yes
#define UPS_MODEL "6000"
#define LCD_DISPLAY TRUE
#define LCD_OPTION_DISABLED			// disable all options
#define DUAL_BOARD TRUE				// if using 2 Model F with separate battery strings
#define DUAL_BOARD_SERIES	FALSE		// if using dual boards in series
#define UPS_STATE_CONTROL TRUE			// router controls the operation of 1 or 2 UPS
//#define BYPASS_RECOVER				// Will try to recover from overload bypass
//#define OVERLOAD_BYPASS				// Overload will force unit on bypass, not defining will shutdown

#define UPS_MANUFACTURER "INTELLIPOWER"
#define UPS_FREQINNOM 60.0
#define UPS_VOLTINNOM 240.0
#define UPS_TIMELOBATNOM 30.0
#define UPS_FREQOUTNOM 60.0
#define UPS_POWOUTNOM 4500.0
#define UPS_VAOUTNOM 6000.0
#define UPS_VOLTOUTNOM 120.0
// 5KVA 230V = 8 Batteries, Engility 120V = 6 Batteries
#define NUM_BAT 8.0
#define NUM_CELLS NUM_BAT * 6.0

#define TEMP_AMB_ALM_ON 	65.0
#define TEMP_AMB_ALM_OFF 	60.0
#define TEMP_AMB_TRIP_ON 	70.0
#define TEMP_AMB_TRIP_OFF 	65.0

#define TEMP_HS_ALM_ON 	85.0
#define TEMP_HS_ALM_OFF 	75.0
#define TEMP_HS_TRIP_ON	90.0
#define TEMP_HS_TRIP_OFF 	85.0

#define OVERLOAD_ALARM			100.0
#define OVERLOAD_ALARM_CLEAR	95.0
#define OVERLOAD_TRIP			110.0
#define OVERLOAD_TRIP_CLEAR	105.0

#define BAT_CAP_JOULE		0				// This uses router Joule calculation based on battery V & I
#define BAT_CAP_MODEL_F	1				// This uses percent reported by Model F boards
#define BAT_CAP_METHOD	BAT_CAP_JOULE		// Select method, use numbers for future expansion
// Joules in string when fully charged for each string, add for 5KVA systems
#define BAT_MAX_JOULE	((long) (24225000 * 0.5))	// Reduced 5% from 25500000
// If there is a difference between accumulating (charging) and de-accumulating (discharging)
// then you can adjust the charging effect on Joule counting, only active if defined
//#define BAT_JOULE_CHARGE_WEIGHTING (0.8875)
#define BAT_JOULE_CHARGE_WEIGHTING (1.0)
// Communication
#define COM_SNMP
//#define COM_UPSILON
// for Thales, UPS uses RS485 to simulate SNMP request for info to charger
#define SNMP_MIMIC
#define SNMP_MIMIC_CHARGER_THRESHOLD	(2.00)// sets charger current threshold for fast/slow
#define RS485LOCKOUT	// FLAG to lock out the RS485 communications for SNMP connection to charger

// Communication Setup 4800, 9600
#define BAUD_UPS1		4800
#define BAUD_UPS2		4800
#define BAUD_SNMP		9600
#define BAUD_UPSNET		9600

// Communication port selection
typedef enum {
	UPS1_PORT = 0,		// Port 0
	SNMP_PORT = 1,		// Port 1
	UPSNET_PORT = 2,	// Port 2 RS485
	USER_PORT = 3,		// Port 3 User and Upsilon
	UPS2_PORT = 3		// Port 3
} portName_t;

// Program configuration

#define UPS_CHECKSUM TRUE
#define UPS_POLL_INTERVAL 1000		// in milliseconds

// Debug

// 10 levels, if greater than 10 then no reporting internal or RS485 will be done
#define EVENT_REPORT_LEVEL 100
//#define EVENT_REPORT_LEVEL 5
#define RS485LOCKOUT_DEBUG		// Allow debug to get around lockout

// DEBUG_PORT TRUE or FALSE used to switch P11.1 and P11.2
// from outputting MCLK and SMCLK to debug use debug utilize
// with command DEBUG_PORT1_1 DEBUG_PORT1_0 DEBUG_PORT2_1 DEBUG_PORT2_0
// DEBUG_PORT3_1 DEBUG_PORT3_0

//#define DEBUG_PORT

	#ifdef DEBUG_PORT
		#define DEBUG_PORT1_1 P11OUT |=  BIT1
		#define DEBUG_PORT1_0 P11OUT &= ~BIT1
		#define DEBUG_PORT2_1 P11OUT |=  BIT2
		#define DEBUG_PORT2_0 P11OUT &= ~BIT2
		#define DEBUG_PORT3_1 P11OUT |=  BIT3
		#define DEBUG_PORT3_0 P11OUT &= ~BIT3
	#else
		#define DEBUG_PORT1_1 		// null definitions so I don't have to remove references in code
		#define DEBUG_PORT1_0 
		#define DEBUG_PORT2_1 
		#define DEBUG_PORT2_0 
		#define DEBUG_PORT3_1 
		#define DEBUG_PORT3_0 
	#endif

#endif // inclusion guard
