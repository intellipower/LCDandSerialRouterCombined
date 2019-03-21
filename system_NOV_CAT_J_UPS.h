//**************************************************************************************
//
//	System Configuration information
//
// Version 1.09
//		Added NOV_CAT_J compiler switch to allow an unused SNMP->CIP parameter Number of Input Lines to be used to report Bender Ground Fault and Impedance fault status.  0=No Fault, 1=Ground Fault, 2=Impedance Fault and 3=Both Faults
// Version 1.08
//		System wasn't shutting down on overload, added #define OVERLOAD_BYPASS_DISABLE
// Version 1.07
//		Cut the joules per string by half.
// Version 1.06
//		Added SNMP_MIMIC_CHARGER_THRESHOLD, changed overload alarm and trips to NOV Dragon levels.
//		Put in OLD_SHUTDOWN so we can do apples/apples comparison when we find a misbehaving unit.
//		Modified Joules from 8 to 9 batteries.
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

#define DESCRIPTION "NOV CAT-J 6KVA Router "
#define autoStart 1						// 0=no auto start, 1=yes
#define UPS_MODEL "6000"
#define LCD_DISPLAY TRUE
#define LCD_OPTION_DISABLED			// disable all options
#define DUAL_BOARD TRUE				// if using 2 Model F with separate battery strings
#define DUAL_BOARD_SERIES	FALSE		// if using dual boards in series
#define UPS_STATE_CONTROL TRUE			// router controls the operation of 1 or 2 UPS
#define OVERLOAD_BYPASS_DISABLE		// Prevent overload from going to bypass
//#define BYPASS_RECOVER					// Will try to recover from overload bypass
//#define OVERLOAD_BYPASS				// Overload will force unit on bypass, not defining will shutdown
#define NOV_CAT_J						// Use ST2 Input Lines to report Bender status

//***
#define OLD_SHUTDOWN					// remove to use new shutdown method to (hopefull) prevent slave DC out at shutdown
//***

#define UPS_MANUFACTURER "INTELLIPOWER"
#define UPS_FREQINNOM 60.0
#define UPS_VOLTINNOM 690.0
#define UPS_TIMELOBATNOM 30.0
#define UPS_FREQOUTNOM 60.0
#define UPS_POWOUTNOM 5500.0
#define UPS_VAOUTNOM 6000.0
#define UPS_VOLTOUTNOM 120.0
// 5KVA 230V = 8 Batteries, Engility 120V = 6 Batteries
#define NUM_BAT 9.0
#define NUM_CELLS NUM_BAT * 6.0

#define TEMP_AMB_ALM_ON 	65.0
#define TEMP_AMB_ALM_OFF 	60.0
#define TEMP_AMB_TRIP_ON 	70.0
#define TEMP_AMB_TRIP_OFF 	65.0

#define TEMP_HS_ALM_ON 	85.0
#define TEMP_HS_ALM_OFF 	75.0
#define TEMP_HS_TRIP_ON	90.0
#define TEMP_HS_TRIP_OFF 	85.0

#define OVERLOAD_ALARM			103.0
#define OVERLOAD_ALARM_CLEAR	98.0
#define OVERLOAD_TRIP			110.0
#define OVERLOAD_TRIP_CLEAR	105.0

#define BAT_CAP_JOULE		0				// This uses router Joule calculation based on battery V & I
#define BAT_CAP_MODEL_F	1				// This uses percent reported by Model F boards
#define BAT_CAP_METHOD	BAT_CAP_JOULE		// Select method, use numbers for future expansion
// Joules in string when fully charged for each string, add for 5KVA systems
// Uses 12V 
//#define BAT_MAX_JOULE	((long) 24225000)	// Reduced 5% from 25500000
// Cutting the maximum joules by half is due to incorrect starting point, this is done in NOV Dragon as well
// this unit has 9 12V 55AH batteries, according to battery spec it can support 3KW (including losses)
// for 2 hours at 21 Amps = 16.33e6 joules per string
#define BAT_MAX_JOULE	((long) 27253130 * 0.5)// 8 => 9 batteries

// Communication
#define COM_SNMP
//#define COM_UPSILON
#define SNMP_MIMIC		// for Thales, UPS uses RS485 to simulate SNMP request for info to charger
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
