//**************************************************************************************
//
//	System Configuration information
//
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
// This version is for Joint Cheifs of Staff and has WRPFC
// It also disables the LCD Bypass Option by removing it from option screen

#ifndef _system_UPS_h_    // inclusion guard
#define _system_UPS_h_

#include "types.h"

//#define TRUE 1
//#define FALSE 0
//#define NOV_UPS FALSE

#define DESCRIPTION "Briefcase Lithium "
#define autoStart 0
#define UPS_MODEL "1150VA"
#define LCD_DISPLAY TRUE
#define DUAL_BOARD FALSE				// if using 2 Model F with separate battery strings
#define DUAL_BOARD_SERIES	FALSE		// if using dual boards in series
#define UPS_STATE_CONTROL TRUE		// router controls the operation of 1 or 2 UPS
#define LCD_BYPASS_OPTION_DISABLED		// Do not display Bypass option on LCD
#define LCD_MANAGER_BOSS

#define UPS_MANUFACTURER "INTELLIPOWER"
#define UPS_FREQINNOM 60.0
#define UPS_VOLTINNOM 240.0
#define UPS_TIMELOBATNOM 30.0
#define UPS_FREQOUTNOM 60.0
#define UPS_POWOUTNOM 800.0
#define UPS_VAOUTNOM 1150.0
#define UPS_VOLTOUTNOM 120.0
// 5KVA 230V = 8 Batteries, Engility 120V = 6 Batteries
#define NUM_BAT 3.0
#define NUM_CELLS NUM_BAT * 6.0
// comment out if to use default, if in it will use values below, they are in volts/cell
#define BAT_SETPOINT_CHANGE
#define BAT_SET_BAT_SD_LOW_PWR			(1.75)
#define BAT_SET_BAT_WARN_LOW_PWR		(1.95)
#define BAT_SET_BAT_SD_NORM_PWR		    (1.67)
#define BAT_SET_BAT_WARN_NORM_PWR		(1.75)

#define TEMP_AMB_ALM_ON 	65.0
#define TEMP_AMB_ALM_OFF 	60.0
#define TEMP_AMB_TRIP_ON 	70.0
#define TEMP_AMB_TRIP_OFF 	65.0

#define TEMP_HS_ALM_ON 	85.0
#define TEMP_HS_ALM_OFF 	75.0
#define TEMP_HS_TRIP_ON	90.0
#define TEMP_HS_TRIP_OFF 	85.0

#define OVERLOAD_ALARM			105.0
#define OVERLOAD_ALARM_CLEAR	100.0
#define OVERLOAD_TRIP			115.0
#define OVERLOAD_TRIP_CLEAR	110.0

#define BAT_CAP_JOULE		0				// This uses router Joule calculation based on battery V & I
#define BAT_CAP_MODEL_F	1				// This uses percent reported by Model F boards
#define BAT_CAP_METHOD	BAT_CAP_MODEL_F		// Select method, use numbers for future expansion
// Joules in string when fully charged for each string, add for 5KVA systems
// Lithium 6AH * 12V * 3600sec/hr * 3 bat = 777.6e3 => at 800W = 16.2 minutes (target is 18 minutes)
#define BAT_MAX_JOULE	((long) 777600)

//#define COM_SNMP
#define COM_UPSILON

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
//#define EVENT_REPORTING_ENABLED
#define EVENT_REPORT_LEVEL 100
//#define EVENT_REPORT_LEVEL 5

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
