//**************************************************************************************
//
// System Configuration information
//
// Version 1.05
// Changed Ambient temperature trip to 70C, it didn't shut down the inverter before.
// Now it does.
//
//
//
//
//
//**************************************************************************************

// UPS system configuration

#ifndef _system_config_h_    // inclusion guard
#define _system_config_h_
// make sure RS485 Debug is off!! (until debug is debugged, it interferes with RS232)
#define VERSION "1.63iX03 2019-11-11"

#define X2_CLOCK_FREQUENCY ((long) 16000000)    // Crystal clock speed
//#define X2_CLOCK_FREQUENCY ((long) 25000000)  // Crystal clock speed

#define CONTROL_TOP_LEDS                        // For versions Rev E X01 and above
//#define COM_DEBUG                             // sends communication to selected port

#define CONFIG_AITA_6KVA_4_2KW_CH10251 "system_AITA_6KVA_4_2KW_CH10251.h"
#define CONFIG_CANES4_220VIN        "system_Canes4_UPS_220VIN.h"    // FA00307 was this one changed to 440 in
#define CONFIG_CANES4_440VIN        "system_Canes4_UPS_440VIN.h"    // FA00307
#define CONFIG_CANES_6_6KVA_440VIN  "system_CANES_6.6KVA_UPS_FA10249.h"
#define CONFIG_COMTECH_8KVA         "system_Comtech_8KVA_UPS_FA10140.h"
#define CONFIG_CRYSTAL              "system_Crystal_UPS.h"
#define CONFIG_BAKER_HUGHES         "system_BakerHughes_UPS.h"
#define CONFIG_BRIEFCASE_LITHIUM    "system_Briefcase_Lithium.h"    //  FA10003
#define CONFIG_GYRODATA_FA10070     "system_GyroData_FA10070.h"     // Briefcase Lithium with SNMP
#define CONFIG_AMSEC                "system_AMSEC_UPS.h"
#define CONFIG_GD_SATCOM            "system_GDSATCOM_UPS.h"
#define CONFIG_THALES_3KVA_UPS      "system_Thales_3KVA_UPS.h"
#define CONFIG_THALES_5KVA_UPS      "system_Thales_5KVA_UPS.h"
#define CONFIG_THALES_5KVA_CHGR     "system_Thales_5KVA_Dual_Charger.h"
#define CONFIG_TRANSTECTOR_5KVA_UPS "system_TRANSTECTOR_UPS_5KVA_FA00424.h"
#define CONFIG_NOV                  "system_NOV_UPS.h"
#define CONFIG_NOV_CAT_J_UPS        "system_NOV_CAT_J_UPS.h"
#define CONFIG_NOV_CAT_J_CHGR       "system_NOV_CAT-J_Dual_Charger.h"
#define CONFIG_NOV_DRAGON_UPS       "system_NOV_DRAGON_UPS.h"       // 6KVA Monster UPS with LCD
#define CONFIG_NOV_DRAGON_CHGR      "system_NOV_Dragon_Dual_Charger.h"
#define CONFIG_NSWC3                "system_NSWC3_UPS.h"
#define CONFIG_NSWC4                "system_NSWC4_UPS.h"
#define CONFIG_FA10002              "system_FA10002_UPS.h"          // UCLASS, 115Vout, 4.2KW
#define CONFIG_FA10021              "system_FA10021_UPS.h"          // UCLASS, 240Vout, 4.2KW
#define CONFIG_FA10241				"system_FA10241_NAVFAC_6KVA_4.3KW_UPS.h" // Monster
#define CONFIG_FAIRLEAD_5KVA        "system_FAIRLEAD_UPS_DUAL_5KVA_4KW.h"
// NOV Pressure 3KVA 2.5KW with charger Model F Board and large battery string
// fixed bug 2014-03-21, check LCD and Serial Router NOV Working folder
// search for compile flags NOV_UPS and AUTO_RESTART, this bug caused system to hang at startup
// with LCD display showing "Initializing...", it was looking for inverter auto/on to continue
// but sometimes Model F would return manual/on
#define CONFIG_NOV     "system_NOV_UPS.h"

// **** Test Configurations
#define TEST_CONFIG_RS485_TO_RS232 "testCommRs485toRs232.h"


#define SYSTEM_CONFIG CONFIG_TRANSTECTOR_5KVA_UPS


#endif // inclusion guard
