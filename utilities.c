/*
 * FILE: utilities.c
 * TARGET: MSP430x54x
 * AUTHOR: Mark James
 * DESCRIPTION:  commonly used functions
*/

//#include "types.h"
#define TRUE 0x01
#define FALSE 0x00

// return the maximum of two long point numbers
int iMax(volatile int var1, volatile int var2) {
	if (var1 >= var2) {
		return(var1);
	}
	else {
		return(var2);
	}
}

// return the minimum of two long point numbers
int iMin(volatile int var1, volatile int var2) {
	if (var1 <= var2) {
		return(var1);
	}
	else {
		return(var2);
	}
}

// return True if first number is between the other two numbers
int iRange(volatile int test, volatile int limit1, volatile int limit2) {

	volatile int varMin, varMax, passed;

	varMin = iMin(limit1,limit2);
	varMax = iMax(limit1,limit2);

	if ((varMin <= test) && (test <= varMax)) {
		passed = TRUE;
	}
	else {
		passed = FALSE;
	}
	return(passed);
}

// return the maximum of two long point numbers
long lMax(volatile long lvar1, volatile long lvar2) {
	if (lvar1 >= lvar2) {
		return(lvar1);
	}
	else {
		return(lvar2);
	}
}

// return the minimum of two long point numbers
long lMin(volatile long lvar1, volatile long lvar2) {
	if (lvar1 <= lvar2) {
		return(lvar1);
	}
	else {
		return(lvar2);
	}
}

// return True if first number is between the other two numbers
int lRange(volatile long test, volatile long limit1, volatile long limit2) {

	volatile long varMin, varMax;

	varMin = lMin(limit1,limit2);
	varMax = lMax(limit1,limit2);

	if ((varMin <= test) && (test <= varMax)) {
		return(TRUE);
	}
	else {
		return(FALSE);
	}
}

// return the maximum of two floating point numbers
float fMax(volatile float fvar1, volatile float fvar2) {
	if (fvar1 >= fvar2) {
		return(fvar1);
	}
	else {
		return(fvar2);
	}
}

// return the minimum of two floating point numbers
float fMin(volatile float fvar1, volatile float fvar2) {
	if (fvar1 <= fvar2) {
		return(fvar1);
	}
	else {
		return(fvar2);
	}
}

// return True if first number is between the other two numbers
int fRange(volatile float test, volatile float limit1, volatile float limit2) {

	volatile float varMin, varMax;

	varMin = fMin(limit1,limit2);
	varMax = fMax(limit1,limit2);

	if ((varMin <= test) && (test <= varMax)) {
		return(TRUE);
	}
	else {
		return(FALSE);
	}
}

// Test functions, not used for normal operation
#if defined TEST_COMM_RS485_TO_RS232		// calls function with Forever loop to sniff RS485 and put it on SNMP RS232
	void testCommRs485toRs232 (void) {
		/*
		volatile struct snmpDataStruct Rs485, Rs232, *pRs485, *pRs232;
		pRs485 = &Rs485;				// address of structure into pointer var
		pRs232 = &Rs232;				// address of structure into pointer var
		snmp.snmpPort = SNMP_PORT;
		Forever {
			if ((usart_peekchar(parseType->snmpPort) == '^') && (parseType->pos > 0)) {
			}
		}
		*/
	}
#endif

	
