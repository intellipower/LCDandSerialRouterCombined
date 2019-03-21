#ifndef UTILITIES_H_
#define UTILITIES_H_

// function prototypes
int iMax(volatile int var1, volatile int var2);
int iMin(volatile int var1, volatile int var2);
int iRange(volatile int test, volatile int limit1, volatile int limit2);
long lMax(volatile long lvar1, volatile long lvar2);
long lMin(volatile long lvar1, volatile long lvar2);
int lRange(volatile long test, volatile long limit1, volatile long limit2);
float fMax(volatile float fvar1, volatile float fvar2);
float fMin(volatile float fvar1, volatile float fvar2);
int fRange(volatile float test, volatile float limit1, volatile float limit2);
// test functions, not used in normal operation
#if defined TEST_COMM_RS485_TO_RS232		// calls function with Forever loop to sniff RS485 and put it on SNMP RS232
	void testCommRs485toRs232 (void);
#endif

#if (defined COM_DEBUG)
	void comDebug (char *str);
#endif // defined COM_DEBUG

#endif /*UTILITIES_H_*/
