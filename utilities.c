/*
 * FILE: utilities.c
 * TARGET: MSP430x54x
 * AUTHOR: Mark James
 * DESCRIPTION:  commonly used functions
*/

//#include "types.h"
#include "string.h"
#include "uscia_UART.h"

// Function Prototypes

int iMax(volatile int var1, volatile int var2);
int iMin(volatile int var1, volatile int var2);
int iRange(volatile int test, volatile int limit1, volatile int limit2);
long lMax(volatile long lvar1, volatile long lvar2);
long lMin(volatile long lvar1, volatile long lvar2);
int lRange(volatile long test, volatile long limit1, volatile long limit2);
float fMax(volatile float fvar1, volatile float fvar2);
float fMin(volatile float fvar1, volatile float fvar2);
int fRange(volatile float test, volatile float limit1, volatile float limit2);

// End of Function Prototypes

#define TRUE 0x01
#define FALSE 0x00

// return the maximum of two long point numbers
int iMax(volatile int var1, volatile int var2) {

	int tmpival1, tmpival2;  
  
//  if (var1 >= var2) {
	tmpival1 = var1;
	tmpival2 = var2;
	if (tmpival1 >= tmpival2) 
	{    
		return(var1);
	}
	else 
	{
		return(var2);
	}
}

// return the minimum of two long point numbers
int iMin(volatile int var1, volatile int var2) {
  
	int tmpival1, tmpival2;  
  
//	if (var1 <= var2) {
	tmpival1 = var1;
	tmpival2 = var2;
	if (tmpival1 <= tmpival2) 
	{    
		return(var1);
	}
	else 
	{
		return(var2);
	}
}

// return True if first number is between the other two numbers
int iRange(volatile int test, volatile int limit1, volatile int limit2) {

 	volatile int varMin, varMax, passed;
	int tmpival1, tmpival2, tmpival3;
        
//	varMin = iMin(limit1,limit2);
//	varMax = iMax(limit1,limit2);
	tmpival1 = limit1;
	tmpival2 = limit2;
	tmpival3 = test;
	varMin = iMin(tmpival1, tmpival2);
	varMax = iMax(tmpival1, tmpival2);
          
//	if ((varMin <= test) && (test <= varMax)) {
	if ((varMin <= tmpival3) && (tmpival3 <= varMax))
	{
		passed = TRUE;
	}
	else 
	{
		passed = FALSE;
	}
	return(passed);
}

// return the maximum of two long point numbers
long lMax(volatile long lvar1, volatile long lvar2) {
  
	long tmplval1, tmplval2;
     
//	if (lvar1 >= lvar2) {
	tmplval1 = lvar1;
	tmplval2 = lvar2;
	if (tmplval1 >= tmplval2)   
	{
		return(lvar1);
	}
	else 
	{
		return(lvar2);
	}
}

// return the minimum of two long point numbers
long lMin(volatile long lvar1, volatile long lvar2) {

     long tmplval1, tmplval2;
     
//	if (lvar1 <= lvar2) {
	tmplval1 = lvar1;
	tmplval2 = lvar2;
	if (tmplval1 <= tmplval2)   
	{
		return(lvar1);
	}
	else 
	{
		return(lvar2);
	}
}

// return True if first number is between the other two numbers
int lRange(volatile long test, volatile long limit1, volatile long limit2) {

	volatile long varMin, varMax;
	long tmplval1, tmplval2, tmplval3;
        
//	varMin = lMin(limit1,limit2);
//	varMax = lMax(limit1,limit2);
	tmplval1 = limit1;
	tmplval2 = limit2;
	tmplval3 = test;
	varMin = lMin(tmplval1, tmplval2);
	varMax = lMax(tmplval1, tmplval2);        

//	if ((varMin <= test) && (test <= varMax)) {
	if ((varMin <= tmplval3) && (tmplval3 <= varMax))        
	{
                return(TRUE);
	}
	else 
	{
		return(FALSE);
	}
}

// return the maximum of two floating point numbers
float fMax(volatile float fvar1, volatile float fvar2) {

	float tmpfval1, tmpfval2;
  
//	if (fvar1 >= fvar2) {
	tmpfval1 = fvar1;
	tmpfval2 = fvar2;
	if (tmpfval1 >= tmpfval2)   
	{
                return(fvar1);
	}
	else 
	{
		return(fvar2);
	}
}

// return the minimum of two floating point numbers
float fMin(volatile float fvar1, volatile float fvar2) {
  
	float tmpfval1, tmpfval2;
  
//	if (fvar1 <= fvar2) {
	tmpfval1 = fvar1;
	tmpfval2 = fvar2;
	if (tmpfval1 <= tmpfval2)   
	{
		return(fvar1);
	}
	else 
	{
		return(fvar2);
	}
}

// return True if first number is between the other two numbers
int fRange(volatile float test, volatile float limit1, volatile float limit2) {

	volatile float varMin, varMax;
	float tmpfval1, tmpfval2, tmpfval3;

//	varMin = fMin(limit1,limit2);
//	varMax = fMax(limit1,limit2);
	tmpfval1 = limit1;
	tmpfval2 = limit2;
	tmpfval3 = test;
	varMin = fMin(tmpfval1, tmpfval2);
	varMax = fMax(tmpfval1, tmpfval2);      
        
//	if ((varMin <= test) && (test <= varMax)) {
	if ((varMin <= tmpfval3) && (tmpfval3 <= varMax))        
	{        
		return(TRUE);
	}
	else 
	{
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

#if (defined COM_DEBUG)

	void comDebug (char *str) {
		char tempStr[100];
		
		strcpy(tempStr, str);
		strcat(tempStr, "\r\n");
		usart_putstr(tempStr, SNMP_PORT);
	}

#endif // defined COM_DEBUG
	
