
#include "IAR_YES_NO.h"
#ifdef __IAR
	#include "io430.h"
#else
	#include <msp430x54x.h>
#endif

//#include "types.h"
//#include "timera.h"

// Function Prototypes

__interrupt void ADC12 (void);
__interrupt void DMA (void);
__interrupt void PORT1 (void);
__interrupt void PORT2 (void);
__interrupt void Sysnmi (void);
__interrupt void Timer0_A1 (void);
__interrupt void Timer1_A0 (void);
__interrupt void Timer1_A1 (void);
__interrupt void Timer_B1 (void);
__interrupt void Unmi(void);
__interrupt void USCI_B0 (void);
__interrupt void USCI_B1 (void);
__interrupt void USCI_B2 (void);
__interrupt void USCI_B3 (void);
__interrupt void WDT (void);
__interrupt void RTC_ISR (void);

// End of Function Prototypes

// ADC12 interrupt service routine
#pragma vector=ADC12_VECTOR
__interrupt void ADC12 (void){
}

// DMA interrupt service routine
#pragma vector=DMA_VECTOR
__interrupt void DMA (void){
}

// PORT1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void PORT1 (void){
}

// PORT2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void PORT2 (void){
}

// SYSNMI interrupt service routine
#pragma vector=SYSNMI_VECTOR
__interrupt void Sysnmi (void){
}
/*
// Timer0 A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void){
}
*/
// Timer0 A1 interrupt service routine
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1 (void){
}

// Timer1 A0 interrupt service routine
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0 (void){
}

// Timer0 A1 interrupt service routine
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1 (void){
}

// Timer B0 interrupt service routine
#pragma vector=TIMERB1_VECTOR
__interrupt void Timer_B1 (void){
}
/*
// Timer B0 interrupt service routine
#pragma vector=TIMER0_B0_VECTOR
__interrupt void Timer0_B0 (void){
}
*/
// UNMI interrupt service routine
#pragma vector=UNMI_VECTOR
__interrupt void Unmi(void){
}

// USCI_B0 interrupt service routine
#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0 (void){
}

// USCI_B1 interrupt service routine
#pragma vector=USCI_B1_VECTOR
__interrupt void USCI_B1 (void){
}

// USCI_B2 interrupt service routine
#pragma vector=USCI_B2_VECTOR
__interrupt void USCI_B2 (void){
}

// USCI_B3 interrupt service routine
#pragma vector=USCI_B3_VECTOR
__interrupt void USCI_B3 (void){
}

// WDI interrupt service routine
#pragma vector=WDT_VECTOR
__interrupt void WDT (void){
}

// RTC interrupt service routine
#pragma vector=RTC_VECTOR
__interrupt void RTC_ISR (void){
}




