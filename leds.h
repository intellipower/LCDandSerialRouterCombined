#ifndef LEDS_H_
#define LEDS_H_

// Set LED pin assignments
#define RED0 BIT0		//0x01
#define GRN0 BIT1		//0x02
#define RED1 BIT2		//0x04
#define GRN1 BIT3		//0x08
#define RED2 BIT4		//0x10
#define GRN2 BIT5		//0x20
#define RED3 BIT6		//0x40
#define GRN3 BIT7		//0x80
#if (defined CONTROL_TOP_LEDS)
    // Top leds
    #define RED0_TOP BIT2
    #define GRN0_TOP BIT3
    #define RED1_TOP BIT0
    #define GRN1_TOP BIT1
    #define RED2_TOP BIT6
    #define GRN2_TOP BIT7
    #define RED3_TOP BIT2
    #define GRN3_TOP BIT3
#endif                  // defined CONTROL_TOP_LEDS
    
// LED0 control
#define RED0_ON  P2OUT &= ~RED0;
#define RED0_OFF P2OUT |= RED0;
#define GRN0_ON  P2OUT &= ~GRN0;
#define GRN0_OFF P2OUT |= GRN0;
#if (defined CONTROL_TOP_LEDS)
    // Top leds
    #define RED0_ON_TOP  P9OUT &= ~RED0_TOP;
    #define RED0_OFF_TOP P9OUT |= RED0_TOP;
    #define GRN0_ON_TOP  P9OUT &= ~GRN0_TOP;
    #define GRN0_OFF_TOP P9OUT |= GRN0_TOP;
#endif                  // defined CONTROL_TOP_LEDS

// LED1 control
#define RED1_ON  P2OUT &= ~RED1;
#define RED1_OFF P2OUT |= RED1;
#define GRN1_ON  P2OUT &= ~GRN1;
#define GRN1_OFF P2OUT |= GRN1;
#if (defined CONTROL_TOP_LEDS)
    // Top leds
    #define RED1_ON_TOP  P9OUT &= ~RED1_TOP;
    #define RED1_OFF_TOP P9OUT |= RED1_TOP;
    #define GRN1_ON_TOP  P9OUT &= ~GRN1_TOP;
    #define GRN1_OFF_TOP P9OUT |= GRN1_TOP;
#endif                  // defined CONTROL_TOP_LEDS

// LED2 control
#define RED2_ON  P2OUT &= ~RED2;
#define RED2_OFF P2OUT |= RED2;
#define GRN2_ON  P2OUT &= ~GRN2;
#define GRN2_OFF P2OUT |= GRN2;
#if (defined CONTROL_TOP_LEDS)
    // Top leds
    #define RED2_ON_TOP  P8OUT &= ~RED2_TOP;
    #define RED2_OFF_TOP P8OUT |= RED2_TOP;
    #define GRN2_ON_TOP  P8OUT &= ~GRN2_TOP;
    #define GRN2_OFF_TOP P8OUT |= GRN2_TOP;
#endif                  // defined CONTROL_TOP_LEDS

// LED3 control
#define RED3_ON  P2OUT &= ~RED3;
#define RED3_OFF P2OUT |= RED3;
#define GRN3_ON  P2OUT &= ~GRN3;
#define GRN3_OFF P2OUT |= GRN3;
#if (defined CONTROL_TOP_LEDS)
    // Top leds
    #define RED3_ON_TOP  P8OUT &= ~RED3_TOP;
    #define RED3_OFF_TOP P8OUT |= RED3_TOP;
    #define GRN3_ON_TOP  P8OUT &= ~GRN3_TOP;
    #define GRN3_OFF_TOP P8OUT |= GRN3_TOP;
#endif                  // defined CONTROL_TOP_LEDS

// LED0 Color Macros
#define LED0_OFF RED0_OFF; GRN0_OFF;
#define LED0_RED RED0_ON;  GRN0_OFF;
#define LED0_GRN RED0_OFF; GRN0_ON;
#define LED0_YLW RED0_ON;  GRN0_ON;

// LED1 Color Macros
#define LED1_OFF RED1_OFF; GRN1_OFF;
#define LED1_RED RED1_ON;  GRN1_OFF;
#define LED1_GRN RED1_OFF; GRN1_ON;
#define LED1_YLW RED1_ON;  GRN1_ON;

// LED2 Color Macros
#define LED2_OFF RED2_OFF; GRN2_OFF;
#define LED2_RED RED2_ON;  GRN2_OFF;
#define LED2_GRN RED2_OFF; GRN2_ON;
#define LED2_YLW RED2_ON;  GRN2_ON;

// LED3 Color Macros
#define LED3_OFF RED3_OFF; GRN3_OFF;
#define LED3_RED RED3_ON;  GRN3_OFF;
#define LED3_GRN RED3_OFF; GRN3_ON;
#define LED3_YLW RED3_ON;  GRN3_ON;

#endif /*LEDS_H_*/
