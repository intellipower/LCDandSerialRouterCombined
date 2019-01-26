#ifndef TIMERB_H_
#define TIMERB_H_

#include "system_config.h"						// includes file name for configuration
#include SYSTEM_CONFIG							// configuration header file

#if X2_CLOCK_MHZ == 16		// Which frequency crystal is in X2
	#define TIMERB_ISR_INTERVAL_1mSEC 16000		// PWM Period, ACLK = 16e6/1000 ~1msec => 16000
#else
	#define TIMERB_ISR_INTERVAL_1mSEC 7373		// PWM Period, ACLK = 7.3728e6/1000 ~1msec => 7373
#endif

#define TIMERB_125_mSEC 125
#define TIMERB_250_mSEC 250
#define TIMERB_500_mSEC 500
#define TIMERB_1000_mSEC 1000
#define TIMERB_1500_mSEC 1500
#define TIMERB_2000_mSEC 2000
#define TIMERB_3000_mSEC 3000
#define TIMERB_5000_mSEC 5000
#define TIMERB_10000_mSEC 10000
#define TIMERB_15000_mSEC 15000
#define TIMERB_20000_mSEC 20000

#define ALARM2_DELAY(value)\
	alarm2_set(value);\
	while(!alarm2_read())

struct timeT {
	volatile int days;
	volatile long msec;
};

//volatile unsigned int timer_b_isr_count;

struct timeT getTime(void);
int timer(volatile struct timeT timestart, volatile long delay);
void alarm1_set(volatile unsigned int value);
void alarm1_switch(volatile unsigned int enable);
unsigned int alarm1_read(void);

void alarm2_set(volatile unsigned int value);
void alarm2_switch(volatile unsigned int enable);
unsigned int alarm2_read(void);

void alarm3_set(volatile unsigned int value);
void alarm3_switch(volatile unsigned int enable);
unsigned int alarm3_read(void);

void timerb_init(void);

#endif /*TIMERB_H_*/
