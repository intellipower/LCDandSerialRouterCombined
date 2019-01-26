#ifndef TIMERA_H_
#define TIMERA_H_

/*
struct timeT {
	volatile int days;
	volatile long msec;
};
*/
// void timera_PWM_set_duty(uint16_t period_counts, uint16_t high_counts);
void timera_init(void);
/*
struct timeT getTime(void);
int timer(volatile struct timeT timestart, volatile long delay);
*/

#endif /*TIMERA_H_*/
