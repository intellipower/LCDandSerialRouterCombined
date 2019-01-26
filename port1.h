#ifndef PORT1_H_
#define PORT1_H_

// Define bit values for each button for masking and testing
#define BUTTON1 BIT0
#define BUTTON2 BIT1
#define BUTTON3 BIT2
#define BUTTON4 BIT3

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

// Define the time that each button must be held for
#define BUTTON1_DURATION TIMERB_2000_mSEC
#define BUTTON2_DURATION TIMERB_500_mSEC
#define BUTTON3_DURATION TIMERB_500_mSEC
#define BUTTON4_DURATION TIMERB_500_mSEC

void buttons_init(void);
unsigned int button1_query(void);
unsigned int button2_query(void);
unsigned int button3_query(void);
unsigned int button4_query(void);
unsigned int buttons_query(void);
unsigned int buttons_peek(void);
void button1_service(unsigned int timer_b_isr_count);
void button2_service(unsigned int timer_b_isr_count);
void button3_service(unsigned int timer_b_isr_count);
void button4_service(unsigned int timer_b_isr_count);

#endif /*PORT1_H_*/
