#ifndef TIMER_H_
#define TIMER_H_
#include "Arduino.h"

uint32_t leap_times[5];
uint32_t current_time;
uint8_t leap_idx;

void openTime(void);
void closeTime(void);
void btnClrTimes(void);

#endif /* TIMER_H_ */