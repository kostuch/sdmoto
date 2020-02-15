#ifndef DIST_H_
#define DIST_H_

//#include "sdmoto18.h"
#define DST_RF_RATE	5															// Czestotliwosc odswierzania dystansow na metromierzu (Hz)
#define DST_DELAY	5000														// Co jaki czas bedzie zapamietywany dystans w EERAM (ms)

uint8_t cur_speed;																// Aktualna predkosc
bool changed_dist_cal;

void openDist(void);
void distances_close(void);
unsigned long calculate_distance(unsigned long pulses);
void update_dist_spd(void);
void dist_cal_up(void);
void dist_cal_dn(void);

#endif /* DIST_H_ */