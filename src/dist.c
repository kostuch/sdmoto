#include <Arduino.h>
#include "dist.h"

void openDist()
{
	
}

void closeDist() {}

unsigned long calculate_distance(unsigned long pulses)
{
	//unsigned long scale = 100000 / dev_config.dist_cal;							// Skalowanie
	//return (pulses * scale / 1000);
	return 0;
}
/* 
void dist_cal_up()
{
	calibrations.dist_cal++;
	changed_dist_cal = true;
}

void dist_cal_dn()
{
	calibrations.dist_cal--;
	changed_dist_cal = true;
}

void update_dist_spd()
{
	// Obwod kola = 2m
	// 4imp na obrot kola -> 1m = 2imp
	// dist_calibration = 200 -> scale = 500
	// Symulacja predkosci 10m/s = 36km/h to ++20 przy DST_RF_RATE=1
	pulses_cnt1 += 2; pulses_cnt2 += 2; pulses_spd += 2;
	static uint16_t old_speed = 0;
	//static unsigned long old_distance1, old_distance2;
	static uint32_t dist_time;
	imp_dist1 = calculate_distance(pulses_cnt1);								// Wyliczenie realnego dystansu
	imp_dist2 = calculate_distance(pulses_cnt2);
	dist4speed = calculate_distance(pulses_spd) * DST_RF_RATE;					// Dystans do wyliczenia predkosci

	if (imp_dist1 > 999999) imp_dist1 = 999999;									// Ograniczenie gornych dystansow

	if (imp_dist2 > 999999) imp_dist2 = 999999;

	
	tft.setFont(&FreeMonoBold18pt7b);
	tft.setTextSize(2);
	uint8_t x;
	x = dist_position_erase(distance1, old_distance1);							// Wymazanie starych dystansow
	tft.fillRect(x, 41, 254 - x, 68, ILI9341_BLACK);
	x = dist_position_erase(distance2, old_distance2);
	tft.fillRect(x, 121, 254 - x, 68, ILI9341_BLACK);
	tft.setCursor(0, 100);
	tft.setTextColor(ILI9341_YELLOW);
	tft.printf_P(PSTR("%6lu"), distance1);										// Wyrownanie do prawej
	tft.setCursor(0, 180);
	tft.setTextColor(ILI9341_ORANGE);
	tft.printf_P(PSTR("%6lu"), distance2);										// Wyrownanie do prawej
	
	cur_speed = dist4speed * 3.6;												// Zamiana m/s na km/h
	cur_speed = (old_speed + cur_speed) / 2;									// Srednia z dwoch pomiarow
	//speed.update(cur_speed);													// Uaktualnij predosciomierz "analogowy"
	old_speed = cur_speed;														// Zapamietaj poprzednia predkosc
	//old_distance1 = distance1;													// Zapamietaj poprzednie dystanse
	//old_distance2 = distance2;
	pulses_spd = 0;																// Zerowanie impulsow naliczonych przez sekunde

	if ((millis() - dist_time) > DST_DELAY)										// Jezeli minal czas do zapisu w EERAM
	{
		eeram_save32(DIST1, pulses_cnt1);
		eeram_save32(DIST2, pulses_cnt2);
		dist_time = millis();													// Uaktualnij czas ostatniego zapisu
	}
}
 */