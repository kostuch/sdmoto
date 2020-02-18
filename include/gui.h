#ifndef GUI_H_
#define GUI_H_

#ifdef __cplusplus
extern "C"
{
	#include "dist.h"
	#include "timer.h"
	#include "navi.h"
	#include "combo.h"
	#include "gps.h"
}
#endif

// Koordynaty ikon na pasku
// 160x128
#define TBARX_GPS		0
#define TBARX_WIFI		32
#define TBARX_MEMORY	64
#define TBARX_SD		96
#define TBARX_TIME		128
#define UTLY			32

typedef struct
{
	uint8_t		screen_id;
	uint8_t		key_id;
	uint8_t		x, y, w, h;
	char		*lbl;
	void		(*key_exec)(void);
} btn_t;

btn_t key_buf;

const btn_t ctrls_data[] PROGMEM =
{
	{0, 0, 65, 110, 32, 12, (char *) "PAUZA", btnStopStart},
	{1, 0, 70, 100, 80, 12, (char *) "Usun czasy", btnClrTimes},
	{2, 0, 3, 35, 70, 12, (char *) "Zapis TRK", btnSaveTrk},
	{2, 1, 3, 49, 70, 12, (char *) "Zapis WPT", btnSaveWpt},
	{2, 2, 3, 63, 70, 12, (char *) "Navi do WPT", btnNav2Wpt},
};

typedef struct
{
	uint8_t screen_id;
	char *screen_name;
	void (*scr_open_exe)(void);
	void (*scr_close_exe)(void);
} screen_t;

const screen_t screen_data[] PROGMEM =
{
	{0, (char *) "Metromierz", openDist, NULL},									// Metromierze
	{1, (char *) "Stoper", openTime, NULL},										// Stoper
	{2, (char *) "Nawigacja", openNavi, NULL},									// Nawigacja
	{3, (char *) "Wskazniki", openCombo, NULL},									// Wskazniki
	{4, (char *) "GPS", openGPS, NULL}											// GPS
};

void renderScreen(void);

#endif /* GUI_H_ */