#include <pgmspace.h>

#ifndef SDMOTO_H_
#define SDMOTO_H_

//#define HOST_NAME		"sdmoto18"												// Na potrzeby RemoteDebug
#define HW_MAJOR_VER	1														// Wersja Hardware
#define HW_MINOR_VER	0

#define I2C_EXP_A		0x20
#define I2C_EERAM_A		0x50
#define SDA_PAD			4														// GPIO
#define SCL_PAD			5														// GPIO
#define BL_PIN			15														// GPIO
#define IMP_IRQ_PIN		0														// GPIO
#define I2C_IRQ_PIN		2														// GPIO
#define TFT_DC_PIN		16														// GPIO
#define TFT_CS_PIN		0														// I2C bit
#define SDC_CS_PIN		1														// I2C bit
#define MUX_PIN			2														// I2C bit
#define KEY_RT			3														// I2C bit
#define KEY_LT			4														// I2C bit
#define KEY_UP			5														// I2C bit
#define KEY_DN			6														// I2C bit
#define KEY_RST			7														// I2C bit
// EERAM adresy
#define EERAM_BASE	0x0000														// Pierwszy adres w EERAM
#define LAST_SCREEN	EERAM_BASE													// Ostatni ekran
#define DIST1		(LAST_SCREEN + 1)											// Dystans odcinka (impulsy lub dystans)
#define DIST2		(DIST1 + 4)													// Dystans calkowity (impulsy lub dystans)
#define LAST_WPT	(DIST2 + 4)													// Ostatni waypoint z listy do ktorego byla nawigacja
#define DIST_CAL	(LAST_WPT + 2)												// Kalibracja dystansu
#define VOLT_CAL	(DIST_CAL + 2)												// Kalibracja napiecia
#define TEMP_CAL	(VOLT_CAL + 2)												// Kalibracja temperatury
#define LONG_PRESS	500															// Czas dlugiego wcisniecia [ms]

enum MUX_STATES		{STARTUP = 1, RUNTIME = 0};									// Stany multipleksera sygnalow
enum BUTTONS		{BTN_RELEASED = 0, BTN_RST = 7, BTN_UP = 5, BTN_DN = 6, BTN_LT = 4, BTN_RT = 3};
enum TOOLBAR_ITEMS	{WIFI_XOFF, WIFI_XSTA, WIFI_XAP, GPS_NOFIX, GPS_FIX, GPS_DATETIME, MEMORY, SD_OK, SD_NOOK, SD_OFF};
enum SCREENS		{SCR_DIST, SCR_TIME, SCR_NAVI, SCR_COMBO, SCR_GPS};			// Ekrany
enum NAVI_STATES	{NO_TARGET, REC_TRK, REC_WPTS, NAVI_WPTS};					// Stany nawigacji
enum BTN_MODES		{CHG_SCR, CHG_CTRL};										// Tryby dzialania przyciskow
enum TIMER_STATES	{TMR_STOP, TMR_RUN};										// Stany stopera

const char obrazek[] PROGMEM = "<img src='data:image/png;base64,iVBORw0KGgoAAAA ... KIB8b8B4VUyW9YaqDwAAAAASUVORK5CYII=' alt=''>";
bool connected;																	// Flaga WiFi
bool internet;																	// Flaga dostepu do Internetu
bool bit_state;																	// DEBUG
volatile bool pcf_signal;														// Flaga przerwania z expandera
volatile bool imp_signal;														// Flaga przerwania z impulsu
enum MUX_STATES mux_state;														// Stan multipleksera
enum SCREENS screen;															// Wyswietlany ekran
enum NAVI_STATES navi_state;													// Stan nawigacji
enum BTN_MODES btn_mode;														// Stan przelaczania ekrany/kontrolki
enum TIMER_STATES timer_state;													// Stan stopera
uint32_t current_time;
uint32_t pulses_cnt1, pulses_cnt2, gps_dist1, gps_dist2;

void welcomeScreen(void);
bool tftImgOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bmp);
void everySecTask(void);
void mux_switch(enum MUX_STATES state);
void i2c_irq(void);
void imp_irq(void);
void tft_cs(bool enabled);
void bootstrap(void);
void btnCheck(void);
void keyShortPress(enum BUTTONS button);
void keyLongPress(enum BUTTONS button);
void prevScr(void);
void nextScr(void);
void openScr(enum SCREENS screen);
void eeram_save16(int16_t addr, uint16_t val);
void eeram_save32(int16_t addr, uint32_t val);
void setupPins(void);															// Ustawienie pinow GPIO
void readConf(void);															// Odczyt konfiguracji urzadzenia
void startWebServer(void);														// Web server
void initWiFi(void);															// Inicjalizacja WiFi
void handleRoot(void);
void handleConf(void);
void handleLogin(void);
void handleLogin2(void);
void handleFWUpdate(void);
void handleFWUpdate2(void);
void handleFileUpload(void);
bool handleFileRead(String path);
String handleCalibration(void);
String SPIFFS_list(void);
void SD_list(void);
String getContentType(String filename);
String HTMLHeader(void);
String HTMLFooter(void);

/*

#define IMP_DELAY		10														// Minimalny odstep miedzy zewnetrznymi impulsami

#define NAVI_SAVE_TRK	0 														// Numer bitu - numer kontrolki
#define NAVI_SAVE_WPT	1
#define NAVI_TO_WPT		2

#define COMBO_CAL_DIST	0 														// Numer bitu - numer kontrolki
#define COMBO_CAL_VOLT	1
#define COMBO_CAL_TEMP	2

typedef struct
{
	uint16_t 	dist_cal;
	uint16_t 	volt_cal;
	uint16_t 	temp_cal;
} cal_t;

typedef struct
{
	int16_t		x;
	int16_t		y;
} point_t;

cal_t calibrations;
bool changed_volt_cal;
bool changed_temp_cal;
volatile uint32_t pulses_cnt1, pulses_cnt2, pulses_spd;							// Liczniki dystansow i predkosci (nie dla GPS!)
uint32_t gps_dist1, gps_dist2;													// Dystanse z GPS
uint32_t imp_dist1, imp_dist2, dist4speed;										// Dystanse z impulsatora
uint8_t ctl_pos[5];																// Tablica pozycji aktywnego przycisku na ekranie
float trt_mtx[3][3];															// Macierz (przesuniecie x obrot x przesuniecie)																				// Najstarszy bit ustawiony, jezeli kontrolka aktywna

void save_device_config(void);
void save_config_cb(void);
void prev_scr(void);
void next_scr(void);
void (*scr_close)(void);
void mux_switch(enum MUX_STATES state);
void read_distances(void);
void render_toolbar(enum TOOLBAR_ITEMS item);
void open_screen(enum SCREENS scr);
void key_long_press(enum BUTTONS btn);
void key_short_press(enum BUTTONS btn);
void save_time(void);
void btn_dist_cal(void);
void btn_rm_times(void);
void btn_save_trk(void);
void btn_save_wpt(void);
void btn_nav2wpt(void);
void volt_cal_up(void);
void volt_cal_dn(void);
void update_compass(uint16_t course);
void make_trt_mtx(int16_t x, int16_t y, float phi);
point_t mtx_mul_vec(float mtx[], point_t xy);
void update_speed(uint8_t speed);
void update_volt(uint8_t volt);
*/
#endif /* SDMOTO_H_ */