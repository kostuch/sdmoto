#include <pgmspace.h>

#ifndef SDMOTO_H_
#define SDMOTO_H_

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
#define EERAM_BASE		0x0000													// Pierwszy adres w EERAM
#define LAST_SCREEN		EERAM_BASE												// Ostatni ekran
#define BRIGHTNESS		(LAST_SCREEN + 1)										// Jasnosc ekranu
#define DIST1			(BRIGHTNESS + 2)										// Dystans odcinka
#define DIST2			(DIST1 + 4)												// Dystans calkowity (impulsy lub dystans)
#define LAST_FILE		(DIST2 + 4)												// Ostatnio uzywany plik z waypointami
#define LAST_WPT		(LAST_FILE)												// Ostatni waypoint z listy do ktorego byla nawigacja
#define DIST_CAL		(LAST_WPT + 2)											// Kalibracja dystansu
#define VOLT_CAL		(DIST_CAL + 2)											// Kalibracja napiecia
#define TEMP_CAL		(VOLT_CAL + 2)											// Kalibracja temperatury
#define BRIGHT_CAL		(TEMP_CAL + 2)											// Kalibracja jasnosci

// Koordynaty ikon na pasku
// 160x128
#define TBARX_GPS		0
#define TBARX_WIFI		32
#define TBARX_MEMORY	64
#define TBARX_SD		96
#define TBARX_TIME		128

#define MAX_SCREENS		9														// Ilosc ekranow aplikacji
#define LONG_PRESS		500														// Czas dlugiego wcisniecia [ms]
#define AP_TIMEOUT		10														// Czas na polaczenie z Access Pointem
#define SCR_SAVER_TIME	1														// Czas do wlaczenia screensavera
#define SAVE_TIME		5														// Co 5 sekund zapis dystansow do pamieci
#define MAX_SATS		20														// Maksymalna liczba widzianych satelitow
#define MAX_DOP			10														// Najgorsza precyzja [m]
#define TIMER_REFRESH	55														// Czas odswiezania stopera [ms]
#define COURSE_DIFF		3														// Minimalna zmiana kursu dla przerysowania kompasu
#define REC_WPT			5														// Ilosc sekund miedzy zapisem WPT do .gpx
#define WPT_NAME_LEN	11														// Dlugosc nazwy waypointa do nawigacji
#define RTE_NAME_LEN	11														// Dlugosc nazwy routy
#define TRK_NAME_LEN	11														// Dlugosc nazwy tracka
#define LIST_FILES_LEN	10														// Ilosc plikow mieszczacych sie na listingu
#define LIST_WPTS_LEN	10														// Ilosc waypointow mieszczaca sie na listingu

enum MUX_STATES		{STARTUP = 1, RUNTIME = 0};									// Stany multipleksera sygnalow
enum BUTTONS		{BTN_RELEASED = 0, BTN_RST = 7, BTN_UP = 5, BTN_DN = 6, BTN_LT = 4, BTN_RT = 3};
enum TOOLBAR_ITEMS	{WIFI_XOFF, WIFI_XSTA, WIFI_XAP, GPS_NOFIX, GPS_FIX,
                     GPS_DATETIME, MEM_FREE, MEM_AVG, MEM_FULL,
                     SD_OK, SD_NOOK, SD_OFF, REC_ON, REC_OFF
                	};															// Elementy statusbara
enum SCREENS		{SCR_DIST, SCR_TIME, SCR_NAVI, SCR_COMBO, SCR_GPS, SCR_UPDATE,
                     SCR_WELCOME, SCR_FILES, SCR_WPTS, SCR_GPXINFO
              		};															// Ekrany
enum NAVI_STATES	{NO_TARGET, REC_TRK, REC_WPTS, NAVI_WPTS};					// Stany nawigacji
enum BTN_MODES		{CHG_SCR, CHG_CTRL};										// Tryby dzialania przyciskow
enum TIMER_STATES	{TMR_STOP, TMR_RUN};										// Stany stopera
enum SDC_STATES		{SDC_OFF, SDC_NOOK, SDC_OK};								// Stany karty SD
typedef struct
{
	int		x;
	int		y;
} point_t;																		// Punkt na ekranie

typedef struct
{
	char		name[WPT_NAME_LEN];
	float		lat;
	float		lon;
} wpt_t;																		// Waypoint do nawigacji

typedef struct
{
	char		name[RTE_NAME_LEN];
	uint16_t 	size;
} rte_t;

typedef struct
{
	char		name[TRK_NAME_LEN];
	uint16_t 	size;
} trk_t;

typedef struct
{
	uint16_t 	dist_cal;
	uint16_t 	volt_cal;
	uint16_t 	temp_cal;
	uint16_t 	bright_cal;
} cal_t;																		// Zestaw kalibracji

typedef struct
{
	uint8_t		screen_id;
	uint8_t		key_id;
	uint8_t		x, y, w, h;
	char		*lbl_off;
	char		*lbl_on;
	void		(*key_exec)(bool on_off);
} btn_t;																		// Kontrolka ekranowa

typedef struct
{
	uint8_t screen_id;
	char *screen_name;
	void (*scr_open_exe)(void);
	void (*scr_close_exe)(void);
} screen_t;																		// Ekran na wyswietlaczu

typedef struct
{
	uint8_t		prn;
	bool		active;
	uint8_t		elevation;
	uint16_t	azimuth;
	uint8_t		snr;
} sat_t;																		// Parametry satelity

bool ssaver_active;																// Flaga uaktywnienia screensavera
bool connected;																	// Flaga WiFi
bool internet;																	// Flaga dostepu do Internetu
bool even_odd;																	// Parzysta/nieparzysta sekunda
bool fix;																		// FIX GPS
bool sat_stats_ready;															// Statystyki satelitow gotowe
bool counter_disable;															// Flaga pauzy metromierza
volatile bool pcf_signal;														// Flaga przerwania z expandera
volatile bool imp_signal;														// Flaga przerwania z impulsu
enum MUX_STATES mux_state;														// Stan multipleksera
enum SCREENS screen;															// Wyswietlany ekran
enum NAVI_STATES navi_state;													// Stan nawigacji
enum BTN_MODES btn_mode;														// Stan przelaczania ekrany/kontrolki
enum TIMER_STATES timer_state;													// Stan stopera
enum SDC_STATES sdc_state;														// Stan karty SD
volatile uint32_t pulses_cnt1, pulses_cnt2, pulses_spd;							// Liczniki impulsow
int fw_upd_progress;															// Postep upgrade
cal_t calibration;																// Kalibracje
uint32_t distance1, distance2;													// Dystanse (odcinka i globalny)
uint32_t cur_lat, cur_lon, old_lat, old_lon;									// Biezaca i poprzednia lokalizacja
uint8_t speed;																	// Predkosc
uint16_t volt;																	// Napiecie
uint32_t tmr_start_time;														// Czas stoperowy
uint8_t ctrl_state[MAX_SCREENS][2];												// #def ekranow, pozycja ramki, nr aktywnej kontrolki
uint32_t tmr_ms;																// Liczba milisekund stopera
uint32_t tmr_sec_num;															// Liczba sekund stopera
uint8_t tmr_hrs, tmr_mins, tmr_secs, tmr_frac;									// Stoper: godziny, minuty, sekundy, ulamki
sat_t sats_stats[MAX_SATS];														// Tabela parametrow satelitow
float trt_mtx[3][3];															// Macierz (przesuniecie x obrot x przesuniecie)
uint16_t course;																// Aktualny kurs wg gps
bool new_course;																// Flaga nowego kursu
char gpx_file[24];																// Plik gpx z nagrywanym sladem
bool trk_rec_flag;																// Flaga nagrywania pliku gpx
wpt_t dest_wpt;																	// Waypoint do nawigacji
uint8_t cur_file;																// Biezacy plik listingu
uint16_t cur_wpt;																// Biezacy waypoint w pliku
uint16_t file_wpts, file_rtes, file_trks;										// Licznik waypointow, routes i tracks w pliku gpx
uint16_t ssaver_time;															// Czas nieaktywnosci dla screensavera

void renderScreenSaver(void);													// Wygaszacz ekranu
void XML_callback(uint8_t statusflags, char *tagName, uint16_t tagNameLen, char *data, uint16_t dataLen);
void make_trt_mtx(point_t xy, float phi);										// Przygotowanie macierzy
point_t mtx_mul_vec(float *mtx, point_t xy);									// Mnozenie macierzy
void tftMsg(String message);													// Komunikat (o bledzie) na ekranie
bool tftImgOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bmp);
void everySecTask(void);														// Funkcja wykonywana co sekunde
void mux_switch(enum MUX_STATES state);											// Przelaczenie multipleksera sygnalow
void i2c_irq(void);																// Przerwanie od ekspandera
void imp_irq(void);																// Przerwanie od impulsu zewnetrznego
void tft_cs(bool enabled);														// Obsluga CS wyswietlacza TFT
void bootstrap(void);															// Spradzenie czy cos bylo nacisniete przy starcie
void btnCheck(void);															// Sprawdzenie aktywnego fizycznego przycisku
void keyShortPress(enum BUTTONS button);										// Krotkie nacisniecie fizycznego przycisku
void keyLongPress(enum BUTTONS button);											// Dlugie nacisniecie fizycznego przycisku
void prevScr(void);																// Poprzedni ekran
void nextScr(void);																// Nastepny ekran
void openScreen(enum SCREENS scr, bool remember);								// Otwarcie ekranu - stala zawartosc
void (*closeScr)(void);															// Callback zamkniecia ekranu
void (*btnExe)(bool on_off);													// Callback aktywacji kontrolki
void renderToolbar(enum TOOLBAR_ITEMS item);									// Odswiezenie _zawartosci_ toolbara
void renderScreen(enum SCREENS scr);											// Odswiezenie _zawartosci_ ekranu
uint16_t eeram_read16(int16_t addr);											// Odczyt dwubajtowej wartosci z EERAM
uint32_t eeram_read32(int16_t addr);											// Odczyt czterobajtowej wartosci z EERAM
void eeram_save16(int16_t addr, uint16_t val);									// Zapis dwubajtowej wartosci do EERAM
void eeram_save32(int16_t addr, uint32_t val);									// Zapis czterobajtowej wartosci do EERAM
void setupPins(void);															// Ustawienie pinow GPIO
void readConf(void);															// Odczyt konfiguracji urzadzenia
void startWebServer(void);														// Web server
void initWiFi(void);															// Inicjalizacja WiFi
void initWiFiStaAp(void);														// Wlaczenie Station albo AccesPoint
void handleRoot(void);															// www/
void handleConf(void);															// www/conf
void handleLogin(void);
void handleLogin2(void);
void handleFWUpdate(void);														// www/update
void handleFWUpdate2(void);														// www/update
void handleFileUpload(void);													// www/list
bool handleFileRead(String path);												// www/list
String handleCalibration(void);													// Generowanie strony /cal
String SPIFFS_list(void);														// Generowanie strony z listingiem plikow SPIFFS
void trySDCard(void);															// Test karty SD
void SD_list(void);
String getContentType(String filename);											// Obsluga typow MIME
String HTMLHeader(bool background);												// Generowanie naglowka HTML
String HTMLFooter(void);														// Generowanie stopki HTML
void computeDistance(void);														// Obliczanie przebytego dystansu
void computeSpeed(void);														// Obliczanie predkosci
void computeVolt(void);															// Obliczanie napiecia
void computeTime(void);															// Obliczanie czasu
uint8_t computeEraseArea(uint32_t new_val, uint32_t old_val, uint8_t length);	// Obliczanie obszaru do zamazania
void clearWindow(void);															// Czyszczenie dolnej czesci ekranu
void openDist(void);															// Otwarcie ekranu - metromierz
void openTime(void);															// Otwarcie ekranu - stoper
void openNavi(void);															// Otwarcie ekranu - nawigacja
void openCombo(void);															// Otwarcie ekranu - combo
void openGPS(void);																// Otwarcie ekranu - gps
void openWptFileList(void);														// Otwarcie ekranu - wybor pliku
void openWptList(void);															// Otwarcie ekranu - wybor waypointa
void openGpxInfo(void);															// Otwarcie ekranu - informacje o zawartosci gpx
void closeTime(void);															// Zamkniecie ekranu - stoper
void btnStopStart(bool on_off);													// Przycisk "start/stop" na metromierzu
void btnSaveTrk(bool on_off);													// Przycisk "zapisuj track"
void btnSaveWpt(bool on_off);													// Przycisk "zapis waypoint"
void btnNav2Wpt(bool on_off);													// Przycisk "nawiguj do waypointa"
void listWptFiles(uint8_t file_pos);											// Listowanie plikow .gpx
void parseGpxFile(void);														// Parsowanie pliku gpx
void btnCancelGpxInfo(bool on_off);												// Wyjscie z ekranu z informacjami o gpx
void btnPrevWptFile(bool on_off);												// Przycisk "nastepny plik z listy"
void btnNextWptFile(bool on_off);												// Przycisk "poprzedni plik z listy"
void btnInfoWptFile(bool on_off);												// Przycisk "informacje o zawartosci gpx"
void btnThisWptFile(bool on_off);												// Przycisk "OK" wybor pliku gpx
void btnCancelFile(bool on_off);												// Przycisk rezygnacji z wyboru pliku
void listWaypoints(uint16_t);													// Listowanie waypointow
void btnPrevWptSet(bool on_off);												// Poprzedni komplet waypointow do listowania
void btnNextWptSet(bool on_off);												// Nastepny komplet waypointow do listowania
void btnPrevWpt(bool on_off);													// Przycisk "poprzedni waypoint"
void btnNextWpt(bool on_off);													// Przycisk "nastepny waypoint"
void btnThisWpt(bool on_off);													// Przycisk "OK" wybor waypointa
void btnCancelWpt(bool on_off);													// Przycisk rezygnacji z wyboru waypointa
void renderCtrl(btn_t *ctrl);													// Rysuj kontrolke na ekranie
void meantimeSave(void);														// Zapis miedzyczas stopera
void satCustomInit(void);														// Inicjalizacja statystyk satelitow
void satUpdateStats(void);														// Aktualizacja statystyk satelitow
void renderCompassNeedle(uint16_t course, point_t xy, uint8_t r);				// Rysowanie igly kompasu
void renderWpt(uint16_t course, point_t xy, uint8_t shift, uint16_t color);		// Rysowanie symbolu waypointa na horyzoncie
void addWpt2Trk(void);															// Zapis WPT do trasy .gpx
void addWpt2Wpt(bool reset_num);												// Zapis WPT do zbioru waypointow .gpx

const char obrazek[] PROGMEM = "<img src='data:image/png;base64,iVBORw0KGgoAAAA ... KIB8b8B4VUyW9YaqDwAAAAASUVORK5CYII=' alt=''>";

const btn_t ctrls_data[] PROGMEM =
{
	{0, 0, 45, 110, 56, 12, (char *) "Zatrzymaj", (char *) "Uruchom", btnStopStart},
	{2, 0, 3, 36, 70, 12, (char *) "Zapis TRK", (char *) "Zapis TRK", btnSaveTrk},
	{2, 1, 3, 50, 70, 12, (char *) "Zapis WPT", (char *) "Zapis WPT", btnSaveWpt},
	{2, 2, 3, 64, 70, 12, (char *) "Navi do WPT", (char *) "Navi do WPT", btnNav2Wpt},
	{7, 0, 128, 36, 28, 12, (char *) "^", (char *) "", btnPrevWptFile},
	{7, 1, 128, 50, 28, 12, (char *) "v", (char *) "", btnNextWptFile},
	{7, 2, 128, 64, 28, 12, (char *) "Info", (char *) "", btnInfoWptFile},
	{7, 3, 128, 78, 28, 12, (char *) "OK", (char *) "", btnThisWptFile},
	{7, 4, 128, 92, 28, 12, (char *) "<==", (char *) "", btnCancelFile},
	{8, 0, 128, 36, 28, 12, (char *) "^", (char *) "", btnPrevWpt},
	{8, 1, 128, 50, 28, 12, (char *) "<<", (char *) "", btnPrevWptSet},
	{8, 2, 128, 64, 28, 12, (char *) ">>", (char *) "", btnNextWptSet},
	{8, 3, 128, 78, 28, 12, (char *) "v", (char *) "", btnNextWpt},
	{8, 4, 128, 92, 28, 12, (char *) "OK", (char *) "", btnThisWpt},
	{8, 5, 128, 106, 28, 12, (char *) "<==", (char *) "", btnCancelWpt},
	{9, 0, 128, 106, 28, 12, (char *) "<==", (char *) "", btnCancelGpxInfo}
};

const screen_t screen_data[] PROGMEM =
{
	{0, (char *) "Metromierz", openDist, NULL},									// Metromierze
	{1, (char *) "Stoper", openTime, closeTime},								// Stoper
	{2, (char *) "Nawigacja", openNavi, NULL},									// Nawigacja
	{3, (char *) "Wskazniki", openCombo, NULL},									// Wskazniki
	{4, (char *) "GPS", openGPS, NULL},											// GPS
	{5, (char *) "", NULL, NULL},												// Dummy (update)
	{6, (char *) "", NULL, NULL},												// Dummy (welcome)
	{7, (char *) "Pliki WPT", openWptFileList, NULL},							// Pliki gpx
	{8, (char *) "Waypointy", openWptList, NULL},								// Waypointy
	{9, (char *) "GPX Info", openGpxInfo, NULL}									// Informacja o gpx
};

#endif /* SDMOTO_H_ */