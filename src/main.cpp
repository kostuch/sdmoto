#include <Arduino.h>
#include <Wire.h>																// I2C
#include <SPI.h>																// SPI
#include <Ticker.h>																// Scheduler
#include <SdFat.h>																// SdFat (2.0 beta)
#include <FS.h>																	// SPIFFS
#include <pcf8574_esp.h>														// PCF8574/PCF8575
#include <ESP8266WiFi.h>														// WiFi
#include <ESP8266WiFiMulti.h>													// WiFi
#include <ESP8266WebServer.h>													// Web Server - konfiguracja
#include <WebConfig.h>															// Web Server - konfiguracja
#include <ESP8266HTTPClient.h>													// OTA http://www.skeletondevices.com
#include <ESP8266httpUpdate.h>													// OTA http://www.skeletondevices.com
#include "SimpleList.h"															// Obsluga list dynamicznych
#include <TJpg_Decoder.h>														// Dekoder jpg
#include <TFT_eSPI.h>															// TFT (ST7735)
#include <RemoteDebug.h>														// https://github.com/JoaoLopesF/RemoteDebug
#include <ArduinoOTA.h>
#include <SerialRAM.h>															// EERAM
#include <TinyXML.h>															// Parser plikow gpx
#include "TinyGPS++.h"															// GPS
#include "sdmoto.h"																// Konfiguracja kompilacji
#include "icons.h"																// Definicje ikon
#include "Gauge.h"																// Klasy wskaznikow

Ticker				every_sec_tmr;												// Timer sekundowy
Ticker 				timer_tmr;													// Timer stopera
Ticker 				one_shoot_tmr;												// Timer jednorazowy
RemoteDebug			Debug;														// Zdalny debug
PCF857x				pcf8575(I2C_EXP_A, &Wire);									// Ekspander PCF8574T
SerialRAM			eeram;														// EERAM
TFT_eSPI 			tft = TFT_eSPI();											// Wyswietlacz TFT
TinyGPSPlus			gps;														// GPS
TinyGPSCustom		totalGPGSVMessages(gps, "GPGSV", 1);						// $GPGSV sentence, pierwszy element
TinyGPSCustom		messageNumber(gps, "GPGSV", 2);								// $GPGSV sentence, drugi element
TinyGPSCustom		satsInView(gps, "GPGSV", 3);								// $GPGSV sentence, trzeci element
TinyGPSCustom		satNumber[4];												// Statystyki satelitow (cztery w kazdej sekwencji GPGSV)
TinyGPSCustom		elevation[4];
TinyGPSCustom		azimuth[4];
TinyGPSCustom		snr[4];
SdFat				sd;															// Karta SD
File				dir;														// Katalog na SD
File				file;														// Plik na SD
ESP8266WebServer	web_server;													// Web server
WebConfig			conf;														// Konfigurator webowy
fs::File			SPIFFS_file;												// Plik na SPIFFS
ESP8266WiFiMulti	wifi_multi;													// WiFi
WiFiClient			client;														// Klient wifi (dla OBD2)
WiFiEventHandler	SAPstationConnectedHandler;
WiFiEventHandler	SAPstationDisconnectedHandler;
WiFiEventHandler	STAstationGotIPHandler;
WiFiEventHandler	STAstationDisconnectedHandler;
WiFiEventHandler	wifiModeChanged;
SimpleList<btn_t>	ctrl_lst;													// Lista kontrolek (przyciskow) na ekranie
SimpleList<String>	wptfile_lst;												// Lista plikow z waypointami
SimpleList<wpt_t>	wpt_lst;
SimpleList<rte_t>	rte_lst;
SimpleList<trk_t>	trk_lst;
// Predkosciomierz do 150km/h
HGauge speed_gauge(&tft, 2, 74, 96, 16, TFT_BLACK, TFT_WHITE, TFT_RED, false, "", 1, G_PLAIN, 0, 150);
// Woltomierz 10-16V
HGauge volt_gauge(&tft, 2, 92, 96, 16, TFT_BLACK, TFT_WHITE, TFT_BLUE, false, "", 1, G_R2G, 10, 16);
// Termometr 0-100 stopni
HGauge temp_gauge(&tft, 2, 110, 96, 16, TFT_BLACK, TFT_WHITE, TFT_BLUE, false, "", 1, G_M_B2R, 0, 100);
// Widoczne satelity
HGauge satsv_gauge(&tft, 2, 34, 94, 16, TFT_BLACK, TFT_WHITE, TFT_GOLD, true, "", 1, G_PLAIN, 0, MAX_SATS);
// Uzywane do fixa satelity
HGauge satsu_gauge(&tft, 2, 50, 94, 16, TFT_BLACK, TFT_WHITE, TFT_GREEN, true, "", 1, G_PLAIN, 0, MAX_SATS);
// Precyzja lokalizacji
HGauge dop_gauge(&tft, 2, 66, 94, 16, TFT_BLACK, TFT_WHITE, TFT_CYAN, false, "HDOP", 1, G_M_R2G, 0, MAX_DOP - 1);

#define SD_CS_PIN 0 															// Fake value dla SdFat
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)							// Magia dla SdFat

void sdCsInit(SdCsPin_t pin) {}													// Inicjalizacja CS (Puste!)

void debugInit(void)
{
	// ------------------------- Remote debug
	Debug.begin(conf.getApName());												// Init socketa
	Debug.setResetCmdEnabled(true);												// Reset dozwolony
	Debug.showProfiler(true);													// Profiler (pomiar czsasu)
	Debug.showColors(true);														// Kolorki
}

void setup()
{
	Serial.begin(9600);															// Init UART (predkosc jak dla GPS)
	SPIFFS.begin();																// Init SPIFFS
	Wire.setClock(500000);														// Predkosc I2C (Max!!!)
	Wire.begin(SDA_PAD, SCL_PAD);												// Init I2C
	pcf8575.begin();															// Init PCF8574
	setupPins();																// Ustawienie pinow GPIO
	mux_switch(RUNTIME);														// Multiplekser w pozycji roboczej
	bootstrap();																// Test czy coś wcisniete przy uruchamianiu
	eeram.begin();																// Init EERAM
	eeram.setAutoStore(true);													// Automatyczne zapamietywanie RAM
	readConf();																	// Odczyt konfiguracji urzadzenia
	initWiFi();																	// Init WiFi (nieblokujacy)
	tft.init(NULL, tft_cs, NULL, NULL);											// Init TFT (DC, CS, RST, TCS, CS via driver)
	tft.setRotation(1);															// Landscape
	calibration.dist_cal = eeram_read16(DIST_CAL);								// Odczyt kalibracji dystansu (dla impulsatora)
	calibration.volt_cal = eeram_read16(VOLT_CAL);								// Odczyt kalibracji napiecia
	calibration.temp_cal = eeram_read16(TEMP_CAL);								// Odczyt kalibracji temperatury
	calibration.bright_cal = eeram_read16(BRIGHT_CAL);							// Odczyt kalibracji jasnosci
	analogWrite(BL_PIN, calibration.bright_cal);								// TFT podswietlenie wg kalibracji
	renderScreen(SCR_WELCOME);													// Pokaz wizytowke
	delay(3000);																// Chwila...
	tft.fillScreen(TFT_BLACK);													// CLS
	renderToolbar(WIFI_XOFF);													// Ikona WiFi
	renderToolbar(GPS_NOFIX);													// Ikona GPS
	SPIFFS_list();																// Ikona pamieci wg zajetosci
	trySDCard();																// Test karty SD
	renderToolbar(GPS_DATETIME);												// Czas
	pulses.imp_current = eeram_read32(DISTANCE1);								// Odczyt dystansu odcinka
	pulses.imp_total = eeram_read32(DISTANCE2);									// Odczyt dystansu globalnego
	pulses.gps_current = eeram_read32(PULSES1);
	pulses.gps_total = eeram_read32(PULSES2);
	pulses.imp_speed_avg = eeram_read32(IMP_SPEED_AVG);
	pulses.time_avg = eeram_read32(TIME_AVG);
	computeDistance();															// Oblicz dystans
	screen = (enum SCREENS) eeram.read(LAST_SCREEN);							// Ostatnio uzywany ekran
	openScreen(screen, true);													// Otworz go
	every_sec_tmr.attach_ms(1000, everySecTask);								// Zadania do wykonania co sekunde
	satCustomInit();															// Inicjalizacja statystyk satelitow
	setupOTA();																	// Ustawienie lokalnego OTA
}

void setupOTA()
{
	ArduinoOTA.onStart([]()
	{
		every_sec_tmr.detach();
		tft.setTextColor(TFT_RED, TFT_BLACK);
		tft.fillScreen(TFT_BLACK);
		tft.drawString(F("Start update"), 0, 0, 2);
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
	{
		tft.drawNumber(progress / (total / 100), 0, 16, 2);
	});
	ArduinoOTA.onEnd([]()
	{
		tft.drawString(F("End update"), 0, 32, 2);
	});
	ArduinoOTA.onError([](ota_error_t error)
	{
		tft.drawString(F("Error!!!"), 0, 64, 2);

		if (error == OTA_AUTH_ERROR) tft.drawString(F("Auth Failed"), 0, 80, 2);
		else if (error == OTA_BEGIN_ERROR) tft.drawString(F("Begin Failed"), 0, 80, 2);
		else if (error == OTA_CONNECT_ERROR) tft.drawString(F("Connect Failed"), 0, 80, 2);
		else if (error == OTA_RECEIVE_ERROR) tft.drawString(F("Receive Failed"), 0, 80, 2);
		else if (error == OTA_END_ERROR) tft.drawString(F("End Failed"), 0, 80, 2);
	});
	ArduinoOTA.begin();
}

void loop()
{
	static uint32_t h_second;

	if (pcf_signal) btnCheck();													// Okresl stan przyciskow

	if (imp_signal)
	{
		computeDistance();

		if (screen == SCR_DIST || screen == SCR_NAVI || screen == SCR_COMBO) renderScreen(screen);

		imp_signal = false;														// Przerwanie obsluzone
		ssaver_time = 0;														// Zerowanie licznika timeoutu
		//attachInterrupt(IMP_IRQ_PIN, imp_irq, FALLING);
	}

	if (!connected)																// Jezeli nie ma WiFi
	{
		if ((millis() - h_second) > 500)										// Co pol sekundy
		{
			initWiFiStaAp(true);												// Probuj STA, ewentualnie wlacz AP
			h_second = millis();												// Uaktualnij zmienna
		}
	}

	if (connected) web_server.handleClient();									// Obsluga Web serwera

	while (Serial.available()) gps.encode(Serial.read());						// Obsluga transmisji NMEA z GPS

	if (totalGPGSVMessages.isUpdated()) satUpdateStats();						// Update statystyk satelitow po otrzymaniu informacji o 4 satelitach

	ArduinoOTA.handle();

	if (connected) Debug.handle();												// Obsluga Remote Debug
}

void bootstrap()
{
	if (((pcf8575.read8() & 0xF8) ^ 0xF8) & (1 << BTN_RST))						// Jezeli wcisniety przycisk RST
	{
		Dir dir = SPIFFS.openDir("/");
		SPIFFS_file = SPIFFS.open("/firmware.bin", "r");						// Otworz plik z SPIFFS
		uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
		debugI("max free sketchspace = %d\r\n", maxSketchSpace);

		if (!Update.begin(maxSketchSpace, U_FLASH)) debugE("ERROR");			// Poczatek odtwarzania

		while (file.available())												// Do konca pliku
		{
			uint8_t ibuffer[128];
			file.read((uint8_t *)ibuffer, 128);
			Update.write(ibuffer, sizeof(ibuffer));
		}

		debugI("Koniec odtwarzania!");
		debugI("Wynik: %d", Update.end(true));
		file.close();
		mux_switch(STARTUP);													// Multiplekser w pozycji poczatkowej
		ESP.restart();															// Restart
	}
}

void sdCsWrite(SdCsPin_t pin, bool level)
{
	pcf8575.write(SDC_CS_PIN, level);											// CS dla SDCard
}

void tft_cs(bool enabled)
{
	pcf8575.write(TFT_CS_PIN, !enabled);										// CS dla TFT
}

void mux_switch(enum MUX_STATES state)
{
	pcf8575.write(MUX_PIN, state);
	mux_state = state;
}

void everySecTask()
{
	static uint8_t save_time;
	static uint8_t rec_wpt;
	static int16_t old_course;

	if (ssaver_time / 60 > SCR_SAVER_TIME)										// Po #def minutach aktywuj screensaver
	{
		ssaver_active = true;
		renderScreenSaver();
	}
	else ssaver_time++;															// Licznik czasu bezczynnosci

	if (!ssaver_active)															// Jezeli nieaktywny screensaver
	{
		even_odd ^= 1;															// Co sekunde zmien flage
		computeSpeed();															// Aktualizacja predkosci
		computeVolt();															// Aktualizacja napiecia
		computeTemp();															// Aktualizacja temperatury

		if (save_time++ > SAVE_TIME)											// Co pewien czas
		{
			eeram_save32(DISTANCE1, pulses.gps_current);						// Zapamietanie dystansow
			eeram_save32(DISTANCE2, pulses.gps_total);
			eeram_save32(PULSES1, pulses.imp_current);							// Zapamietanie impulsow
			eeram_save32(PULSES2, pulses.imp_total);
			save_time = 0;
		}

		renderToolbar(GPS_DATETIME);											// Nowy czas z GPS

		if (screen == SCR_COMBO) renderScreen(SCR_COMBO);

		if (screen == SCR_NAVI) renderScreen(SCR_NAVI);

		if (screen == SCR_GPS) renderScreen(SCR_GPS);
	}

	if (gps.location.isValid() && (gps.location.age() < 2000))					// Jezeli swieza lokalizacja
	{
		if (!fix)																// Jezeli go nie bylo
		{
			renderToolbar(GPS_FIX);												// Przerysuj ikone
			fix = true;															// Jest FIX
			old_lat = gps.location.lat();										// Inicjalizacja lokalizacji
			old_lon = gps.location.lng();
		}

		cur_lat = gps.location.lat();											// Biezaca lokalizacja
		cur_lon = gps.location.lng();
		//uint32_t dist = gps.distanceBetween(cur_lat, cur_lon, old_lat, old_lon);// Przebyty przez sekunde dystans
		gps_distance = gps.speed.knots() * KNOTS2MS / 1000;						// metry przebyte w ostatniej sekundzie
		pulses.gps_current += gps_distance;										// Kumulacja przebytego dystansu
		pulses.gps_total += gps_distance;
		old_lat = cur_lat;														// Zapamietaj lokalizacje jako stara
		old_lon = cur_lon;
		course = (int16_t) gps.course.deg();									// Nowy kurs

		if (gps_distance) ssaver_time = 0;										// Wyzeruj czas nieaktywnosci, jezeli zmiana

		if (navi_state == REC_TRK)												// Jezeli nagrywanie gpx
		{
			if (rec_wpt++ > REC_WPT)											// Co zdefiniowany czas
			{
				addWpt2Trk();													// Dodaj punkt do .gpx
				rec_wpt = 0;													// Wyzeruj timeout
			}
		}

		if (abs(course - old_course) > COURSE_DIFF)								// Jezeli kurs sie zmienil o co najmniej #def
		{
			new_course = true;													// Ustaw flage
			old_course = course;												// Zapamietaj ten kurs
		}
	}
	else
	{
		if (fix)																// Jezeli byl
		{
			renderToolbar(GPS_NOFIX);											// Przerysuj ikone
			fix = false;														// Nie ma FIXa
		}
	}

	if (ssaver_active && !(ssaver_time > SCR_SAVER_TIME))						// Jezeli aktywny screensaver i wyzerowane sekundy
	{
		ssaver_active = false;													// Wyzeruj flage
		tft.fillScreen(TFT_BLACK);

		if (fix) renderToolbar(GPS_FIX);										// Ikona GPS
		else renderToolbar(GPS_NOFIX);

		if (connected && internet) renderToolbar(WIFI_XSTA);					// Ikona wifi
		else renderToolbar(WIFI_XAP);

		SPIFFS_list();															// Ikona pamieci

		if (sdc_state == SDC_OFF) renderToolbar(SD_OFF);
		else if (sdc_state == SDC_NOOK) renderToolbar(SD_NOOK);
		else renderToolbar(SD_OK);												// Ikona karty SD

		openScreen(screen, true);												// Ostatni ekran
	}
}

void setupPins()
{
	// Konfiguracja przerwania dla impulsu zewnetrznego
	pinMode(IMP_IRQ_PIN, INPUT_PULLUP);
	attachInterrupt(IMP_IRQ_PIN, imp_irq, FALLING);
	// Konfiguracja przerwania dla zmiany na ekspanderze I2C
	pinMode(I2C_IRQ_PIN, INPUT_PULLUP);
	attachInterrupt(I2C_IRQ_PIN, i2c_irq, FALLING);
	pinMode(BL_PIN, OUTPUT);													// Sterowanie podswietleniem TFT
}

ICACHE_RAM_ATTR void imp_irq(void)
{
	if (!fix)																	// Jezeli brak sygnalu GPS
	{
		if (!counter_disable)													// Jezeli nie ma blokady metromierza
		{
			pulses.imp_current++;												// Zwieksz liczniki impulsow
			pulses.imp_total++;
			imp_signal = true;													// Flaga pojawienia sie impulsu
			//detachInterrupt(IMP_IRQ_PIN);										// Odepnij przerwania, bo inaczej CRASH
		}

		pulses_speed++;															// Impulsy do pomiaru predosci
	}
}

ICACHE_RAM_ATTR void i2c_irq(void)
{
	pcf_signal = true;															// Flaga zdarzenia na ekspanderze
}

void trySDCard()
{
	if (!sd.begin(SD_CONFIG))
	{
		renderToolbar(SD_OFF);													// Brak karty SD
		sdc_state = SDC_OFF;
	}
	else if (!dir.open("/"))
	{
		renderToolbar(SD_NOOK);													// Karta bez filesystemu
		sdc_state = SDC_NOOK;
	}
	else
	{
		renderToolbar(SD_OK);													// Karta OK
		sdc_state = SDC_OK;
	}
}

uint16_t eeram_read16(int16_t addr)
{
	uint16_t temp = 0;

	for (size_t i = 0; i < 2; i++) temp += (eeram.read(addr + i) << (i * 8));

	return temp;
}

uint32_t eeram_read32(int16_t addr)
{
	uint32_t temp = 0;

	for (size_t i = 0; i < 4; i++) temp += (eeram.read(addr + i) << (i * 8));

	return temp;
}

void eeram_save16(int16_t addr, uint16_t val)
{
	eeram.write(addr, val);														// LSB
	eeram.write(addr + 1, val >> 8);											// MSB
}

void eeram_save32(int16_t addr, uint32_t val)
{
	for (uint8_t a = 0; a < 4; a++)	eeram.write(addr + a, val >> (8 * a));		// Czterobajtowa zmienna (zapis od LSB)
}

void readConf()
{
	String params = F("["
	                  "{"
	                  "'name':'dev_pwd',"
	                  "'label':'Hasło urządzenia',"
	                  "'type':");
	params += String(INPUTPASSWORD);
	params += F(","
	            "'default':'SDMoto18'"
	            "},"
	            "{"
	            "'name':'ssid1',"
	            "'label':'Nazwa sieci WiFi',"
	            "'type':");
	params += String(INPUTTEXT);
	params += F(","
	            "'default':'SSID1'"
	            "},"
	            "{"
	            "'name':'pwd1',"
	            "'label':'Hasło WiFi',"
	            "'type':");
	params += String(INPUTPASSWORD);
	params += F(","
	            "'default':'moje_haslo_1'"
	            "},"
	            "{"
	            "'name':'ssid2',"
	            "'label':'Nazwa sieci WiFi',"
	            "'type':");
	params += String(INPUTTEXT);
	params += F(","
	            "'default':'SSID2'"
	            "},"
	            "{"
	            "'name':'pwd2',"
	            "'label':'Hasło WiFi',"
	            "'type':");
	params += String(INPUTPASSWORD);
	params += F(","
	            "'default':'moje_haslo_2'"
	            "},"
	            "{"
	            "'name':'obd2_net',"
	            "'label':'Sieć WiFi OBD2',"
	            "'type':");
	params += String(INPUTTEXT);
	params += F(","
	            "'default':'OBD2'"
	            "},"
	            /* 	            "{"
	            	            "'name':'imp_src',"
	            	            "'label':'Dane o dystansie',"
	            	            "'type':");
	            	params += String(INPUTRADIO);
	            	params += F(","
	            	            "'options':["
	            	            "{'v':'0','l':'GPS'},"
	            	            "{'v':'1','l':'Impulsator'}],"
	            	            "'default':'g'"
	            	            "},"
	             */	            "{"
	            "'name':'tz',"
	            "'label':'Strefa czasowa',"
	            "'type':");
	params += String(INPUTNUMBER);
	params += F(","
	            "'min':-12,'max':12,"
	            "'default':'2'"
	            "},"
	            "{"
	            "'name':'scr_dist',"
	            "'label':'Metromierz',"
	            "'type':");
	params += String(INPUTCHECKBOX);
	params += F(","
	            "'default':'1'"
	            "},"
	            "{"
	            "'name':'scr_time',"
	            "'label':'Stoper',"
	            "'type':");
	params += String(INPUTCHECKBOX);
	params += F(","
	            "'default':'1'"
	            "},"
	            "{"
	            "'name':'scr_navi',"
	            "'label':'Nawigacja',"
	            "'type':");
	params += String(INPUTCHECKBOX);
	params += F(","
	            "'default':'1'"
	            "},"
	            "{"
	            "'name':'scr_combo',"
	            "'label':'Wskaźniki',"
	            "'type':");
	params += String(INPUTCHECKBOX);
	params += F(","
	            "'default':'1'"
	            "},"
	            "{"
	            "'name':'scr_gps',"
	            "'label':'GPS',"
	            "'type':");
	params += String(INPUTCHECKBOX);
	params += F(","
	            "'default':'1'"
	            "},"
	            "{"
	            "'name':'scr_obd2',"
	            "'label':'OBD2',"
	            "'type':");
	params += String(INPUTCHECKBOX);
	params += F(","
	            "'default':'1'"
	            "}"
	            "]");
	conf.setDescription(params);
	conf.readConfig();
}

String getContentType(String filename)
{
	if (web_server.hasArg(F("download")))
		return F("application/octet-stream");
	else if (filename.endsWith(F(".htm")))
		return F("text/html");
	else if (filename.endsWith(F(".html")))
		return F("text/html");
	else if (filename.endsWith(F(".css")))
		return F("text/css");
	else if (filename.endsWith(F(".js")))
		return F("application/javascript");
	else if (filename.endsWith(F(".png")))
		return F("image/png");
	else if (filename.endsWith(F(".gif")))
		return F("image/gif");
	else if (filename.endsWith(F(".jpg")))
		return F("image/jpeg");
	else if (filename.endsWith(F(".ico")))
		return F("image/x-icon");
	else if (filename.endsWith(F(".xml")))
		return F("text/xml");
	else if (filename.endsWith(F(".pdf")))
		return F("application/x-pdf");
	else if (filename.endsWith(F(".zip")))
		return F("application/x-zip");
	else if (filename.endsWith(F(".gz")))
		return F("application/x-gzip");

	return F("text/plain");
}

bool handleFileRead(String path)
{
	if (path.endsWith("/"))	path += "index.htm";

	String contentType = getContentType(path);
	String pathWithGz = path + ".gz";

	if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path))
	{
		if (SPIFFS.exists(pathWithGz)) path += ".gz";

		fs::File file = SPIFFS.open(path, "r");
		web_server.streamFile(file, contentType);
		file.close();
		return true;
	}

	return false;
}

void handleFileUpload()
{
	if (web_server.uri() != "/list") return;

	HTTPUpload &upload = web_server.upload();

	if (!upload.filename.length()) return;

	String filename = upload.filename;

	if (upload.status == UPLOAD_FILE_START)
	{
		if (!filename.startsWith("/")) filename = "/" + filename;

		SPIFFS_file = SPIFFS.open(filename, "w");
		filename = String();
	}
	else if (upload.status == UPLOAD_FILE_WRITE)
	{
		if (SPIFFS_file) SPIFFS_file.write(upload.buf, upload.currentSize);
	}
	else if (upload.status == UPLOAD_FILE_END)
	{
		if (SPIFFS_file) SPIFFS_file.close();

		//debugI("Zaladowany plik %s o rozmiarze %d", filename.c_str(), upload.totalSize);
	}
}

String SPIFFS_list()
{
	fs::FSInfo fs_info;
	SPIFFS.info(fs_info);
	int32_t free_space = (fs_info.totalBytes - fs_info.usedBytes) / 1024;		// Kilobajty
	uint8_t spiffs_usage = (fs_info.usedBytes * 100) / fs_info.totalBytes;		// Procent zajetosci SPIFFS

	if (spiffs_usage < 50)	renderToolbar(MEM_FREE);							// Ikona pamieci
	else if (spiffs_usage < 80) renderToolbar(MEM_AVG);
	else renderToolbar(MEM_FULL);

	fs::Dir dir = SPIFFS.openDir("/");
	String page = HTMLHeader(false);
	page += F("<h3>Dostępne w pamięci pliki</h3>\n");

	while (dir.next())
	{
		String FileName = dir.fileName();
		SPIFFS_file = dir.openFile("r");
		String FileSize = String(SPIFFS_file.size());
		page += F("<form method='POST' action='/list'>\n");
		page += F("<p style='text-align: left; font-family: \"Courier New\", Courier, monospace'><a href='");
		page += FileName + F("'><strong>");
		page += FileName;
		int whsp = 20 - FileName.length();

		while (whsp-- > 0) page += F("&nbsp;");

		page += F(":");
		whsp = 6 - FileSize.length();

		while (whsp-- > 0)
		{
			//FileList += " ";
			page += F("&nbsp;");
		}

		page += FileSize + F("&nbsp;&nbsp;&nbsp;</strong></a>");
		page += F("<button type='submit' name='DELETE' value='");
		page += FileName + F("' class='btn-danger'>Usuń</button></form>\n");;
		//FileList += FileSize + " " + FileName + "\n";
	}

	page += F("<p class='text-left'>Wolne miejsce w pamięci: ");
	page += free_space;
	page += F("kB</p>"
	          "<hr>"
	          "<p>Zapis plików do urządzenia</p>"
	          "<form method='POST' action='/list' "
	          "enctype='multipart/form-data'><input type='file' "
	          "name='NEW_FILE' class='btn-default active'><input type='submit'"
	          " value='Zapisz plik' class='btn-success'></form>");
	page += HTMLFooter();

	if (connected) web_server.send(200, F("text/html"), page);

	return page;
}

String macToString(const unsigned char *mac)
{
	char buf[20];
	snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
	         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	return String(buf);
}

void initWiFi()
{
	WiFi.persistent(false);														// Nie zapisuj konfiga WiFi we FLASH
	WiFi.mode(WIFI_STA);														// Tryb STATION
	wifi_multi.addAP(conf.getValue("ssid1"), conf.getValue("pwd1"));			// Dodaj pierwsza siec
	wifi_multi.addAP(conf.getValue("ssid2"), conf.getValue("pwd2"));			// Dodaj druga siec
}

void initWiFiStaAp(bool fallback2AP)
{
	static uint8_t ap_time;

	if (wifi_multi.run() != WL_CONNECTED)
	{
		if (ap_time++ > AP_TIMEOUT)
		{
			if (fallback2AP)													// Jezeli jest opcja stworzenia AP
			{
				IPAddress local_IP(10, 0, 0, 1);
				IPAddress gateway(10, 0, 0, 1);
				IPAddress subnet(255, 0, 0, 0);
				WiFi.softAPConfig(local_IP, gateway, subnet);
				WiFi.softAP(conf.getApName(), conf.getValue("dev_pwd"));		// Ustaw tryb Access Pointa
				connected = true;												// Ustaw flage
				internet = false;
				renderToolbar(WIFI_XAP);										// Aktualizuj ikone na toolbarze
				startWebServer();												// Wystartuj Serwer www
				debugInit();
				ap_time = 0;													// Wyzeruj timeout na potrzeby reconnecta
			}
		}
	}
	else
	{
		connected = true;														// Flaga polaczenia
		internet = true;														// Flaga dostepu do internetu
		renderToolbar(WIFI_XSTA);												// Aktualizuj ikone na pasku
		WiFi.hostname(conf.getApName());										// Nazwa hosta (kosmetyka)
		startWebServer();														// Wystartuj Serwer www
		debugInit();
		ap_time = 0;															// Wyzeruj timeout na potrzeby reconnecta
	}
}

void update_started(void)
{
	web_server.sendHeader(F("Connection"), F("close"));
	web_server.sendHeader(F("Access-Control-Allow-Origin"), F("*"));
	web_server.send(200, F("text/plain"), F("Czekaj na zakonczenie update..."));
	mux_switch(STARTUP);														// Wylaczenie sygnalu przerwan
	every_sec_tmr.detach();														// Zatrzymaj timer sekundowy
}

void update_finished(void)
{
	tft.setCursor(2, 64, 1);
	tft.setTextSize(2);
	tft.setTextColor(TFT_BLACK);
	tft.printf_P(PSTR("Restartuje..."));
}

void update_progress(int cur, int total)
{
	fw_upd_progress = (cur * 100) / total;										// Procent postepu aktualizacji
	renderScreen(SCR_UPDATE);
}

void update_error(int err)
{
	Serial.printf_P(PSTR("HTTP update fatal error code %d\n"), err);
	fw_upd_progress = err;
	//renderScreen(SCR_UPDATE);
}

void handleFWUpdate()
{
	Serial.println(F("Checking Firmware..."));
	WiFiClient wifi_client;
	HTTPClient http;
	http.begin(wifi_client, F("http://www.skeletondevices.com/free_files/sdmoto18_versions.txt"));
	int httpCode = http.GET();

	if (httpCode == 200)
	{
		String new_fw = http.getString();
		Serial.printf_P(PSTR("Available versions:\r\n%s\r\n"), new_fw.c_str());
		SimpleList<uint32_t> fw_list;
		uint8_t	r = 0;
		String page = HTMLHeader(true);
		page += F("<h1><strong>Dostępne wersje oprogramowania</strong></h1>\n"
		          "<form action='/update_ver' method='POST'>\n"
		          "<div>\n"
		          "<select name='wersja' "
		          "style='color: #FF0000'>");

		for (uint16_t i = 0; i < new_fw.length(); i++)
		{
			if (new_fw.charAt(i) == '\n')
			{
				page += F("<option value='");
				page += new_fw.substring(r, i - 1);
				page += F("'>");
				page += new_fw.substring(r, i - 1);
				page += F("</option>\n");
				fw_list.push_back(new_fw.substring(r, i - 1).toInt());
				r = (i + 1);
			}
		}

		page += F("</select></div>\n"
		          "<p></p>"
		          "<div><button type='button submit' name='SAVE' value='1' "
		          "class='btn btn-success btn-lg'>Aktualizuj</button></div></form>\n");
		page += HTMLFooter();
		web_server.send(200, F("text/html"), page);
	}
}

void handleFWUpdate2()
{
	WiFiClient wifi_client;
	Serial.printf_P(PSTR("Obecna wersja: %d\r\n"), FW_VERSION);
	Serial.printf_P(PSTR("Bedzie upgrade do wersji %s\r\n"), web_server.arg("wersja").c_str());
	// Add optional callback notifiers
	ESPhttpUpdate.onStart(update_started);
	ESPhttpUpdate.onEnd(update_finished);
	ESPhttpUpdate.onProgress(update_progress);
	ESPhttpUpdate.onError(update_error);
	String new_fw = web_server.arg("wersja");
	//String new_file = "http://www.skeletondevices.com/free_files/sdmoto18_" + new_fw + ".bin";
	String new_file = F("http://www.skeletondevices.com/free_files/firmware.bin");
	Serial.print(F("Starting update with: ")); Serial.println(new_file);
	t_httpUpdate_return ret = ESPhttpUpdate.update(wifi_client, new_file);

	switch (ret)
	{
		case HTTP_UPDATE_FAILED:
			Serial.printf_P(PSTR("HTTP_UPDATE_FAILED Error (%d): %s"), ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
			web_server.sendHeader(F("Connection"), F("close"));
			web_server.sendHeader(F("Access-Control-Allow-Origin"), F("*"));
			web_server.send(200, F("text/plain"), F("Coś nie bangla...\nNie ma takiego pliku."));
			break;

		case HTTP_UPDATE_NO_UPDATES:
			Serial.println("HTTP_UPDATE_NO_UPDATES");
			web_server.sendHeader(F("Connection"), F("close"));
			web_server.sendHeader(F("Access-Control-Allow-Origin"), F("*"));
			web_server.send(200, F("text/plain"), F("Nie wiem o co kaman..."));
			break;

		case HTTP_UPDATE_OK:
			break;
	}
}

void handleRoot()
{
	String page = HTMLHeader(true);
	//for (size_t i = 0; i < sizeof(obrazek); i++) page += char(pgm_read_byte(obrazek + i));
	page += F("<img src='/top.jpg' alt=''>");
	page += F("<h3>Wersja software: ");
	page += FW_VERSION;
	page += F("</h3>");
	page += F("<p><a href='/conf'><strong>Konfiguracja</strong></a></p>"
	          "<p><a href='/cal'><strong>Kalibracja</strong></a></p>"
	          "<p><a href='/help'><strong>Pomoc</strong></a></p>"
	          "<p><a href='/list'><strong>Pliki w pamięci</strong></a></p>"
	          "<p><a href='/login'><strong>Login test</strong></a></p>"
	          "<hr />"
	          "<p><a href='/update'><strong>FW Update</strong></a></p>"
	          "<hr />");
	page += HTMLFooter();
	web_server.send(200, F("text/html"), page);
}

void handleConf()
{
	conf.handleFormRequest(&web_server);										// Konfiguracja przez www
}

void handleLogin()
{
	web_server.send(200, F("text/html"), F("<form action='/login_check'"
	                                       " method='POST'><input type='text' name='username' placeholder='Username'></br>"
	                                       "<input type='password' name='password' placeholder='Password'></br>"
	                                       "<input type='submit' value='Login'></form><p>Try 'John Doe' and 'password123' ...</p>"));
}

void handleLogin2()
{
	if ( ! web_server.hasArg("username") || ! web_server.hasArg("password") ||
	        web_server.arg("username") == NULL || web_server.arg("password") == NULL)   // If the POST request doesn't have username and password data
	{
		web_server.send(400, F("text/plain"), F("400: Invalid Request"));         // The request is invalid, so send HTTP status 400
		return;
	}

	if (web_server.arg("username") == F("John Doe") && web_server.arg("password") == F("password123"))  // If both the username and the password are correct
		web_server.send(200, "text/html", "<h1>Welcome, " + web_server.arg("username") + "!</h1><p>Login successful</p>");
	else web_server.send(401, F("text/plain"), F("401: Unauthorized"));               // Username and password don't match
}

void startWebServer()
{
	web_server.onFileUpload(handleFileUpload);
	web_server.on(F("/"), handleRoot);
	web_server.on(F("/update"), handleFWUpdate);
	web_server.on(F("/update_ver"), HTTP_POST, handleFWUpdate2);
	web_server.on(F("/conf"), handleConf);
	web_server.on(F("/cal"), handleCalibration);
	web_server.on(F("/cal/cal_dist_plus10"), []()
	{
		calibration.dist_cal += 10;
		web_server.sendHeader("Location", "/cal", true);
		web_server.send (302, "text/plain", "");
		//web_server.send(200, F("text/html"), handleCalibration());
	});
	web_server.on(F("/cal/cal_dist_plus"), []()
	{
		calibration.dist_cal++;
		web_server.sendHeader("Location", "/cal", true);
		web_server.send (302, "text/plain", "");
	});
	web_server.on(F("/cal/cal_dist_minus10"), []()
	{
		if (calibration.dist_cal > 10) calibration.dist_cal -= 10;

		web_server.sendHeader("Location", "/cal", true);
		web_server.send (302, "text/plain", "");
	});
	web_server.on(F("/cal/cal_dist_minus"), []()
	{
		if (calibration.dist_cal > 1) calibration.dist_cal--;

		web_server.sendHeader("Location", "/cal", true);
		web_server.send (302, "text/plain", "");
		//web_server.send(200, F("text/html"), handleCalibration());
	});
	web_server.on(F("/cal/cal_volt_plus"), []()
	{
		calibration.volt_cal++;
		web_server.sendHeader("Location", "/cal", true);
		web_server.send (302, "text/plain", "");
	});
	web_server.on(F("/cal/cal_volt_minus"), []()
	{
		if (calibration.volt_cal > 1) calibration.volt_cal--;

		web_server.sendHeader("Location", "/cal", true);
		web_server.send (302, "text/plain", "");
	});
	web_server.on(F("/cal/cal_temp_plus"), []()
	{
		calibration.temp_cal++;
		web_server.sendHeader("Location", "/cal", true);
		web_server.send (302, "text/plain", "");
	});
	web_server.on(F("/cal/cal_temp_minus"), []()
	{
		if (calibration.temp_cal > 1) calibration.temp_cal--;

		web_server.sendHeader("Location", "/cal", true);
		web_server.send (302, "text/plain", "");
	});
	web_server.on(F("/cal/cal_bright_plus"), []()
	{
		if (calibration.bright_cal > 200) calibration.bright_cal -= 200;		// Jasniej

		analogWrite(BL_PIN, calibration.bright_cal);
		web_server.sendHeader("Location", "/cal", true);
		web_server.send (302, "text/plain", "");
	});
	web_server.on(F("/cal/cal_bright_minus"), []()
	{
		if (calibration.bright_cal < 800) calibration.bright_cal += 200;		// Ciemniej

		analogWrite(BL_PIN, calibration.bright_cal);
		web_server.sendHeader("Location", "/cal", true);
		web_server.send (302, "text/plain", "");
	});
	web_server.on(F("/login"), HTTP_GET, handleLogin);
	web_server.on(F("/login_check"), HTTP_POST, handleLogin2);
	web_server.on(F("/list"), HTTP_GET, SPIFFS_list);
	web_server.on(F("/list"), HTTP_POST, []()
	{
		if (web_server.hasArg("DELETE")) SPIFFS.remove(web_server.arg("DELETE"));

		web_server.send(200, F("text/html"), SPIFFS_list());
	});
	web_server.on(F("/all"), HTTP_GET, []()
	{
		String json = F("{'heap':");
		json += String(ESP.getFreeHeap());
		json += F(", 'analog':");
		json += String(analogRead(A0));
		json += F(", 'gpio':");
		json += String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
		json += F("}");
		web_server.send(200, F("text/json"), json);
		json = String();
	});
	web_server.onNotFound([]()
	{
		if (!handleFileRead(web_server.uri())) //web_server.send(404, F("text/plain"), F("FileNotFound"));
		{
			String message = F("File Not Found\n\n");
			message += F("URI: ");
			message += web_server.uri();
			message += F("\nMethod: ");
			message += (web_server.method() == HTTP_GET) ? F("GET") : F("POST");
			message += F("\nArguments: ");
			message += web_server.args();
			message += F("\n");

			for (uint8_t i = 0; i < web_server.args(); i++)
				message += " NAME:" + web_server.argName(i) + "\n VALUE:" + web_server.arg(i) + "\n";

			web_server.send(404, F("text/plain"), message);
			Serial.println(message);
		}
	});
	web_server.begin(80);
}

String handleCalibration()
{
	if (web_server.hasArg("SAVE"))												// Po kliknieciu buttona "Zapisz"
	{
		eeram_save16(DIST_CAL, calibration.dist_cal);
		eeram_save16(VOLT_CAL, calibration.volt_cal);
		eeram_save16(TEMP_CAL, calibration.temp_cal);
		eeram_save16(BRIGHT_CAL, calibration.bright_cal);
	}

	String page = HTMLHeader(false);
	page += F("<h1>Kalibracja</h1>\n");
	//Image to DataURL converter https://onlinecamscanner.com/image-to-dataurl
	//for (size_t i = 0; i < sizeof(obrazek); i++) page += char(pgm_read_byte(obrazek + i));
	//p += "<img src='/top.jpg' alt=''>";
	page += F("<table style='border: medium solid #FFFFFF; width:100%; height: 28px;'>\n"
	          "<tr>\n"
	          "<th class='text-center' colspan='2'>Dystans</th>\n"
	          "<th class='text-center'>Napięcie</th>\n"
	          "<th class='text-center'>Temperatura</th>\n"
	          "<th class='text-center'>Jasność</th>\n"
	          "</tr>\n"
	          "<tr>\n"
	          "<td style='width: 20%'><a href = '/cal/cal_dist_plus10'><button class='btn btn-primary btn-lg'>CAL +10</button></a></td>\n"
	          "<td><a href = '/cal/cal_dist_plus'><button class='btn btn-primary btn-lg'>CAL +</button></a></td>\n"
	          "<td><a href = '/cal/cal_volt_plus'><button class='btn btn-primary btn-lg'>CAL +</button></a></td>\n"
	          "<td><a href = '/cal/cal_temp_plus'><button class='btn btn-primary btn-lg'>CAL +</button></a></td>\n"
	          "<td><a href = '/cal/cal_bright_plus'><button class='btn btn-primary btn-lg'>CAL +</button></a></td>\n"
	          "</tr>\n"
	          "<tr>\n"
	          "<td style='width: 20%'><a href = '/cal/cal_dist_minus10'><button class='btn btn-secondary btn-lg'>CAL -10</button></a></td>\n"
	          "<td><a href = '/cal/cal_dist_minus'><button class='btn btn-secondary btn-lg'>CAL -</button></a></td>\n"
	          "<td><a href = '/cal/cal_volt_minus'><button class='btn btn-secondary btn-lg'>CAL -</button></a></td>\n"
	          "<td><a href = '/cal/cal_temp_minus'><button class='btn btn-secondary btn-lg'>CAL -</button></a></td>\n"
	          "<td><a href = '/cal/cal_bright_minus'><button class='btn btn-secondary btn-lg'>CAL -</button></a></td>\n"
	          "</tr>\n"
	          "<tr>\n"
	          "<td colspan='2'><h3>");
	page += String(calibration.dist_cal) + F("</h3>");
	page += F("<td><h3>");
	page += String(calibration.volt_cal) + F("</h3>");
	page += F("<td><h3>");
	page += String(calibration.temp_cal) + F("</h3>");
	page += F("<td><h3>");
	page += String(calibration.bright_cal / 200) + F("</h3>");
	page += F("</tr>\n"
	          "</table> \n");
	page += F("<p><form action='/cal' method='POST'><button type='button submit' name='SAVE' value='1' "
	          "class='btn btn-success btn-lg'>Zapis</button></form></p>\n");
	page += HTMLFooter();
	web_server.send(200, F("text/html"), page);
	return page;
}

String HTMLHeader(bool background)
{
	String h = F("<!DOCTYPE html>\n"
	             "<html>\n"
	             "<head>\n"
	             //"<link href='/favicon.ico' rel='icon' type='image/x-icon' />"
	             "<title>Centrum kontroli nad wszechświatem</title>\n"
	             "<meta charset='utf-8'>\n"
	             "<meta name='viewport' content='width=device-width, initial-scale=1'>\n"
	             "<link rel='stylesheet' href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.4/css/bootstrap.min.css' >\n"
	             "</head>\n");

	if (background)
		h += F("<body style='text-align: center;color: white; background: black;font-size: 1.5em; "
		       "background-size: 100% 100vh; background-image: url(/logo_content.jpg)'>\n");
	else
		h += F("<body style='text-align: center;color: white; background: black;font-size: 1.5em;'>\n");

	return h;
}

String HTMLFooter()
{
	String f = F("<p>SkeletonDevices &copy; 2020</p>"
	             "</body>\n"
	             "</html>\n");
	return f;
}

bool tftImgOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap)
{
	// Stop further decoding as image is running off bottom of screen
	if ( y >= tft.height() ) return 0;

	// This function will clip the image block rendering automatically at the TFT boundaries
	// This might work instead if you adapt the sketch to use the Adafruit_GFX library
	tft.pushImage(x, y, w, h, bitmap);
	// tft.drawRGBBitmap(x, y, bitmap, w, h);
	// Return 1 to decode next block
	return 1;
}

void SD_list()
{
	if (!dir.open("/"))	Serial.println("dir.open failed");

	while (file.openNext(&dir, O_RDONLY))
	{
		file.printFileSize(&Serial);
		Serial.write(' ');
		file.printModifyDateTime(&Serial);
		Serial.write(' ');
		file.printName(&Serial);

		if (file.isDir()) Serial.write('/');

		Serial.println();
		file.close();
	}

	if (dir.getError())	Serial.println("openNext failed");
	else Serial.println("Done!");
}

void btnCheck()
{
	static uint32_t key_time;													// Czas wcisniecia
	static uint8_t button_num, prev_button;
	uint8_t i2c_state = (pcf8575.read8() & 0xF8) ^ 0xF8;						// 5 najstarszych bitow

	for (size_t bin_num = 3; bin_num < 8; bin_num++)							// Bity 3-7
	{
		if (i2c_state & (1 << bin_num))											// Jezeli bit ustawiony
		{
			button_num = bin_num;												// Numer bitu przycisku
			prev_button = button_num;											// Zapamietaj jaki to byl przycisk
			break;																// Przerwij
		}

		button_num = BTN_RELEASED;
	}

	switch (button_num)
	{
		case BTN_RST:
			key_time = millis();
			break;

		case BTN_UP:
		case BTN_RT:
			key_time = millis();
			break;

		case BTN_DN:
		case BTN_LT:
			key_time = millis();
			break;

		case BTN_RELEASED:
			if ((millis() - key_time) < LONG_PRESS) keyShortPress((enum BUTTONS) prev_button);	// Obsluz przycisk
			else keyLongPress((enum BUTTONS) prev_button);						// Obsluz przycisk

			break;
	}

	ssaver_time = 0;															// Zerowanie czasu nieaktywnosci
	pcf_signal = false;															// Przycisk obsluzony
}

void keyShortPress(enum BUTTONS button)
{
	switch (button)
	{
		case BTN_RST:
			if (btn_mode == CHG_SCR)
			{
				switch (screen)
				{
					case SCR_DIST:
						pulses.imp_current = 0;									// Skasuj dystans odcinka
						computeDistance();										// Przelicz dystanse
						renderScreen(screen);									// Aktualizuj ekran
						break;

					case SCR_TIME:
						if (timer_state == TMR_STOP)
						{
							timer_state = TMR_RUN;								// Uruchom stoper
							timer_tmr.attach_ms(TIMER_REFRESH, std::bind(renderScreen, screen));
							tmr_start_time = millis();							// Wartosc poczatkowa
						}
						else
						{
							timer_tmr.detach();
							meantimeSave();										// Zapamietaj miedzyczas
							tmr_start_time = millis();							// Wartosc poczatkowa - wyzerowanie stopera
							renderScreen(SCR_TIME);								// Pokaz wyzerowany stoper
							timer_state = TMR_STOP;								// Zatrzymaj stoper
						}

						break;

					case SCR_NAVI:
						pulses.imp_current = 0;									// Skasuj dystans odcinka
						computeDistance();										// Przelicz dystanse

						if (navi_state == REC_WPTS)	addWpt2Wpt(false);			// Dodaj kolejny waypoint do pliku gpx, jezeli aktywny zapis

						renderScreen(screen);									// Aktualizuj ekran
						break;

					case SCR_COMBO:
						pulses.imp_current = 0;									// Skasuj dystans odcinka
						computeDistance();
						renderScreen(screen);									// Aktualizuj ekran
						break;

					default:
						break;
				}
			}
			else
			{
				SimpleList<btn_t>::iterator idx = ctrl_lst.begin();

				if (!(ctrl_state[screen][1] & 0x80))							// Jezeli (zadna) kontrolka nie jest aktywna (nieustawione MSB)
				{
					ctrl_state[screen][1] = ctrl_state[screen][0];				// Zapisz biezaca kontrolke jako aktywna
					CTRL_ON;													// Ustaw MSB jako znacznik aktywowanej kontrolki
					renderCtrl(idx + ctrl_state[screen][0]);
					btnExe = (idx + ctrl_state[screen][0])->key_exec;			// exe w zaleznosci od kontrolki

					if (btnExe) btnExe(true);									// Wykonaj
				}
				else
				{
					// Jezeli ustawiony MSB i nr biezacej kontrolki taki jak nr aktywnej kontrolki
					if ((ctrl_state[screen][1] & 0x80) && ((ctrl_state[screen][1] & 0x7F) == ctrl_state[screen][0]))
					{
						CTRL_OFF;												// Skasuj MSB jako znacznik deaktywowanej kontrolki
						renderCtrl(idx + ctrl_state[screen][0]);
						btnExe = (idx + ctrl_state[screen][0])->key_exec;		// exe w zaleznosci od kontrolki

						if (btnExe) btnExe(false);								// Wykonaj
					}
				}
			}

			break;

		case BTN_UP:
		case BTN_RT:
			if (btn_mode == CHG_SCR) nextScr();									// Nastepny ekran
			else																// Nastepna kontrolka
			{
				SimpleList<btn_t>::iterator idx = ctrl_lst.begin();

				if (ctrl_state[screen][0] > 0)									// Jezeli jeszcze jakies dostepne kontrolki
				{
					ctrl_state[screen][0]--;			 						// Poprzednia
					renderCtrl((idx + ctrl_state[screen][0]) + 1);				// Usun focus ze starej
				}
				else
				{
					ctrl_state[screen][0] = ctrl_lst.size() - 1;				// Albo od konca
					renderCtrl(idx);											// Usun focus z pierwszej
				}

				renderCtrl(idx + ctrl_state[screen][0]);
			}

			break;

		case BTN_DN:
		case BTN_LT:
			if (btn_mode == CHG_SCR) prevScr();									// Poprzedni ekran
			else 																// Poprzednia kontrolka
			{
				SimpleList<btn_t>::iterator idx = ctrl_lst.begin();

				if (ctrl_state[screen][0] < (ctrl_lst.size() - 1))				// Jezeli jeszcze jakies dostepne kontrolki
				{
					ctrl_state[screen][0]++;		 							// Nastepna
					renderCtrl((idx + ctrl_state[screen][0]) - 1);				// Usun focus ze starej
				}
				else
				{
					ctrl_state[screen][0] = 0;									// Albo od poczatku
					renderCtrl(idx + ctrl_lst.size() - 1);						// Usun focus z ostatniej
				}

				renderCtrl(idx + ctrl_state[screen][0]);
			}

			break;

		default:
			break;
	}
}

void keyLongPress(enum BUTTONS button)
{
	switch (button)
	{
		case BTN_RST:
			switch (screen)
			{
				case SCR_DIST:
				case SCR_NAVI:
				case SCR_COMBO:
					if (btn_mode == CHG_SCR)									// W trybie zmiany ekranow
					{
						pulses.imp_total = 0;									// Skasuj dystans globalny
						computeDistance();
						renderScreen(screen);									// Aktualizuj ekran
					}

					break;

				case SCR_TIME:
					if (timer_state == TMR_RUN)	meantimeSave();					// Zapamietaj miedzyczas

					break;

				default:
					break;
			}

			break;

		case BTN_UP:
		case BTN_RT:
		case BTN_DN:
		case BTN_LT:
			if (ctrl_lst.size())												// Jezeli dla ekranu sa jakiekolwiek przyciski
			{
				SimpleList<btn_t>::iterator idx = ctrl_lst.begin();

				if (btn_mode == CHG_SCR)
				{
					btn_mode = CHG_CTRL;										// Zmien tryb zmiany ekranu/kontrolki
					renderCtrl(idx + ctrl_state[screen][0]);
					tft.drawRect(0, 32, 160, 96, TFT_BLACK);					// Skasuj ramke okna
				}
				else
				{
					btn_mode = CHG_SCR;											// Zmien tryb zmiany ekranu/kontrolki
					renderCtrl(idx + ctrl_state[screen][0]);
					tft.drawRect(0, 32, 160, 96, TFT_YELLOW);					// Rysuj ramke okna
				}
			}

			break;

		default:
			break;
	}
}

void prevScr()
{
	switch (screen)
	{
		case SCR_DIST:
			if (conf.getInt("scr_obd2") == 1) screen = SCR_OBD2;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;

			break;

		case SCR_TIME:
			if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_obd2") == 1) screen = SCR_OBD2;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;

			break;

		case SCR_NAVI:
			if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_obd2") == 1) screen = SCR_OBD2;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;

			break;

		case SCR_COMBO:
			if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_obd2") == 1) screen = SCR_OBD2;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;

			break;

		case SCR_GPS:
			if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_obd2") == 1) screen = SCR_OBD2;

			break;

		case SCR_OBD2:
			if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;

			break;

		default:
			//screen = SCR_NAVI;													// Wyjscie z podekranow nawigacji
			//navi_state = NO_TARGET;												// Wylacz tryb nawigacji
			break;
	}

	openScreen(screen, true);
}

void nextScr()
{
	switch (screen)
	{
		case SCR_DIST:
			if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_obd2") == 1) screen = SCR_OBD2;

			break;

		case SCR_TIME:
			if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_obd2") == 1) screen = SCR_OBD2;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;

			break;

		case SCR_NAVI:
			if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_obd2") == 1) screen = SCR_OBD2;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;

			break;

		case SCR_COMBO:
			if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_obd2") == 1) screen = SCR_OBD2;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;

			break;

		case SCR_GPS:
			if (conf.getInt("scr_obd2") == 1) screen = SCR_OBD2;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;

			break;

		case SCR_OBD2:
			if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;

			break;

		default:
			//screen = SCR_NAVI;													// Wyjscie z podekranow nawigacji
			//navi_state = NO_TARGET;												// Wylacz tryb nawigacji
			break;
	}

	openScreen(screen, true);
}

void openScreen(enum SCREENS scr, bool remember)
{
	if (closeScr) closeScr();													// Wykonaj zamkniecie poprzedniego ekranu (jesli ustawione)

	if (remember) eeram.write(LAST_SCREEN, screen);								// Zapamietaj aktualny ekran

	screen_t screen_buf;
	btn_t key_buf;																// Pomocniczy bufor dla kontrolki
	memcpy_P(&screen_buf, &screen_data[screen], sizeof(screen_buf));			// Kopiuj opis z tabeli do bufora

	if (screen_buf.scr_close_exe) closeScr = screen_buf.scr_close_exe;			// Jezeli jest funkcja zamkniecia ekranu, to ustaw wskaznik
	else closeScr = NULL;

	if (screen_buf.scr_open_exe) screen_buf.scr_open_exe();						// Uruchom funkcje skojarzona z otwarciem nowego ekranu

	ctrl_lst.clear();															// Wyczysc stara liste przyciskow

	// Jezeli sa jakies przyciski, to utworz z nich liste
	for (uint8_t btn = 0; btn < sizeof(ctrls_data) / sizeof(btn_t); btn++)		// Przejrzyj wszystkie przyciski we FLASH
	{
		memcpy_P(&key_buf, &ctrls_data[btn], sizeof(key_buf));					// Kopiowanie definicji przycisku z FLASH do RAM

		if (key_buf.screen_id == screen) ctrl_lst.push_back(key_buf);			// Dodaj przycisk do listy jezeli nalezy do ekranu
	}

	SimpleList<btn_t>::iterator idx = ctrl_lst.begin();

	for (uint8_t i = 0; i < ctrl_lst.size(); i++, idx++)						// Rysuj kontrolki
		renderCtrl(idx);

	renderScreen(screen);
}

void renderScreen(enum SCREENS scr)
{
	static uint32_t /*old_distance1,*/ old_distance2;
	static uint8_t old_secs, old_mins, old_hrs;
	static uint32_t old_distance_to;
	static uint16_t old_course_to;
	uint8_t cursor_pos;
	uint8_t sats_on_sky = 0;

	switch (scr)
	{
		case SCR_UPDATE:
			// Prosty progressbar zapelniajacy ekran
			tft.drawLine(0, map(fw_upd_progress, 0, 100, 128, 32), 160, map(fw_upd_progress, 0, 100, 128, 32), TFT_YELLOW);
			break;

		case SCR_WELCOME:
			TJpgDec.setJpgScale(1);
			TJpgDec.setSwapBytes(true);
			TJpgDec.setCallback(tftImgOutput);
			TJpgDec.drawFsJpg(0, 0, "/logogray160x128.jpg");					// Obrazek powitalny
			tft.setCursor(160 - 10 * 6, TFT_HEIGHT / 2 + 8);
			tft.setTextColor(TFT_GREENYELLOW);
			tft.setTextSize(1);
			tft.printf("FW: %d", FW_VERSION);
			break;

		case SCR_DIST:
			//cursor_pos = computeEraseArea(distance1, old_distance1, 4);			// Selektywne zamazywanie dystansu odcinka Max 10^4 metrow
			//tft.fillRect((cursor_pos * 32), 33, 160 - (cursor_pos * 32) - 1, 48, TFT_BLACK);
			tft.setTextFont(7);													// Font 7segment
			tft.setCursor(0, 33);												// Gorny lewy rog
			tft.setTextColor(TFT_YELLOW, TFT_BLACK);
			tft.printf("%05d", pulses.gps_current);
			cursor_pos = computeEraseArea(distance_total, old_distance2, 6);	// Selektywne zamazywanie dystansu globalnego Max 10^6 metrow

			if (cursor_pos > 3)													// Jezeli zmiany ponizej tysiaca
				tft.fillRect(160 - ((7 - cursor_pos) * 21),
				             81,
				             ((7 - cursor_pos) * 21) - 1,
				             25, TFT_BLACK);
			else																// Zmiany powyzej tysiaca
				tft.fillRect(160 - ((7 - cursor_pos) * 21) - 10,				// 10 to poprawka na kropke oddzielajaca
				             81,
				             ((7 - cursor_pos) * 21) - 1 + 10,
				             25, TFT_BLACK);

			tft.setFreeFont(&FreeMonoBold18pt7b);
			tft.setTextColor(TFT_GOLD, TFT_BLACK);								// Kolor tla nie dziala dla FreeFont, wiec trzeba czyscic
			tft.setCursor(160 - (3 * 21), 104);
			tft.printf("%03d", distance_total % 1000);							// Kilometry
			tft.setCursor(0, 104);
			tft.printf("%04d", (distance_total % 10000) / 1000);				// Metry
			tft.fillCircle(90, 102, 2, TFT_GOLD);								// Kropka oddziela kilometry od metrow
			old_distance2 = distance_total;										// Zapamietaj poprzednie wartosci
			break;

		case SCR_TIME:
			if (timer_state == TMR_RUN)
			{
				computeTime();
				tft.setTextSize(2);												// Podwojna wielkosc znaku
				tft.setTextColor(TFT_WHITE, TFT_BLACK);
				tft.setCursor(160 - (3 * 12), 36);
				tft.printf("%02d", tmr_frac);									// Ulamki

				if (old_secs != tmr_secs)										// Jezeli uplynela nastepna sekunda
				{
					if (!(tmr_secs % 10))										// Renderuj sekundy dwucyfrowe, jezeli podzielne przez 10 bez reszty
					{
						tft.setCursor(160 - (6 * 12), 36);
						tft.printf("%02d", tmr_secs);
					}
					else														// Renderuj sekundy jednocyfrowe
					{
						tft.setCursor(160 - (5 * 12), 36);
						tft.printf("%d", tmr_secs % 10);
					}
				}

				if (old_mins != tmr_mins)										// Jezeli uplynela nastepna minuta
				{
					if (!(tmr_mins % 10))										// Renderuj minuty dwucyfrowe, jezeli podzielne przez 10 bez reszty
					{
						tft.setCursor(160 - (9 * 12), 36);
						tft.printf("%02d", tmr_mins);
					}
					else														// Renderuj minuty jednocyfrowe
					{
						tft.setCursor(160 - (8 * 12), 36);
						tft.printf("%d", tmr_mins % 10);
					}
				}

				if (old_hrs != tmr_hrs)											// Renderuj godziny
				{
					tft.setCursor(160 - (12 * 12), 36);
					tft.printf("%02d", tmr_hrs);
				}

				old_hrs = tmr_hrs;
				old_mins = tmr_mins;
				old_secs = tmr_secs;
			}

			break;

		case SCR_NAVI:
			if (navi_state != NAVI_WPTS)										// Jezeli nie ma nawigacji na waypoint
			{
				tft.setTextSize(2);
				tft.setCursor(4, 94);
				tft.setTextColor(TFT_YELLOW, TFT_BLACK);
				tft.printf_P("%4d.%03d", distance_current / 1000, distance_current % 1000);	// Dystans odcinka
				tft.setCursor(4, 110);
				tft.setTextColor(TFT_GOLD, TFT_BLACK);
				tft.printf_P("%04d.%03d", distance_total / 1000, distance_total % 1000);	// Dystans globalny
			}
			else
			{
				// Odleglosc od waypointa (w metrach)
				uint32_t distance_to = gps.distanceBetween(gps.location.lat(), gps.location.lng(), dest_wpt.lat, dest_wpt.lon);
				// Kurs na waypoint
				uint16_t course_to = (uint16_t) gps.courseTo(gps.location.lat(), gps.location.lng(), dest_wpt.lat, dest_wpt.lon);

				if ((distance_to != old_distance_to) || (course_to != old_course_to))
				{
					tft.setTextColor(TFT_YELLOW, TFT_BLACK);
					tft.setCursor(4, 86);
					tft.printf_P("Cel: %s", dest_wpt.name);
					tft.setCursor(4, 94);

					if (distance_to < 1000)	tft.printf_P("WPT: %03dm  ", distance_to);			// Dystans do celu (waypointa)
					else tft.printf_P("WPT: %02d,%dkm", distance_to / 1000, (distance_to % 1000) / 100);

					tft.setCursor(4, 102);

					// Godzin i minut do celu
					if (gps.speed.kmph() > 1)
					{
						uint8_t h = distance_to / 1000 / gps.speed.kmph();
						uint8_t m = (uint8_t)(distance_to / 1000 / gps.speed.kmph() * 60) % 60;
						tft.printf_P("ETA: %02d:%02d", h, m);
					}
					else tft.printf_P("ETA: --:--");

					// W zaleznosci od odleglosci do waypointa, zaznacz go innym kolorem i w innej pozycji wzgledem horyzontu
					if (distance_to > 2000) renderWpt(old_course_to - gps.course.deg(), (point_t) {122, 70}, 34, TFT_WHITE);
					else if (distance_to > 1000) renderWpt(old_course_to - gps.course.deg(), (point_t) {122, 70}, 34, TFT_RED);
					else if (distance_to > 500) renderWpt(old_course_to - gps.course.deg(), (point_t) {122, 70}, 26, TFT_YELLOW);
					else if (distance_to > 250) renderWpt(old_course_to - gps.course.deg(), (point_t) {122, 70}, 18, TFT_GREENYELLOW);
					else renderWpt(old_course_to - gps.course.deg(), (point_t) {122, 70}, 10, TFT_GREEN);
					old_distance_to = distance_to;
					old_course_to = course_to;
				}
			}

			tft.setTextSize(1);
			break;

		case SCR_COMBO:
			tft.setTextSize(2);
			tft.setCursor(4, 34);
			tft.setTextColor(TFT_YELLOW, TFT_BLACK);
			tft.printf("%4d.%03d", distance_current / 1000, distance_current % 1000);			// Dystans odcinka
			tft.setCursor(4, 50);
			tft.setTextColor(TFT_GOLD, TFT_BLACK);
			tft.printf("%04d.%03d", distance_total / 1000, distance_total % 1000);		// Dystans globalny
			tft.setTextSize(1);
			tft.setTextColor(TFT_WHITE, TFT_BLACK);
			tft.setCursor(4 + 7 * 6, 66);

			if (timer_state == TMR_RUN)
			{
				computeTime();
				tft.printf_P("%02d:%02d:%02d", tmr_hrs, tmr_mins, tmr_secs);	// Czas stoperowy bez ulamkow
			}

			tft.setCursor(100, 80);
			tft.printf_P("%3d km/h", speed);									// Predkosc
			tft.setCursor(100, 96);
			tft.printf_P("%2d.%d V", volt / 10, volt % 10);						// Napiecie
			tft.setCursor(100, 114);
			tft.printf_P("%3d %cC", temperature, 247);							// Temperatura debug

			if (new_course)														// Nowy kurs
			{
				renderCompassNeedle(course, (point_t) {116, 48}, 15);			// Przerysuj igle kompasu
				tft.setCursor(134, 34);											// Napisz obok kompasu kurs
				tft.printf_P("%3d%c", (int) gps.course.deg(), 247);				// Symbol stopni
				tft.setCursor(134, 42);
				tft.printf_P("%-3s", gps.cardinal(course));						// Wyrownanie do lewej (i jednoczesnie kasowanie)
				new_course = false;												// Skasuj flage
			}

			speed_gauge.update(speed);
			volt_gauge.update(volt / 10);
			temp_gauge.update(temperature);										// debug
			break;

		case SCR_GPS:
			satsu_gauge.update(gps.satellites.value());							// Sledzone satelity

			if (sat_stats_ready)												// Jezeli sa nowe statystyki
			{
				for (size_t i = 0; i < MAX_SATS; i++)
				{
					if (sats_stats[i].prn) sats_on_sky++;
				}

				sat_stats_ready = false;										// Skasuj flage aktualnych statystyk
			}

			satsv_gauge.update(sats_on_sky);									// Widoczne satelity
			dop_gauge.update(MAX_DOP - gps.hdop.hdop());						// Precyzja lokalizacji

			if (new_course)														// Nowy kurs
			{
				renderCompassNeedle(course, (point_t) {127, 64}, 30);			// Przerysuj igle kompasu
				tft.setCursor(120, 100);										// Napisz pod kompasem kurs
				tft.printf_P("%3d%c", (int) gps.course.deg(), 247);				// Symbol stopni
				tft.setCursor(120, 108);
				tft.printf_P("%-3s", gps.cardinal(course));						// Wyrownanie do lewej (i jednoczesnie kasowanie)
				new_course = false;												// Skasuj flage
			}

			if (gps.location.isUpdated())
			{
				tft.setCursor(4 + 7 * 6, 84);									// Tylko wartosc, wiec przesuniety kursor
				tft.printf_P("%8.5f%c", gps.location.lat(), 247);
				tft.setCursor(4 + 7 * 6, 92);									// Tylko wartosc, wiec przesuniety kursor
				tft.printf_P("%8.5f%c", gps.location.lng(), 247);
				//debugI("LAT: %8.5f %09lu %s %s %s", gps.location.lat(), (long unsigned)gps.location.rawLat().billionths, gpsFormatConvert(gps.location.rawLat().deg, gps.location.rawLat().billionths, GPS_SD).c_str(), gpsFormatConvert(gps.location.rawLat().deg, gps.location.rawLat().billionths, GPS_SMS).c_str(), gpsFormatConvert(gps.location.rawLat().deg, gps.location.rawLat().billionths, GPS_SMD).c_str());
				//debugI("LON: %8.5f %09lu %s %s %s", gps.location.lng(), (long unsigned)gps.location.rawLng().billionths, gpsFormatConvert(gps.location.rawLng().deg, gps.location.rawLng().billionths, GPS_SD).c_str(), gpsFormatConvert(gps.location.rawLng().deg, gps.location.rawLng().billionths, GPS_SMS).c_str(), gpsFormatConvert(gps.location.rawLng().deg, gps.location.rawLng().billionths, GPS_SMD).c_str());
			}

			if (gps.altitude.isUpdated())
			{
				tft.setCursor(4 + 5 * 6, 100);									// Tylko wartosc, wiec przesuniety kursor
				tft.printf_P("%4d", (int) gps.altitude.meters());
			}

			if (gps.speed.isUpdated())
			{
				tft.setCursor(4 + 6 * 6, 108);									// Tylko wartosc, wiec przesuniety kursor
				tft.printf_P("%3d", (int) gps.speed.kmph());
			}

			tft.setCursor(4 + 6 * 6, 116);										// Tylko wartosc, wiec przesuniety kursor
			tft.printf_P("%3d", gps.location.age() < 999000 ? gps.location.age() / 1000 : 0);
			break;

		default:
			break;
	}
}

void renderCtrl(btn_t *ctrl)
{
	tft.setTextFont(1);
	uint16_t frame_c, fill_c, txt_c;
	bool focus, active;

	// Jezeli ustawiony MSB i key_id na młodszych bitach, to kontrolka aktywna
	if ((ctrl_state[screen][1] & 0x80) && ((ctrl_state[screen][1] & 0x7F) == ctrl->key_id)) active = true;
	else active = false;

	// Jezeli key_id taki jak [screen][0] i tryb zmiany kontrolek to kontrolka ma focus
	if ((ctrl_state[screen][0] == ctrl->key_id) && (btn_mode == CHG_CTRL)) focus = true;
	else focus = false;

	if (focus) frame_c = TFT_YELLOW;
	else frame_c = TFT_BLACK;

	txt_c = TFT_BLACK;															// Domyslny kolor etykiety

	if (active)
	{
		fill_c = TFT_MAGENTA;
		txt_c = TFT_WHITE;
	}
	else fill_c = TFT_CYAN;

	//debugI("Control: %d Focus: %d Active: %d", ctrl->key_id, focus, active);
	tft.drawRect((ctrl->x) - 1, (ctrl->y) - 1, (ctrl->w) + 2, (ctrl->h) + 2, frame_c);	// Ramka
	tft.drawRect((ctrl->x) - 2, (ctrl->y) - 2, (ctrl->w) + 4, (ctrl->h) + 4, frame_c);	// Ramka
	tft.fillRect(ctrl->x, ctrl->y, ctrl->w, ctrl->h, fill_c);					// Wypelnienie
	tft.setTextColor(txt_c);													// Tekst

	if (active)	tft.drawString(ctrl->lbl_on, (ctrl->x) + 2, (ctrl->y) + 2);
	else tft.drawString(ctrl->lbl_off, (ctrl->x) + 2, (ctrl->y) + 2);
}

void renderToolbar(enum TOOLBAR_ITEMS item)
{
	char str[16];
	char *res;

	switch (item)
	{
		case WIFI_XOFF:
			tft.drawBitmap(TBARX_WIFI, 0, wifi_sym, 32, 32, TFT_LIGHTGREY, TFT_BLACK);
			break;

		case WIFI_XAP:
			tft.drawBitmap(TBARX_WIFI, 0, ap_sym, 32, 32, TFT_GREEN, TFT_BLACK);
			tft.setTextColor(TFT_RED);
			WiFi.softAPIP().toString().toCharArray(str, sizeof(str));
			res = strtok(str, ".");

			for (size_t i = 0; i < 4; i++)										// Cztery oktety adresu IP
			{
				tft.setCursor(TBARX_WIFI, i * 8);
				tft.print(res);
				res = strtok(NULL, ".");
			}

			break;

		case WIFI_XSTA:
			tft.drawBitmap(TBARX_WIFI, 0, wifi_sym, 32, 32, TFT_GREEN, TFT_BLACK);
			tft.setTextColor(TFT_RED);
			WiFi.localIP().toString().toCharArray(str, sizeof(str));
			res = strtok(str, ".");

			for (size_t i = 0; i < 4; i++)										// Cztery oktety adresu IP
			{
				tft.setCursor(TBARX_WIFI, i * 8);
				tft.print(res);
				res = strtok(NULL, ".");
			}

			break;

		case GPS_DATETIME:
			// Co sekunde zmien kolor (miganie)
			even_odd ? tft.setTextColor(TFT_WHITE, TFT_BLACK) : tft.setTextColor(TFT_BLACK, TFT_BLACK);
			tft.setCursor(TBARX_TIME, 0);
			tft.setTextSize(1);
			tft.setTextFont(1);
			tft.printf_P("%02d:%02d", ((gps.time.hour() + conf.getInt("tz")) % 24), gps.time.minute());
			break;

		case GPS_FIX:
			tft.drawBitmap(TBARX_GPS, 0, satellite_sym, 32, 32, TFT_GREEN);
			break;

		case GPS_NOFIX:
			tft.drawBitmap(TBARX_GPS, 0, satellite_sym, 32, 32, TFT_RED);
			break;

		case MEM_FREE:
			tft.drawBitmap(TBARX_MEMORY, 0, memory_sym, 32, 32, TFT_GREEN);
			break;

		case MEM_AVG:
			tft.drawBitmap(TBARX_MEMORY, 0, memory_sym, 32, 32, TFT_YELLOW);
			break;

		case MEM_FULL:
			tft.drawBitmap(TBARX_MEMORY, 0, memory_sym, 32, 32, TFT_RED);
			break;

		case SD_OK:
			tft.drawBitmap(TBARX_SD, 0, sd_sym, 32, 32, TFT_GREEN);
			break;

		case SD_NOOK:
			break;

		case SD_OFF:
			tft.drawBitmap(TBARX_SD, 0, sd_sym, 32, 32, TFT_DARKGREY);
			break;

		case REC_ON:
			tft.setCursor(TBARX_TIME, 16);
			tft.setTextSize(1);
			tft.setTextFont(1);
			tft.setTextColor(TFT_WHITE, TFT_RED);
			tft.print(F("*REC*"));
			break;

		case REC_OFF:
			tft.fillRect(TBARX_TIME, 16, 30, 8, TFT_BLACK);
			break;

		case OBD2_OFF:
			tft.setCursor(TBARX_OBD2, 8);
			tft.setTextSize(1);
			tft.setTextFont(1);
			tft.setTextColor(TFT_BLACK, TFT_BLACK);
			tft.print(F("OBD2"));
			break;

		case OBD2_AP:
			tft.setCursor(TBARX_OBD2, 8);
			tft.setTextSize(1);
			tft.setTextFont(1);
			tft.setTextColor(TFT_BLUE, TFT_BLACK);
			tft.print(F("OBD2"));
			break;

		case OBD2_IF:
			tft.setCursor(TBARX_OBD2, 8);
			tft.setTextSize(1);
			tft.setTextFont(1);
			tft.setTextColor(TFT_YELLOW, TFT_BLACK);
			tft.print(F("OBD2"));
			break;

		case OBD2_CAR:
			tft.setCursor(TBARX_OBD2, 8);
			tft.setTextSize(1);
			tft.setTextFont(1);
			tft.setTextColor(TFT_GREEN, TFT_BLACK);
			tft.print(F("OBD2"));
			break;

		case OBD2_ERROR:
			tft.setCursor(TBARX_OBD2, 8);
			tft.setTextSize(1);
			tft.setTextFont(1);
			tft.setTextColor(TFT_RED, TFT_BLACK);
			tft.print(F("OBD2"));
			break;

		default:
			break;
	}
}

void computeDistance()
{
	if (fix)
	{
		if (!counter_disable)
		{
			pulses.imp_current += gps_distance;
			pulses.imp_total += gps_distance;
		}

		pulses.imp_speed_avg += gps_distance * calibration.dist_cal / 100;	// Przeliczenie metrow na "impulsy"
	}

	distance_current = (pulses.imp_current * 100) / calibration.dist_cal;					// Obliczenie dystansu
	distance_total = (pulses.imp_total * 100) / calibration.dist_cal;

	if (distance_current > 9999999) distance_current = 9999999;

	if (distance_total > 9999999) distance_total = 9999999;
}

void computeSpeed()
{
	static uint8_t old_speed;

	if (dist_source == IMP)
	{
		speed = (pulses_speed * 100) / calibration.dist_cal;					// Metry (czyli po sekundzie predkosc [m/s])
		speed *= 3.6;															// Predkosc [km/h]
		speed = (speed + old_speed) / 2;										// Srednia z dwoch pomiarow
		old_speed = speed;														// Zapamietaj poprzednia wartosc
		pulses_speed = 0;														// Wyzeruj licznik impulsow/s
	}
	else speed = gps.speed.kmph();
}

void computeVolt()
{
	static uint16_t old_volt;
	debugI("Volt (raw): %d", analogRead(A0));
	volt = (analogRead(A0) * 22 * 10) / 1024.0;									// dzielnik rezystorowy 1:22 [*10]
	volt = (volt * 100) / calibration.volt_cal;									// Kalibracja
	volt = (volt + old_volt) / 2;												// Srednia z dwoch pomiarow
	old_volt = volt;															// Zapamietaj poprzednia wartosc
}

void computeTemp()
{
	static uint16_t old_temp;
	mux_switch(STARTUP);
	debugI("Temperature (raw): %d", analogRead(A0));
	temperature = (analogRead(A0) * 10 * 10) / 1024.0;							// dzielnik rezystorowy 1:10 [*10]
	mux_switch(RUNTIME);
	temperature = (temperature * 100) / calibration.temp_cal;					// Kalibracja
	temperature = (temperature + old_temp) / 2;									// Srednia z dwoch pomiarow
	old_temp = temperature;														// Zapamietaj poprzednia wartosc
	old_temp = radians(100);
}

void computeTime()
{
	tmr_ms = millis() - tmr_start_time;											// Różnica w milisekundach miedzy teraz a startem
	tmr_sec_num = tmr_ms / 1000;												// Sekundy odliczone od startu
	tmr_hrs = tmr_sec_num / 3600;												// Uplynelo godzin
	tmr_mins = (tmr_sec_num - (tmr_hrs * 3600)) / 60;							// Minut
	tmr_secs = tmr_sec_num - (tmr_hrs * 3600) - (tmr_mins * 60);				// I sekund
	tmr_frac = (tmr_ms % 1000) / 10;											// Ulamki sekundy
}

uint8_t computeEraseArea(uint32_t new_val, uint32_t old_val, uint8_t length)
{
	uint8_t digit_pos = 0;

	for (size_t pos = 0; pos < length; pos++)									// Ilosc pozycji dziesietnych liczby
	{
		uint32_t d1 = new_val / pow(10, length - pos);							// Kolejne cyfry od lewej
		uint32_t d2 = old_val / pow(10, length - pos);

		if (d1 - d2) break;														// Jezeli zmiana na pierwszej pozycji od lewej, przerwij
		else digit_pos++;														// A jezeli sa rowne, zwieksz licznik cyfr
	}

	return digit_pos;
}

void clearWindow()
{
	tft.fillRect(0, 32, 160, 96, TFT_BLACK);									// Wyczysc ekran poza toolbarem

	if (btn_mode == CHG_SCR) tft.drawRect(0, 32, 160, 96, TFT_YELLOW);			// Ramka okna - sygnalizuje przelaczanie ekranow

	tft.setTextFont(1);
	tft.setTextSize(1);
}

void tftMsg(String message)
{
	tft.fillRect(10, 40, 140, 40, TFT_BLACK);
	tft.drawRect(10, 40, 140, 40, TFT_RED);
	tft.drawRect(11, 41, 138, 38, TFT_RED);
	tft.drawCentreString(message, 80, 60, 1);
	one_shoot_tmr.once_ms(1500, std::bind(openScreen, screen, true));			// Odswiez po chwili ekran
}

void openDist() {clearWindow();}

void openTime()
{
	clearWindow();
	tft.setTextSize(2);															// Podwojna wielkosc znaku
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.setCursor(160 - (12 * 12), 36);
	tft.printf_P("00:00:00.00");

	if (timer_state == TMR_RUN) timer_tmr.attach_ms(TIMER_REFRESH, std::bind(renderScreen, screen));// Przerysuj po 50ms ekran
}

void closeTime() {if (timer_state == TMR_RUN) timer_tmr.detach();}				// Wylacz odswierzanie ekranu stopera

void openNavi()
{
	clearWindow();

	if (navi_state == NAVI_WPTS)													// Jezeli to tryb nawigacji do waypointa
	{
		tft.drawCircle(122, 70, 30, TFT_WHITE);										// Okrag kompasu/horyzontu
		tft.fillTriangle(122, 70 - 4, 122 + 4, 70 + 4, 122 - 4, 70 + 4, TFT_WHITE);	// Symbol w srodku horyzontu
	}
	else
	{
		tft.fillTriangle(122, 70 - 4, 122 + 4, 70 + 4, 122 - 4, 70 + 4, TFT_BLACK); //Skasuj symbol w srodku horyzontu
	}
}

void openWptFileList()
{
	clearWindow();
	fs::Dir dir = SPIFFS.openDir("/");
	wptfile_lst.clear();

	while (dir.next())															// Przejrzyj katalog
	{
		if (dir.fileName().endsWith(".gpx")) wptfile_lst.push_back(dir.fileName()); // Wszystkie pliki gpx dodaj do listy
	}

	if (wptfile_lst.size())														// Jezeli sa jakies pliki na liscie - listuj
	{
		tft.setTextColor(TFT_YELLOW, TFT_BLACK);
		tft.setCursor(6, 32);
		tft.printf_P("Wybierz plik");
		listWptFiles(cur_file);
	}
	else 																		// Jak nie ma plikow
	{
		screen = SCR_NAVI;														// Wroc do ekranu nawigacji
		tftMsg(F("Brak plikow gpx"));
		delay(2000);															// NIE WIEM DLACZEGO OD RAZU RYSUJE KONTROLKI...
	}
}

void listWptFiles(uint8_t file_pos)
{
	uint8_t files_set_no = file_pos / LIST_FILES_LEN;							// Nr "Kompletu" plikow. Na ekranie miesci sie #def nazw

	if (wptfile_lst.size() > LIST_FILES_LEN)									// Jezeli pliki nie mieszcza sie na ekranie
	{
		tft.setCursor(6, 40 + 10 * 8);											// Na dole listy
		tft.setTextColor(TFT_YELLOW, TFT_BLACK);
		tft.print(F("...wiecej"));												// Zacheta
	}

	tft.setCursor(0, 40);														// Listuj zawsze od gory
	SimpleList<String>::iterator itr = wptfile_lst.begin() + (files_set_no * LIST_FILES_LEN); // Odleglosc od poczatku listy w kompletach

	for (uint8_t file = 0; file < LIST_FILES_LEN; file++)						// Najwyzej #def plikow na ekranie
	{
		uint8_t file_num = (files_set_no * LIST_FILES_LEN) + file;				// Pozycja pliku na liscie

		if (file_num == wptfile_lst.size()) break;								// Jezeli koniec listy plikow, przerwij listowanie
		else
		{
			if (file_num == file_pos)
			{
				tft.setTextColor(TFT_BLACK, TFT_YELLOW);						// Biezacy plik listuj odmiennym kolorem
				tft.setCursor(0, 40 + file * 8);
				tft.print(F(">"));												// Wskaznik graficzny
			}
			else tft.setTextColor(TFT_GREEN, TFT_BLACK);						// Pozostale normalnie

			if (itr->length() > 20)	tft.drawString(itr->substring(1, 20), 6, 40 + file * 8);
			else tft.drawString(itr->substring(1), 6, 40 + file * 8);

			itr++;
		}
	}
}

void openWptList()
{
	clearWindow();

	if (wpt_lst.size())															// Jezeli sa jakies waypointy - listuj
	{
		tft.setTextColor(TFT_YELLOW, TFT_BLACK);
		tft.setCursor(6, 32);
		tft.printf_P("Wybierz waypoint");
		listWaypoints(cur_wpt);
	}
	else 																		// Jak nie ma waypointow
	{
		screen = SCR_FILES;														// Wroc do ekranu z plikami gpx
		tftMsg(F("Brak waypointow"));
		//delay(2000);															// NIE WIEM DLACZEGO OD RAZU RYSUJE KONTROLKI...
	}
}

void listWaypoints(uint16_t wpt_pos)
{
	uint8_t wpts_set_no = wpt_pos / LIST_WPTS_LEN;								// Nr "Kompletu" waypointow. Na ekranie miesci sie #def nazw

	if (wpt_lst.size() > LIST_WPTS_LEN)											// Jezeli waypointy nie mieszcza sie na ekranie
	{
		tft.setCursor(6, 40 + 10 * 8);											// Na dole listy
		tft.setTextColor(TFT_YELLOW, TFT_BLACK);
		tft.print(F("...wiecej"));												// Zacheta
	}

	tft.setCursor(0, 40);														// Listuj zawsze od gory
	SimpleList<wpt_t>::iterator itr = wpt_lst.begin() + (wpts_set_no * LIST_WPTS_LEN); // Odleglosc od poczatku listy w kompletach

	for (uint8_t wpt = 0; wpt < LIST_WPTS_LEN; wpt++)							// Najwyzej #def waypointow na ekranie
	{
		uint8_t wpt_num = (wpts_set_no * LIST_WPTS_LEN) + wpt;					// Pozycja waypointa na liscie

		if (wpt_num == wpt_lst.size()) break;									// Jezeli koniec listy waypointow, przerwij listowanie
		else
		{
			if (wpt_num == wpt_pos)
			{
				tft.setTextColor(TFT_BLACK, TFT_YELLOW);						// Biezacy waypoint listuj odmiennym kolorem
				tft.setCursor(0, 40 + wpt * 8);
				tft.print(F(">"));												// Wskaznik graficzny
			}
			else tft.setTextColor(TFT_GREEN, TFT_BLACK);						// Pozostale normalnie

			tft.drawString(itr->name, 6, 40 + wpt * 8);
			itr++;
		}
	}
}

void openGpxInfo()
{
	clearWindow();
	tft.setTextColor(TFT_YELLOW, TFT_BLACK);
	tft.setCursor(4, 32);
	tft.printf_P("Waypoints: %5d", file_wpts);
	tft.setCursor(4, 40);
	tft.printf_P("Tracks:    %5d", file_trks);
	tft.setCursor(4, 48);
	tft.printf_P("Routes:    %5d", file_rtes);
	/*
	debugI("Waypoints: %d Routes: %d Tracks: %d", file_wpts, file_rtes, file_trks);
	SimpleList<wpt_t>::iterator itw = wpt_lst.begin();							// DEBUG

	for (size_t i = 0; i < wpt_lst.size(); i++, itw++) debugI("WPT: NAME=%s LAT=%f LON=%.5f", itw->wpt_name, itw->wpt_lat, itw->wpt_lon);

	SimpleList<rte_t>::iterator itr = rte_lst.begin();							// DEBUG

	for (size_t i = 0; i < rte_lst.size(); i++, itr++) debugI("RTE: NAME=%s SIZE:%d", itr->rte_name, itr->rte_size);

	SimpleList<trk_t>::iterator itt = trk_lst.begin();							// DEBUG

	for (size_t i = 0; i < trk_lst.size(); i++, itt++) debugI("TRK: NAME=%s SIZE:%d", itt->trk_name, itt->trk_size);
	*/
}

void openCombo()
{
	clearWindow();
	tft.setTextSize(2);
	tft.setCursor(4, 34);
	tft.setTextColor(TFT_YELLOW);
	tft.printf("%4d.%03d", distance_current / 1000, distance_current % 1000);
	tft.setCursor(4, 50);
	tft.setTextColor(TFT_GOLD);
	tft.printf("%04d.%03d", distance_total / 1000, distance_total % 1000);
	tft.setTextSize(1);
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.setCursor(4, 66);
	tft.printf_P("Timer: 00:00:00");
	tft.drawCircle(116, 48, 15, TFT_WHITE);										// Okrag kompasu
	speed_gauge.show();
	volt_gauge.show();
	temp_gauge.show();
}

void openGPS()
{
	clearWindow();
	satsv_gauge.show();
	satsu_gauge.show();
	dop_gauge.show();
	tft.drawCircle(127, 64, 30, TFT_WHITE);										// Okrag kompasu
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.setCursor(4, 84);
	tft.printf_P("Lat: N %8.5f%c", gps.location.lat(), 247);
	tft.setCursor(4, 92);
	tft.printf_P("Lon: E %8.5f%c", gps.location.lng(), 247);
	tft.setCursor(4, 100);
	tft.printf_P("Alt: %4d m", (int) gps.altitude.meters());
	tft.setCursor(4, 108);
	tft.printf_P("Spd:  %3d km/h", (int) gps.speed.kmph());
	tft.setCursor(4, 116);
	tft.printf_P("FIX:  %3d s", gps.location.age() < 999000 ? gps.location.age() / 1000 : 0);
}

void openOBD2()
{
	clearWindow();
}

void closeOBD2() {}

void btnOBD2Connect(bool on_off)
{
	if (on_off)																	// Laczenie do OBD2
	{
		if (WiFi.getMode() == WIFI_STA)											// Jezeli jako stacja podlaczona do AP
		{
			WiFi.disconnect();													// Odlacz

			while (WiFi.isConnected()) delay(100);								// Czekaj na odlaczenie

			wifi_multi.cleanAPlist();											// Wyczysc liste sieci
		}
		else WiFi.mode(WIFI_STA);												// Jezeli byl jako AP, ustaw tryb stacji

		renderToolbar(WIFI_XOFF);
		wifi_multi.addAP(conf.getValue("obd2_net"));							// Dodaj siec interface OBD2
		connected = false;														// Skasuj flagi polaczenia
		internet = false;
		uint32_t timeout = millis();

		while (!connected)														// Czekaj na polaczenie z interfejsem OBD2
		{
			initWiFiStaAp(false);												// Bez opcji zostania AP
			delay(500);

			if ((millis() - timeout) > OBD2_CONNECT) break;						// Timeout na polaczenie do AP OBD2
		}

		if (WiFi.isConnected())													// Jezeli polaczony z AP OBD2
		{
			client.connect(WiFi.gatewayIP(), 35000);							// Polacz z interfejsem OBD2 na port 35000

			while (!client.connected()) delay(100);								// Czekaj na polaczenie socketa

			renderToolbar(OBD2_AP);

			if (String(txRawOBD2("ATI\r").startsWith(F("ATI\rELM327")))) renderToolbar(OBD2_IF);

			initOBD2();
			//tft.drawString(txATOBD2("@1"), 0, 108);							// Info
			//tft.drawString(txATOBD2("DP"), 0, 116);							// DEBUG (default protocol)
			//tft.drawString(txRawOBD2("0100\r"), 0, 108);						// PIDy
			//tft.drawString(txRawOBD2("0101\r"), 0, 116);						// MIL, DTCs
			//tft.drawString(txRawOBD2("0902\r"), 0, 108);						// VIN (nie dziala)
		}
		else																	// Nie udalo sie polaczyc z interfejsem OBD2
		{
			wifi_multi.cleanAPlist();											// Wyczysc liste sieci
			initWiFi();															// Inicjalizuj sieci WiFi poza OBD2

			while (!connected)													// Czekaj na polaczenie do normalnej sieci
			{
				initWiFiStaAp(true);
				delay(500);
			}

			CTRL_OFF;															// Wylacz kontrolke
			openScreen(SCR_OBD2, true);											// Przerysuj ekran
		}
	}
	else																		// Odlaczenie z OBD2
	{
		if (client.connected()) client.stop();									// Zatrzymaj klienta wifi

		while (client.connected()) delay(100);									// Czekaj na zamkniecie socketa

		renderToolbar(OBD2_OFF);
		WiFi.disconnect();														// Odlacz
		wifi_multi.cleanAPlist();												// Wyczysc liste sieci
		renderToolbar(WIFI_XOFF);
		connected = false;														// Skasuj flagi polaczenia
		internet = false;
		initWiFi();																// Inicjalizuj WiFi bez OBD2

		while (!connected)
		{
			initWiFiStaAp(true);
			delay(500);
		}
	}
}

void btnOBD2Errors(bool on_off) {}
void btnOBD2Reads1(bool on_off) {}
void btnOBD2Reads2(bool on_off) {}
void btnOBD2Reads3(bool on_off) {}

String txATOBD2(const char *request)
{
	char command[32] = "";
	sprintf(command, "AT%s\r", request);										// Doklej AT i CR
	String response = txRawOBD2(command);										// Wyslij kompletna komende
	return response;
}

String txRawOBD2(const char *request)
{
	String obd2_response = "";
	client.print(request);														// Wyslij komende do ELM327
	uint32_t obd2_timeout = millis();

	while (client.available() == 0)												// Czekaj na odpowiedz z interfejsu OBD2
	{
		if (millis() - obd2_timeout > OBD2_RESPONSE)
		{
			if (client.connected())
			{
				renderToolbar(OBD2_ERROR);										// Timeout
				return obd2_response;											// Pusty string
			}
		}
	}

	while (client.available()) obd2_response += char(client.read());			// Odpowiedz z interfejsu OBD2 i/lub ECU

	return obd2_response;
}

void initOBD2()
{
	txATOBD2("D");																// Defaulty
	txATOBD2("E0");																// Echo OFF
	txATOBD2("H1");																// Header ON
	txATOBD2("L0");																// LF OFF
	txATOBD2("S0");																// Spacje OFF
	txATOBD2("SP2");															// Protokol AUTO (2=VPW)
}

void meantimeSave(void)
{
	static uint8_t positions;													// Pierwszy miedzyczas
	tft.setTextSize(1);
	tft.setCursor(5, 36 + 20 + ((positions % 8) * 8));							// Po 8 zapisach nadpisuje od nowa
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.printf("%02d ", ++positions);
	tft.setTextColor(TFT_YELLOW, TFT_BLACK);
	tft.printf("%02d:%02d:%02d.%02d", tmr_hrs, tmr_mins, tmr_secs, tmr_frac);
}
void satCustomInit()
{
	for (uint8_t i = 0; i < 4; ++i)												// Inicjalizacja statystyk satelitow
	{
		satNumber[i].begin(gps, "GPGSV", 4 + 4 * i);							// offsets 4, 8, 12, 16
		elevation[i].begin(gps, "GPGSV", 5 + 4 * i);							// offsets 5, 9, 13, 17
		azimuth[i].begin(gps, "GPGSV", 6 + 4 * i);								// offsets 6, 10, 14, 18
		snr[i].begin(gps, "GPGSV", 7 + 4 * i);									// offsets 7, 11, 15, 19
	}
}
void btnStopStart(bool on_off) {counter_disable = on_off;}						// Zmien flage naliczania dystansu
void btnDecDist(bool on_off)
{
	if (pulses.imp_current) pulses.imp_current--;								// Tylko jezeli jest z czego odejmowac

	if (pulses.imp_total) pulses.imp_total--;

	computeDistance();
	CTRL_OFF;
	openScreen(screen, false);
}
void btnIncDist(bool on_off)
{
	if (pulses.imp_current < 999999) pulses.imp_current++;

	if (pulses.imp_total < 999999) pulses.imp_total++;

	computeDistance();
	CTRL_OFF;
	openScreen(screen, false);
}

void btnSaveTrk(bool on_off)
{
	if (!fix)
	{
		tftMsg(F("Brak FIXa!!!"));
		CTRL_OFF;																// Skasuj MSB jako znacznik deaktywowanej kontrolki
		return;
	}

	if (on_off)
	{
		sprintf(gpx_file, "/trk%02d%02d%02d_%02d-%02d.gpx", gps.date.year() - 2000, gps.date.month(), gps.date.day(),
		        ((gps.time.hour() + conf.getInt("tz")) % 24), gps.time.minute());
		SPIFFS_file = SPIFFS.open(gpx_file, "w");
		SPIFFS_file.print(F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n"	// Naglowek pliku
		                    "<gpx version=\"1.1\" creator=\"SDMoto\" xmlns=\"http://www.topografix.com/GPX/1/1\" "
		                    "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
		                    "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\r\n"
		                    "\t<trk>\r\n"));
		SPIFFS_file.printf_P("\t<name>trk%02d-%02d</name>\r\n", (gps.time.hour() + conf.getInt("tz")) % 24, gps.time.minute());
		SPIFFS_file.print(F("\t\t<trkseg>\r\n"));
		SPIFFS_file.print(F("\t\t</trkseg>\r\n\t</trk>\r\n</gpx>\r\n"));		// 30 znakow na koncu!!!
		SPIFFS_file.close();
		addWpt2Trk();															// Pierwszy waypoint od razu
		navi_state = REC_TRK;													// Ustaw flage nagrywania sladu gpx
		renderToolbar(REC_ON);													// Na toolbarze pokaz ze sie nagrywa
	}
	else
	{
		navi_state = NO_TARGET;													// Skasuj flage nagrywania
		renderToolbar(REC_OFF);													// Na toolbarze usun wskaznik nagrywania
	}
}
void addWpt2Trk(void)
{
	char wpt_date[22];															// Data zalogowania WPT
	sprintf(wpt_date, "%4d-%02d-%02dT%02d:%02d:%02dZ", gps.date.year(), gps.date.month(),
	        gps.date.day(), ((gps.time.hour() + conf.getInt("tz")) % 24), gps.time.minute(), gps.time.second());
	SPIFFS_file = SPIFFS.open(gpx_file, "r+");									// Otworz plik do dopisywania
	SPIFFS_file.seek(SPIFFS_file.size() - 30, SeekSet);							// Koncowka pliku - 30 bajtow
	SPIFFS_file.print(F("\t\t\t<trkpt lat=\""));								// Budowanie xml
	SPIFFS_file.printf_P("%.5f\" lon=\"", gps.location.lat());
	SPIFFS_file.printf_P("%.5f\">\r\n", gps.location.lng());
	SPIFFS_file.print(F("\t\t\t\t<time>"));
	SPIFFS_file.print(wpt_date);
	SPIFFS_file.print(F("</time>\r\n\t\t\t\t<ele>"));
	SPIFFS_file.printf_P("%d</ele>\r\n\t\t\t\t<hdop>", (int) gps.altitude.meters());
	SPIFFS_file.printf_P("%.1f</hdop>\r\n\t\t\t</trkpt>\r\n", gps.hdop.hdop());
	SPIFFS_file.print(F("\t\t</trkseg>\r\n\t</trk>\r\n</gpx>\r\n"));			// Ostatnie 30 bajtow
	SPIFFS_file.close();
}
void btnSaveWpt(bool on_off)
{
	if (!fix)
	{
		tftMsg(F("Brak FIXa!!!"));
		CTRL_OFF;																// Skasuj MSB jako znacznik deaktywowanej kontrolki
		return;
	}

	if (on_off)
	{
		sprintf(gpx_file, "/wpt%02d%02d%02d_%02d-%02d.gpx", gps.date.year() - 2000, gps.date.month(), gps.date.day(),
		        ((gps.time.hour() + conf.getInt("tz")) % 24), gps.time.minute());
		SPIFFS_file = SPIFFS.open(gpx_file, "w");
		SPIFFS_file.print(F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n"	// Naglowek pliku
		                    "<gpx version=\"1.1\" creator=\"SDMoto\" xmlns=\"http://www.topografix.com/GPX/1/1\" "
		                    "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
		                    "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\r\n"
		                    "</gpx>\r\n"));										// Zamykajacy tag (8 bajtow)
		SPIFFS_file.close();
		addWpt2Wpt(true);														// Pierwszy waypoint
		navi_state = REC_WPTS;
	}
	else navi_state = NO_TARGET;
}
void addWpt2Wpt(bool reset_num)
{
	static uint16_t wpt_num;													// Licznik waypointow

	if (reset_num) wpt_num = 0;													// Kasowanie numeru waypointa, jezeli nowy plik

	renderToolbar(REC_ON);														// Pokaz wskaznik nagrywania
	one_shoot_tmr.once_ms(500, std::bind(renderToolbar, REC_OFF));				// Po pol sekundzie go usun
	SPIFFS_file = SPIFFS.open(gpx_file, "r+");									// Otworz plik do dopisywania
	SPIFFS_file.seek(SPIFFS_file.size() - 8, SeekSet);							// Koncowka pliku - 8 bajtow
	debugI("Waypoint %d", wpt_num);
	SPIFFS_file.print(F("\t<wpt lat=\""));										// Budowanie xml
	SPIFFS_file.printf_P("%.5f\" lon=\"", gps.location.lat());
	SPIFFS_file.printf_P("%.5f\">\r\n", gps.location.lng());
	SPIFFS_file.printf_P("\t<name>WPT-%03d</name>\r\n\t</wpt>\r\n</gpx>\r\n", wpt_num++);
	SPIFFS_file.close();
}
void btnNav2Wpt(bool on_off)
{
	if (!fix)
	{
		tftMsg(F("Brak FIXa!!!"));
		CTRL_OFF;																// Skasuj MSB jako znacznik deaktywowanej kontrolki
		return;
	}

	if (navi_state != NAVI_WPTS)
	{
		CTRL_OFF;																// Skasuj MSB jako znacznik deaktywowanej kontrolki
		screen = SCR_FILES;
		openScreen(screen, false);
		navi_state = NAVI_WPTS;													// Tryb nawigacji do waypointa
	}
	else
	{
		navi_state = NO_TARGET;
		tft.fillRect(96, 32, 64, 96, TFT_BLACK);								// Usun horyzont
	}
}
void btnPrevWptFile(bool on_off)
{
	if (cur_file > 0) cur_file--;												// Poprzedni plik z listy
	else cur_file = wptfile_lst.size() - 1;										// Ostatni plik

	CTRL_OFF;																	// Skasuj MSB jako znacznik deaktywowanej kontrolki
	openScreen(screen, false);													// Przerysuj ekran
}
void btnNextWptFile(bool on_off)
{
	if (cur_file < (wptfile_lst.size() - 1)) cur_file++;						// Nastepny plik z listy
	else cur_file = 0;															// Pierwszy plik

	CTRL_OFF;																	// Skasuj MSB jako znacznik deaktywowanej kontrolki
	openScreen(screen, false);													// Przerysuj ekran
}
void btnInfoWptFile(bool on_off)
{
	parseGpxFile();
	CTRL_OFF;																	// Skasuj MSB jako znacznik deaktywowanej kontrolki
	screen = SCR_GPXINFO;
	openScreen(screen, false);													// Przejscie do ekranu z informacjami o zawartosci pliku gpx
}
void btnCancelGpxInfo(bool on_off)
{
	CTRL_OFF;																	// Skasuj MSB jako znacznik deaktywowanej kontrolki
	screen = SCR_FILES;															// Powrot do ekranu z listingiem gpx
	openScreen(screen, false);
}
void btnThisWptFile(bool on_off)
{
	parseGpxFile();																// Sprawdzenie poprawnosci xml i parsowanie do listy waypointow
	CTRL_OFF;																	// Skasuj MSB jako znacznik deaktywowanej kontrolki
	screen = SCR_WPTS;
	openScreen(screen, false);													// Przejscie do ekranu listy waypointow
}
void btnCancelFile(bool on_off)
{
	CTRL_OFF;																	// Skasuj MSB jako znacznik deaktywowanej kontrolki
	navi_state = NO_TARGET;														// Wylaczenie trybu nawigacji do waypointa
	screen = SCR_NAVI;															// Powrot do ekranu glownego nawigacji
	openScreen(screen, true);
}
void parseGpxFile()
{
	wpt_lst.clear();															// Wyczysc listy waypointow
	rte_lst.clear();															// Routow
	trk_lst.clear();															// Trackow
	SimpleList<String>::iterator itr_f = wptfile_lst.begin() + cur_file;
	file_wpts = 0;																// Init licznikow
	file_rtes = 0;
	file_trks = 0;
	debugI("Plik %s", itr_f->c_str());
	TinyXML gpx_xml;
	uint8_t buffer[150];														// Bufor dekodera XML
	gpx_xml.init((uint8_t *)buffer, sizeof(buffer), &XML_callback);				// Inicjalizacja
	gpx_xml.reset();															// wtf?
	SPIFFS_file = SPIFFS.open(itr_f->c_str(), "r");								// Otworz gpx

	while (SPIFFS_file.available())
	{
		char c = SPIFFS_file.read();
		gpx_xml.processChar(c);													// Czytaj plik i dekoduj
	}

	SPIFFS_file.close();
}
void XML_callback(uint8_t statusflags, char *tagName, uint16_t tagNameLen, char *data, uint16_t dataLen)
{
	static wpt_t temp_wpt;
	static rte_t temp_rte;
	static trk_t temp_trk;
	static uint8_t waypoint_state, route_state, track_state;
	static uint16_t rte_size, trk_size;

	/*
		if (statusflags & STATUS_START_TAG) debugI("XML Start TAG: tagname=%s", tagName);

		if (statusflags & STATUS_TAG_TEXT) debugI("XML TAG text: tagname=%s data=%s", tagName, data);

		if (statusflags & STATUS_ATTR_TEXT) debugI("XML ATTR text: tagname=%s data=%s", tagName, data);

		if (statusflags & STATUS_END_TAG) debugI("XML End TAG: tagname=%s", tagName);
	 */
	// WPT
	if ((statusflags & STATUS_START_TAG) && (!strcasecmp(tagName, "/gpx/wpt")) && waypoint_state == 0)
	{
		waypoint_state = 1;														// Nowy waypoint
		return;
	}

	if ((statusflags & STATUS_ATTR_TEXT) && (!strcasecmp(tagName, "lat")) && waypoint_state == 1)
	{
		temp_wpt.lat = atof(data);
		waypoint_state = 2;														// Jest LAT
		return;
	}

	if ((statusflags & STATUS_ATTR_TEXT) && (!strcasecmp(tagName, "lon")) && waypoint_state == 2)
	{
		temp_wpt.lon = atof(data);
		waypoint_state = 3;														// Jest LON
		return;
	}

	if ((statusflags & STATUS_TAG_TEXT) && (!strcasecmp(tagName, "/gpx/wpt/name")) && waypoint_state == 3)
	{
		strncpy(temp_wpt.name, data, WPT_NAME_LEN - 1);						// Skopiuj nazwe waypointa
		waypoint_state = 0;														// Jest nazwa
		wpt_lst.push_back(temp_wpt);											// Dodaj waypoint do listy
		file_wpts++;															// Zwieksz licznik waypointow
		return;
	}

	// RTE
	if ((statusflags & STATUS_TAG_TEXT) && (!strcasecmp(tagName, "/gpx/rte/name")) && route_state == 0)
	{
		strncpy(temp_rte.name, data, RTE_NAME_LEN - 1);						// Skopiuj nazwe routy
		route_state = 1;
		file_rtes++;															// Zwieksz licznik routow
		rte_size = 0;															// Wyzeruj licznik rozmiaru routy
		return;
	}

	if ((statusflags & STATUS_START_TAG) && (!strcasecmp(tagName, "/gpx/rte/rtept")) && route_state == 1)
	{
		rte_size++;																// Nowy punkt route
		return;
	}

	if (statusflags & STATUS_END_TAG && (!strcasecmp(tagName, "/gpx/rte")) && route_state == 1)
	{
		route_state = 0;														// Koniec routy
		temp_rte.size = rte_size;											// Uaktualnij rozmiar routy
		rte_lst.push_back(temp_rte);											// Dodaj route do listy
	}

	// TRK
	if ((statusflags & STATUS_TAG_TEXT) && (!strcasecmp(tagName, "/gpx/trk/name")) && track_state == 0)
	{
		strncpy(temp_trk.name, data, TRK_NAME_LEN - 1);						// Skopiuj nazwe tracka
		track_state = 1;
		file_trks++;															// Zwieksz licznik trackow
		trk_size = 0;															// Wyzeruj licznik rozmiaru tracka
		return;
	}

	if ((statusflags & STATUS_START_TAG) && (!strcasecmp(tagName, "/gpx/trk/trkseg/trkpt")) && track_state == 1)
	{
		trk_size++;																// Nowy punkt tracka
		return;
	}

	if (statusflags & STATUS_END_TAG && (!strcasecmp(tagName, "/gpx/trk")) && track_state == 1)
	{
		track_state = 0;														// Koniec tracka
		temp_trk.size = trk_size;											// Uaktualnij rozmiar tracka
		trk_lst.push_back(temp_trk);											// Dodaj track do listy
	}
}
void btnPrevWptSet(bool on_off)
{
	if (cur_wpt > (LIST_WPTS_LEN - 1)) cur_wpt -= LIST_WPTS_LEN;				// Poprzedni komplet waypointow

	CTRL_OFF;																	// Skasuj MSB jako znacznik deaktywowanej kontrolki
	openScreen(screen, false);													// Przerysuj ekran
}
void btnNextWptSet(bool on_off)
{
	uint16_t wpts_set_no = cur_wpt / LIST_WPTS_LEN;								// Nr "Kompletu" waypointow
	uint16_t last_set = (wpt_lst.size() - 1) / LIST_WPTS_LEN;					// Ostatni komplet

	if (wpts_set_no < last_set)
	{
		if ((cur_wpt + LIST_WPTS_LEN) < (uint16_t)(wpt_lst.size() - 1)) cur_wpt += LIST_WPTS_LEN;	// Nastepny komplet waypointow z listy
		else cur_wpt = wpt_lst.size() - 1;										// Albo ostatni waypoint
	}

	CTRL_OFF;																	// Skasuj MSB jako znacznik deaktywowanej kontrolki
	openScreen(screen, false);													// Przerysuj ekran
}
void btnPrevWpt(bool on_off)
{
	if (cur_wpt > 0) cur_wpt--;													// Poprzedni waypoint z listy
	else cur_wpt = wpt_lst.size() - 1;											// Ostatni waypoint

	CTRL_OFF;																	// Skasuj MSB jako znacznik deaktywowanej kontrolki
	openScreen(screen, false);													// Przerysuj ekran
}
void btnNextWpt(bool on_off)
{
	if (cur_wpt < (wpt_lst.size() - 1)) cur_wpt++;								// Nastepny waypoint z listy
	else cur_wpt = 0;															// Pierwszy waypoint

	CTRL_OFF;																	// Skasuj MSB jako znacznik deaktywowanej kontrolki
	openScreen(screen, false);													// Przerysuj ekran
}
void btnThisWpt(bool on_off)
{
	SimpleList<wpt_t>::iterator itr = wpt_lst.begin() + cur_wpt;
	dest_wpt = *itr;
	debugI("Waypoint NAME: %s LAT:%.5f LON:%.5f", dest_wpt.name, dest_wpt.lat, dest_wpt.lon);
	screen = SCR_NAVI;															// Powrot do ekranu nawigacji
	CTRL_ON;																	// Ustaw MSB jako znacznik aktywowanej kontrolki
	openScreen(screen, true);
}
void btnCancelWpt(bool on_off)
{
	CTRL_OFF;																	// Skasuj MSB jako znacznik deaktywowanej kontrolki
	cur_wpt = 0;																// Listuj zawsze od pierwszego w nowo otwartym pliku
	screen = SCR_FILES;															// Powrot do ekranu z listingiem gpx
	openScreen(screen, false);
}
void satUpdateStats()
{
	/*
	GPS Satellites in view
	$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
	$GPGSV,3,2,11,14,25,170,00,16,57,208,39,18,67,296,40,19,40,246,00*74
	$GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D
	1    = Total number of messages of this type in this cycle
	2    = Message number
	3    = Total number of SVs in view
	4    = SV PRN number
	5    = Elevation in degrees, 90 maximum
	6    = Azimuth, degrees from true north, 000 to 359
	7    = SNR, 00-99 dB (null when not tracking)
	8-11 = Information about second SV, same as field 4-7
	12-15= Information about third SV, same as field 4-7
	16-19= Information about fourth SV, same as field 4-7
	*/
	static uint8_t slot = 0;
	static sat_t sats_temp[MAX_SATS];											// Tablica parametrow sygnalu z satelitow (tymczasowa)
	uint8_t totalMessages = atoi(totalGPGSVMessages.value());					// Calkowita ilosc sentencji GPGSV
	uint8_t currentMessage = atoi(messageNumber.value());						// Kolejny numer sentencji GPGSV

	// Cztery komplety danych w sentencji GPGSV lub mniej w ostatniej sekwencji
	for (uint8_t i = 0; currentMessage < totalMessages ? i < 4 : i < (atoi(satsInView.value()) % 4); ++i)
	{
		uint8_t sat_prn = atoi(satNumber[i].value());

		if (sat_prn)															// Jezeli jest satelita
		{
			sats_temp[slot].prn = sat_prn;										// Zapisz dane
			sats_temp[slot].elevation = atoi(elevation[i].value());
			sats_temp[slot].azimuth = atoi(azimuth[i].value());
			sats_temp[slot].snr = atoi(snr[i].value());

			// Jezeli SNR > 0, i ustawiony azymut i elewacja, to satelita jest sledzony (uzywany do fixa)
			if (sats_temp[slot].snr && sats_temp[slot].elevation && sats_temp[slot].azimuth) sats_temp[slot].active = true;
			else sats_temp[slot].active = false;

			slot++;																// Kolejny zestaw danych
		}
	}

	if (totalMessages == currentMessage)										// Jezeli otrzymano komplet sentencji GPGSV
	{
		memcpy(sats_stats, sats_temp, sizeof(sats_temp));						// Kopiuj do bufora
		sat_stats_ready = true;													// Statystyki aktualne

		for (uint8_t i = 0; i < MAX_SATS; ++i) sats_temp[i].prn = 0;			// Wyzeruj informacje o satelitach

		slot = 0;
	}
}
void renderWpt(uint16_t course, point_t xy, uint8_t shift, uint16_t color)
{
	static point_t wpt_xy;														// Wspolrzedne srodku kolka waypointa
	make_trt_mtx(xy, -course * 0.0175);											// Ujemne radiany
	tft.fillCircle(wpt_xy.x, wpt_xy.y, 3, TFT_BLACK);							// Zamaz kolko waypointa
	wpt_xy = mtx_mul_vec(*trt_mtx, (point_t) {xy.x, xy.y - shift});				// Oblicz polozenie srodka kolka waypointa
	tft.fillCircle(wpt_xy.x, wpt_xy.y, 3, color);								// Narysuj kolko waypointa
}
void renderCompassNeedle(uint16_t course, point_t xy, uint8_t r)
{
	static point_t n, s, w, e;
	static uint8_t old_r;
	make_trt_mtx(xy, -course * 0.0175);											// Ujemne radiany

	if (old_r == r)																// Jezeli to ten sam kompas
	{
		tft.fillTriangle(n.x, n.y, e.x, e.y, w.x, w.y, TFT_BLACK);				// Zamaz poprzednie wskazanie kompasu
		tft.fillTriangle(s.x, s.y, e.x, e.y, w.x, w.y, TFT_BLACK);
	}

	n = mtx_mul_vec(*trt_mtx, (point_t) {xy.x, xy.y - r + 2});					// Oblicz polozenie "igly kompasu"
	e = mtx_mul_vec(*trt_mtx, (point_t) {xy.x + (r / 5), xy.y});
	w = mtx_mul_vec(*trt_mtx, (point_t) {xy.x - (r / 5), xy.y});
	s = mtx_mul_vec(*trt_mtx, (point_t) {xy.x, xy.y + r - 2});
	tft.fillTriangle(n.x, n.y, e.x, e.y, w.x, w.y, TFT_RED);					// Nowe wskazanie kompasu (czerwona igla)
	tft.fillTriangle(s.x, s.y, e.x, e.y, w.x, w.y, TFT_WHITE);					// Biala igla
	old_r = r;
}
void renderScreenSaver()
{
	tft.setTextColor(random(65535), TFT_BLACK);
	tft.drawString("SkeletonDevices", random(100), random(120), random(6));
}
////////////////////////////////////////////////////////////////////////////////
// Macierze https://eduinf.waw.pl/inf/utils/002_roz/2008_21.php
////////////////////////////////////////////////////////////////////////////////
void make_trt_mtx(point_t xy, float phi)
{
	// Tworzy macierz rotacji wokol punktu (x, y)
	// phi - kat obrotu w radianach
	// [ cos(phi)                            sin(phi)         0 ]
	// [ -sin(phi)                           cos(phi)         0 ]
	// [ x(1-cos(phi))+y*sin(phi)  -x*sin(phi)+y(1-cos(phi))  1 ]
	// Dla phi=0 jest to macierz translacji
	// Dla x=y=0 jest to macierz obrotu wokol poczatku ukladu wspolrzednych
	trt_mtx[0][0] = cos(phi);
	trt_mtx[1][1] = trt_mtx[0][0];
	trt_mtx[0][1] = sin(phi);
	trt_mtx[1][0] = -trt_mtx[0][1];
	trt_mtx[2][0] = (xy.x * (1 - cos(phi))) + (xy.y * sin(phi));
	trt_mtx[2][1] = (-xy.x * sin(phi)) + (xy.y * (1 - cos(phi)));
}
point_t mtx_mul_vec(float mtx[], point_t xy)
{
	// Macierz B musi posiadac tyle wierszy, ile kolumn posiada macierz A.
	// Macierz wynikowa C posiada tyle wierszy, ile posiada macierz A oraz tyle kolumn, ile posiada macierz B
	//                           [1  2  3]
	// [x', y', 1] = [x, y, 1] * [4  5  6]
	//                           [7  8  9]
	// x' = x*1 + y*4 + 1*7
	// y' = x*2 + y*5 + 1*8
	// 1 = 1
	point_t point;
	point.x = xy.x * mtx[0] + xy.y * mtx[3] + mtx[6];
	point.y = xy.x * mtx[1] + xy.y * mtx[4] + mtx[7];
	return point;
}
String gpsFormatConvert(uint8_t deg, uint32_t frac, enum GPS_FORMATS fmt)
{
	// https://www.szukaj-trasy.com/wgs84.html
	// https://www.wspolrzedne-gps.pl/konwerter
	String result;
	char out[16];
	long unsigned f103 = frac / 10000;

	switch (fmt)
	{
		case GPS_SD:
			sprintf(out, "%d.%05lu%c", deg, f103, 247);							// Stopnie i ulamki stopnia
			break;

		case GPS_SMS:
			//sprintf(out, "%d%c%lu'%lu.%lu\"", deg, 42 /*247*/,
			//(f103 * 60) / 100000,												// Minuty
			//((f103 * 3600 / 100) - ((f103 * 60) / 100000) * 60 * 1000) / 1000,	// Sekundy
			//((f103 * 3600 / 100) - ((f103 * 60) / 100000) * 60 * 1000) % 1000);	// Ulamki sekund
			sprintf(out, "%d%c%02lu'%02lu.%03lu\"", deg, 42 /*247*/,
			        (f103 * 6) / 10000,													// Minuty
			        ((f103 * 36) - ((f103 * 6) / 10000) * 60000) / 1000,				// Sekundy
			        ((f103 * 36) - ((f103 * 6) / 10000) * 60000) % 1000);				// Ulamki sekund
			break;

		case GPS_SMD:
			sprintf(out, "%d%c%02lu.%04lu'", deg, 42 /*247*/,
			        (f103 * 6) / 10000,													// Minuty
			        (f103 * 6) - ((f103 * 6) / 10000) * 10000);							// Ulamki minut
			break;
	}

	result = String(out);
	return result;
}