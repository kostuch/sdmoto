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
//#include <TJpg_Decoder.h>														// Dekoder jpg
#include <TFT_eSPI.h>															// TFT (ST7735)
#include <RemoteDebug.h>														// https://github.com/JoaoLopesF/RemoteDebug
#include <SerialRAM.h>															// EERAM
#include "TinyGPS++.h"
#include "sdmoto.h"																// Konfiguracja kompilacji
#include "gui.h"																// Definicje GUI
#include "icons.h"																// Definicje ikon

Ticker				every_sec_tmr;												// Timer sekundowy
RemoteDebug			Debug;														// Zdalny debug
PCF857x				pcf8575(I2C_EXP_A, &Wire);									// Ekspander PCF8574T
SerialRAM			eeram;														// EERAM
TFT_eSPI 			tft = TFT_eSPI();											// Wyswietlacz TFT
TinyGPSPlus			gps;
SdFat				sd;															// Karta SD
File				dir;														// Katalog na SD
File				file;														// Plik na SD
ESP8266WebServer	web_server;													// Web server
WebConfig			conf;														// Konfigurator webowy
fs::File			SPIFFS_file;												// Plik na SPIFFS
WiFiEventHandler	SAPstationConnectedHandler;
WiFiEventHandler	SAPstationDisconnectedHandler;
WiFiEventHandler	STAstationGotIPHandler;
WiFiEventHandler	STAstationDisconnectedHandler;
WiFiEventHandler	wifiModeChanged;
SimpleList<btn_t>	ctrl_list;													// Lista kontrolek (przyciskow) na ekranie

#define SD_CS_PIN 0 															// Fake value dla SdFat
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)							// Magia dla SdFat

void sdCsInit(SdCsPin_t pin)
{}																				// Inicjalizacja CS (Puste!)

void sdCsWrite(SdCsPin_t pin, bool level)
{
	pcf8575.write(SDC_CS_PIN, level);											// CS dla SDCard
}

void tft_cs(bool enabled)
{
	pcf8575.write(TFT_CS_PIN, !enabled);										// CS dla TFT
}

uint16_t pwm_val = 0;
uint16_t kalibracja = 100;

void mux_switch(enum MUX_STATES state)
{
	pcf8575.write(MUX_PIN, state);
	mux_state = state;
}

void setup()
{
	Serial.begin(9600);															// Init UART (predkosc jak dla GPS)
	SPIFFS.begin();																// Init SPIFFS
	Wire.setClock(400000);														// Predkosc I2C (Max!!!)
	Wire.begin(SDA_PAD, SCL_PAD);												// Init I2C
	pcf8575.begin();															// Init PCF8574
	setupPins();																// Ustawienie pinow GPIO
	mux_switch(RUNTIME);														// Multiplekser w pozycji roboczej
	bootstrap();																// Test czy coś wcisniete przy uruchamianiu
	eeram.begin();																// Init EERAM
	eeram.setAutoStore(true);													// Automatyczne zapamietywanie RAM

	if (!sd.begin(SD_CONFIG)) Serial.println("Karta SD sie zesrala.");			// Init SDCard

	readConf();																	// Odczyt konfiguracji urzadzenia
	startWebServer();															// Start Serwera web
	tft.init(NULL, tft_cs, NULL, NULL);											// Init TFT (DC, CS, RST, TCS, CS via driver)
	//tft.setSwapBytes(true);
	analogWrite(BL_PIN, pwm_val);												// DEBUG
	tft.setRotation(1);
	tft.fillScreen(TFT_BLACK);
	tft.drawCentreString("Test ST7735!", tft.width() / 2, tft.height() / 2, 2);
	tft.drawCentreString(String(millis()), tft.width() / 2, tft.height() / 4, 2);
	// ------------------------- Remote debug
	Debug.begin(conf.getApName());												// Init socketa
	Debug.setResetCmdEnabled(true);												// Reset dozwolony
	Debug.showProfiler(true);													// Profiler (pomiar czsasu)
	Debug.showColors(true);														// Kolorki
	screen = (enum SCREENS) eeram.read(LAST_SCREEN);							// Ostatnio uzywany ekran
	renderToolbar(WIFI_XOFF);													// Ikona WiFi
	renderToolbar(GPS_NOFIX);													// Ikona GPS
	//renderToolbar(MEMORY);														// Ikona pamieci
	renderToolbar(SD_OK);														// Ikona karty SD
	renderToolbar(GPS_DATETIME);												// Czas
	openScr(screen);															// Otworz go
	every_sec_tmr.attach_ms(1000, everySecTask);								// Zadania do wykonania co sekunde
	Serial.println("Koniec SETUP!");
}

void loop()
{
	if (pcf_signal) btnCheck();													// Okresl stan przyciskow

	if (imp_signal)
	{
		debugI("Przerwanie IMP");
		// Po obsludze przerwania
		imp_signal = false;														// Przerwanie obsluzone
		attachInterrupt(IMP_IRQ_PIN, imp_irq, FALLING);
	}

	if (connected) web_server.handleClient();									// Obsluga Web serwera

	while (Serial.available()) gps.encode(Serial.read());						// Obsluga transmisji NMEA z GPS

	Debug.handle();																// Obsluga Remote Debug
}

void bootstrap()
{
	if (((pcf8575.read8() & 0xF8) ^ 0xF8) & (1 << BTN_RST))						// Jezeli wcisniety przycisk RST
	{
		Dir dir = SPIFFS.openDir("/");
		SPIFFS_file = SPIFFS.open("/firmware.bin", "r");						// Otworz plik z SPIFFS
		uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
		debugI("max free sketchspace = %d\r\n", maxSketchSpace);

		if (!Update.begin(maxSketchSpace, U_FLASH))								// Poczatek odtwarzania
		{
			//start with max available size
			//Update.printError(Serial);
			debugE("ERROR");
		}

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

void everySecTask()
{
	//analogWrite(BL_PIN, pwm_val);												// DEBUG

	//if (pwm_val < 1023) pwm_val += 200;
	//else pwm_val = 0;

	//uint16_t pomiar = analogRead(A0);
	//debugI("Napiecie (RAW): %d (REAL): %.1f", pomiar, (pomiar / 1024.0) * 22);
	if (gps.date.isUpdated())
	{
		tft.fillRect(TBARX_DATETIME, 0, 64, 8, TFT_BLACK);
		tft.setTextColor(TFT_GREEN);
		String x = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
		tft.drawString(x, TBARX_DATETIME, 0, 1);
		//tft.printf("%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
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
	analogWrite(BL_PIN, 0);														// TFT podswietlenie ON (max)
}

ICACHE_RAM_ATTR void imp_irq(void)
{
	pulses_cnt1++;																// Zwieksz liczniki impulsow
	pulses_cnt2++;
	pulses_spd++;																// Impulsy do pomiaru predosci
	imp_signal = true;															// Flaga pojawienia sie impulsu
	detachInterrupt(IMP_IRQ_PIN);												// Odepnij przerwania, bo inaczej CRASH
}

ICACHE_RAM_ATTR void i2c_irq(void)
{
	pcf_signal = true;															// Flaga zdarzenia na ekspanderze
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
	            "'name':'ssid',"
	            "'label':'Nazwa sieci WiFi',"
	            "'type':");
	params += String(INPUTTEXT);
	params += F(","
	            "'default':'Moje WiFi'"
	            "},"
	            "{"
	            "'name':'pwd',"
	            "'label':'Hasło WiFi',"
	            "'type':");
	params += String(INPUTPASSWORD);
	params += F(","
	            "'default':'moje_tajne_haslo'"
	            "},"
	            "{"
	            "'name':'imp_src',"
	            "'label':'Dane o dystansie',"
	            "'type':");
	params += String(INPUTRADIO);
	params += F(","
	            "'options':["
	            "{'v':'g','l':'GPS'},"
	            "{'v':'i','l':'Impulsator'}],"
	            "'default':'g'"
	            "},"
	            "{"
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

		debugI("Zaladowany plik %s o rozmiarze %d", filename.c_str(), upload.totalSize);
	}
}

String SPIFFS_list()
{
	fs::FSInfo fs_info;
	SPIFFS.info(fs_info);
	int32_t free_space = (fs_info.totalBytes - fs_info.usedBytes) / 1024;
	//String FileList = F("File List:\n");
	fs::Dir dir = SPIFFS.openDir("/");
	String page = HTMLHeader();
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

void onStationConnected(const WiFiEventSoftAPModeStationConnected &evt)
{
	Serial.print(F("Station connected: "));
	Serial.println(macToString(evt.mac));
}

void onStationDisconnected(const WiFiEventSoftAPModeStationDisconnected &evt)
{
	Serial.print(F("Station disconnected: "));
	Serial.println(macToString(evt.mac));
}

void onWiFiModeChanged(const WiFiEventModeChange &evt)
{
	Serial.printf_P(PSTR("WiFi mode changed to %d\r\n"), WiFi.getMode());
}

void onStationGotIP(const WiFiEventStationModeGotIP &evt)
{
	Serial.print(F("Net = "));
	Serial.println(evt.ip);
	Serial.println(evt.mask);
	Serial.println(evt.gw);
}

void onClientDisconnected(const WiFiEventStationModeDisconnected &evt)
{
	Serial.println("Disconnected from AP " + evt.ssid + " because of " + evt.reason);
}

void initWiFi()
{
	// AP mode
	SAPstationConnectedHandler = WiFi.onSoftAPModeStationConnected(&onStationConnected);
	SAPstationDisconnectedHandler = WiFi.onSoftAPModeStationDisconnected(&onStationDisconnected);
	// STA mode
	STAstationGotIPHandler = WiFi.onStationModeGotIP(&onStationGotIP);
	STAstationDisconnectedHandler = WiFi.onStationModeDisconnected(&onClientDisconnected);
	wifiModeChanged = WiFi.onWiFiModeChange(&onWiFiModeChanged);
	WiFi.persistent(false);														 // Don't save WiFi configuration in flash - optional
	ESP8266WiFiMulti wifi_multi;
	WiFi.mode(WIFI_STA);
	wifi_multi.addAP(conf.getValue("ssid"), conf.getValue("pwd"));
	uint8_t cnt = 0;

	while ((wifi_multi.run() != WL_CONNECTED) && (cnt < 20))
	{
		delay(500);
		Serial.print(".");
		cnt++;
	}

	Serial.println();

	if (wifi_multi.run() == WL_CONNECTED)
	{
		internet = true;
		WiFi.hostname(conf.getApName());
	}
	else
	{
		IPAddress local_IP(10, 0, 0, 1);
		IPAddress gateway(10, 0, 0, 1);
		IPAddress subnet(255, 0, 0, 0);
		WiFi.softAPConfig(local_IP, gateway, subnet);
		//WiFi.mode(WIFI_AP);//192.168.4.1
		WiFi.softAP(conf.getApName(), conf.getValue("dev_pwd"));
		Serial.println(conf.getApName());
		Serial.println(WiFi.softAPIP());
	}

	connected = true;
}

void update_started(void)
{
	web_server.sendHeader(F("Connection"), F("close"));
	web_server.sendHeader(F("Access-Control-Allow-Origin"), F("*"));
	web_server.send(200, F("text/plain"), F("Czekaj na zakonczenie update..."));
	Serial.println(F("Update started"));
}
void update_finished(void)
{
	Serial.println(F("\r\nUpdate finished"));
}

void update_progress(int cur, int total)
{
	Serial.printf_P(PSTR("HTTP update process at %d of %d bytes...\r"), cur, total);
}

void update_error(int err)
{
	Serial.printf_P(PSTR("HTTP update fatal error code %d\n"), err);
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
		String page = HTMLHeader();
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
	String page = HTMLHeader();
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
	conf.handleFormRequest(&web_server);
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
	initWiFi();
	web_server.onFileUpload(handleFileUpload);
	web_server.on(F("/"), handleRoot);
	web_server.on(F("/update"), handleFWUpdate);
	web_server.on(F("/update_ver"), HTTP_POST, handleFWUpdate2);
	web_server.on(F("/conf"), handleConf);
	web_server.on(F("/cal"), handleCalibration);
	web_server.on(F("/cal/cal_dist_plus"), []()
	{
		kalibracja++;
		web_server.sendHeader("Location", "/cal", true);
		web_server.send (302, "text/plain", "");
		//web_server.send(200, F("text/html"), handleCalibration());
	});
	web_server.on(F("/cal/cal_dist_minus"), []()
	{
		kalibracja--;
		web_server.sendHeader("Location", "/cal", true);
		web_server.send (302, "text/plain", "");
		//web_server.send(200, F("text/html"), handleCalibration());
	});
	web_server.on(F("/login"), HTTP_GET, handleLogin);
	web_server.on(F("/login_check"), HTTP_POST, handleLogin2);
	/*
	web_server.on(F("/send"), HTTP_GET, []()
	{
		web_server.sendHeader(F("Connection"), F("close"));
		web_server.sendHeader(F("Access-Control-Allow-Origin"), F("*"));
		web_server.send(200, F("text/html"), F("<form method='POST' action='/upload' "
		                                       "enctype='multipart/form-data'><input type='file' "
		                                       "name='update'><input type='submit' value='Zapisz plik'></form>"));
	});
	web_server.on(F("/upload"), HTTP_POST, []()
	{
		web_server.sendHeader(F("Connection"), F("close"));
		web_server.sendHeader(F("Access-Control-Allow-Origin"), F("*"));
		web_server.send(200, F("text/plain"), F("OK"));
	});
	*/
	web_server.on(F("/list"), HTTP_GET, SPIFFS_list);
	web_server.on(F("/list"), HTTP_POST, []()
	{
		if (web_server.hasArg("DELETE"))
		{
			SPIFFS.remove(web_server.arg("DELETE"));
			Serial.println("Usuniety plik " + web_server.arg("DELETE"));
		}

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
	//called when the url is not defined here
	//use it to load content from SPIFFS
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
	if (web_server.hasArg("SAVE")) Serial.println("SAVE");

	debugI("Nacisniety SAVE po raz %d", kalibracja);
	String page = HTMLHeader();
	page += F("<p><h3>Kalibracja metromierza</h3></p>\n");
	//Image to DataURL converter https://onlinecamscanner.com/image-to-dataurl
	//for (size_t i = 0; i < sizeof(obrazek); i++) page += char(pgm_read_byte(obrazek + i));
	//p += "<img src='/top.jpg' alt=''>";
	page += F("<p><a href = '/cal/cal_dist_plus'><button class='btn btn-primary btn-lg'>CAL +</button></a></p>\n"
	          "<p><a href = '/cal/cal_dist_minus'><button class='btn btn-secondary btn-lg'>CAL -</button></a></p>\n"
	          "<p><form action='/cal' method='POST'><button type='button submit' name='SAVE' value='1'"
	          "class='btn btn-success btn-lg'>Zapis</button></form></p>\n"
	          "<p><h1>");
	page += String(kalibracja) + F("</h1></p>");
	page += HTMLFooter();
	web_server.send(200, F("text/html"), page);
	return page;
}
String HTMLHeader()
{
	String h = F("<!DOCTYPE html>\n"
	             "<html>\n"
	             "<head>\n"
	             //"<link href='/favicon.ico' rel='icon' type='image/x-icon' />"
	             "<title>Centrum kontroli nad wszechświatem</title>\n"
	             "<meta charset='utf-8'>\n"
	             "<meta name='viewport' content='width=device-width, initial-scale=1'>\n"
	             "<link rel='stylesheet' href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.4/css/bootstrap.min.css' >\n"
	             "</head>\n"
	             //"<body style='text-align: center;color: white; background: black;font-size: 1.5em;'>\n");
	             "<body style='text-align: center;color: white; background: black;font-size: 1.5em; "
	             "background-size: 100% 100vh; background-image: url(/logo_content.jpg)'>\n");
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
	tft.pushImage(x, y, w, h, bitmap);
	// This might work instead if you adapt the sketch to use the Adafruit_GFX library
	// tft.drawRGBBitmap(x, y, bitmap, w, h);
	// Return 1 to decode next block
	return 1;
}

/* void welcomeScreen()
{
	TJpgDec.setJpgScale(1);
	TJpgDec.setSwapBytes(true);
	TJpgDec.setCallback(tftImgOutput);
	TJpgDec.drawFsJpg(0, 0, "/skeleton.jpg");
	tft.printf("FW: %d", FW_VERSION);
	tft.printf("HW: %d.%d", HW_MAJOR_VER, HW_MINOR_VER);
}
 */

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

	pcf_signal = false;															// Przycisk obsluzony
}

void keyShortPress(enum BUTTONS button)
{
	debugI("Krotkie nacisniecie %d", button);

	switch (button)
	{
		case BTN_RELEASED:
			break;

		case BTN_RST:
			switch (screen)
			{
				case SCR_DIST:
					if (conf.getInt("imp_src") == 1) gps_dist1 = 0;				// Skasuj dystans odcinka
					else pulses_cnt1 = 0;

					break;

				case SCR_TIME:
					if (btn_mode == CHG_SCR)									// W trybie zmiany ekranu
					{
						if (timer_state == TMR_STOP)
						{
							timer_state = TMR_RUN;								// Uruchom stoper
							current_time = 0;									// Wartosc poczatkowa
						}
						else
						{
							timer_state = TMR_STOP;								// Zatrzymaj stoper
							//save_time();										// Zanotuj miedzyczas
							current_time = 0;									// Skasuj czas
						}
					}
					else 														// W trybie menu
					{
						// obsluga kasowania miedzyczasow
					}

					break;

				case SCR_NAVI:
					if (btn_mode == CHG_SCR)									// W trybie zmiany ekranu
					{
						if (conf.getInt("imp_src") == 1) gps_dist1 = 0;			// Skasuj dystans odcinka
						else pulses_cnt1 = 0;
					}
					else 														// W trybie zmiany kontrolki
					{
						/* 							if (ctl_pos[SCR_NAVI] & (1 << NAVI_SAVE_TRK))			// Kontrolka "Zapisuj slad"
													{
														if (!(ctl_pos[SCR_NAVI] & (1 << 7)))				// MSB = 0 (kontrolka nieaktywna)
														{
															ctl_pos[SCR_NAVI] |= 0x80;						// Ustaw MSB
															// Zmien wyglad kontrolki
															// Obsluga zapisu sladu do gpx
														}
														else 												// MSB = 1 (kontrolka aktywna)
														{
															ctl_pos[SCR_NAVI] &= 0xEF;						// Skasuj MSB
															// Zmien wyglad kontrolki
															// Zatrzymaj zapis sladu w gpx
														}

														return;
													}
						 */
						/*							if (ctl_pos[SCR_NAVI] & (1 << NAVI_SAVE_WPT))			// Kontrolka "Zapisuj waypointy"
													{
														if (!(ctl_pos[SCR_NAVI] & (1 << 7)))				// MSB = 0 (kontrolka nieaktywna)
														{
															ctl_pos[SCR_NAVI] |= 0x80;						// Ustaw MSB
															// Zmien wyglad kontrolki
															// Obsluga zapisu waypointow do gpx
														}
														else 												// MSB = 1 (kontrolka aktywna)
														{
															ctl_pos[SCR_NAVI] &= 0xEF;						// Skasuj MSB
															// Zmien wyglad kontrolki
															// Zatrzymaj zapis waypointow w gpx
														}

														return;
													}
						 */
						/* 							if (ctl_pos[SCR_NAVI] & (1 << NAVI_TO_WPT))				// Kontrolka "Nawiguj do waypointow"
													{
														if (!(ctl_pos[SCR_NAVI] & (1 << 7)))				// MSB = 0 (kontrolka nieaktywna)
														{
															ctl_pos[SCR_NAVI] |= 0x80;						// Ustaw MSB
															// Zmien wyglad kontrolki
															// Obsluga nawigacji
														}
														else 												// MSB = 1 (kontrolka aktywna)
														{
															ctl_pos[SCR_NAVI] &= 0xEF;						// Skasuj MSB
															// Zmien wyglad kontrolki
															// Zatrzymaj nawigacje po waypointach
														}

														return;
													}
						 */
					}

				case SCR_COMBO:
					if (btn_mode == CHG_SCR)									// W trybie zmiany ekranu
					{
						if (conf.getInt("imp_src") == 1) gps_dist1 = 0;			// Skasuj dystans odcinka
						else pulses_cnt1 = 0;
					}
					else 														// W trybie zmiany kontrolki
					{
						/* 							if (ctl_pos[SCR_COMBO] & (1 << COMBO_CAL_DIST))
													{
														if (!(ctl_pos[SCR_COMBO] & (1 << 7)))				// MSB = 0 (kontrolka nieaktywna)
														{
															ctl_pos[SCR_COMBO] |= 0x80;						// Ustaw MSB
															// Zmien wyglad kontrolki
														}
														else 												// MSB = 1 (kontrolka aktywna)
														{
															//if (changed_dist_cal)
															{
																eeram_save16(DIST_CAL, calibrations.dist_cal);
																// Pokaz komunikat
															}

															ctl_pos[SCR_COMBO] &= 0xEF;						// Skasuj MSB
															// Zmien wyglad kontrolki
														}

														return;
													}
						 */
						/* 							if (ctl_pos[SCR_COMBO] & (1 << COMBO_CAL_VOLT))
													{
														if (!(ctl_pos[SCR_COMBO] & (1 << 7)))				// MSB = 0 (kontrolka nieaktywna)
														{
															ctl_pos[SCR_COMBO] |= 0x80;						// Ustaw MSB
															// Zmien wyglad kontrolki
														}
														else 												// MSB = 1 (kontrolka aktywna)
														{
															//if (changed_volt_cal)
															{
																eeram_save16(VOLT_CAL, calibrations.volt_cal);
																// Pokaz komunikat
															}

															ctl_pos[SCR_COMBO] &= 0xEF;						// Skasuj MSB
															// Zmien wyglad kontrolki
														}

														return;
													}
						 */
						/* 							if (ctl_pos[SCR_COMBO] & (1 << COMBO_CAL_TEMP))
													{
														if (!(ctl_pos[SCR_COMBO] & (1 << 7)))				// MSB = 0 (kontrolka nieaktywna)
														{
															ctl_pos[SCR_COMBO] |= 0x80;						// Ustaw MSB
															// Zmien wyglad kontrolki
														}
														else 												// MSB = 1 (kontrolka aktywna)
														{
															if (changed_volt_cal)
															{
																eeram_save16(TEMP_CAL, calibrations.temp_cal);
																// Pokaz komunikat
															}

															ctl_pos[SCR_COMBO] &= 0xEF;						// Skasuj MSB
															// Zmien wyglad kontrolki
														}

														return;
													}
						 */
					}

					break;

				case SCR_GPS:
					break;
			}

			break;

		case BTN_UP:
		case BTN_RT:
			if (btn_mode == CHG_SCR) nextScr();									// Nastepny ekran
			else 																// Nastepna kontrolka
			{
				switch (screen)
				{
					case SCR_NAVI:
						//if (ctl_pos[SCR_NAVI] < 4) ctl_pos[SCR_NAVI] <<= 1;		// Poprzedni id przycisku
						//else ctl_pos[SCR_NAVI] = 1;								// lub od konca
						break;

					case SCR_COMBO:
						//if (ctl_pos[SCR_COMBO] < 4) ctl_pos[SCR_COMBO] <<= 1;	// Poprzedni id przycisku
						//else ctl_pos[SCR_COMBO] = 1;							// lub od konca
						break;

					default:
						break;
				}
			}

			break;

		case BTN_DN:
		case BTN_LT:
			if (btn_mode == CHG_SCR) prevScr();									// Poprzedni ekran
			else 																// Poprzednia kontrolka
			{
				// MSB - kontrolka aktywowana
				// 0000.0100
				// 0000.0010
				// 0000.0001
				switch (screen)
				{
					case SCR_NAVI:
						//if (ctl_pos[SCR_NAVI] > 1) ctl_pos[SCR_NAVI] >>= 1;		// Kolejny id przycisku
						//else ctl_pos[SCR_NAVI] = 4;								// lub od poczatku
						break;

					case SCR_COMBO:
						//if (ctl_pos[SCR_COMBO] > 1) ctl_pos[SCR_COMBO] >>= 1;	// Kolejny id przycisku
						//else ctl_pos[SCR_COMBO] = 4;							// lub od poczatku
						break;

					default:
						break;
				}
			}

			break;
	}
}

void keyLongPress(enum BUTTONS button)
{
	debugI("DLUGIE nacisniecie %d", button);

	switch (button)
	{
		case BTN_RELEASED:
			break;

		case BTN_RST:
			switch (screen)
			{
				case SCR_DIST:
				case SCR_NAVI:
				case SCR_COMBO:
					if (conf.getInt("imp_src") == 1) gps_dist2 = 0;				// Skasuj dystans odcinka
					else pulses_cnt2 = 0;

					break;

				case SCR_TIME:
				case SCR_GPS:
					break;
			}

			break;

		case BTN_UP:
		case BTN_RT:
		case BTN_DN:
		case BTN_LT:
			/* if (ctrl_list.size())												// Jezeli dla ekranu sa przyciski
			{
				if (btn_mode == CHG_SCR) btn_mode = CHG_CTRL;					// Zmien tryb zmiany ekranu/kontrolki
				else btn_mode = CHG_SCR;
			}
			*/
			break;
	}
}

void prevScr()
{
	// DIST
	// TIME
	// NAVI
	// COMBO
	// GPS
	switch (screen)
	{
		case SCR_DIST:
			if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;

			break;

		case SCR_TIME:
			if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;

			break;

		case SCR_NAVI:
			if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;

			break;

		case SCR_COMBO:
			if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;

			break;

		case SCR_GPS:
			if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;

			break;
	}

	openScr(screen);
}

void nextScr()
{
	// DIST
	// TIME
	// NAVI
	// COMBO
	// GPS
	switch (screen)
	{
		case SCR_DIST:
			if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;

			break;

		case SCR_TIME:
			if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;

			break;

		case SCR_NAVI:
			if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;
			else if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;

			break;

		case SCR_COMBO:
			if (conf.getInt("scr_gps") == 1) screen = SCR_GPS;
			else if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;

			break;

		case SCR_GPS:
			if (conf.getInt("scr_dist") == 1) screen = SCR_DIST;
			else if (conf.getInt("scr_time") == 1) screen = SCR_TIME;
			else if (conf.getInt("scr_navi") == 1) screen = SCR_NAVI;
			else if (conf.getInt("scr_combo") == 1) screen = SCR_COMBO;

			break;
	}

	openScr(screen);
}

void openScr(enum SCREENS scr)
{
	if (closeScr) closeScr();													// Wykonaj zamkniecie poprzedniego ekranu (jesli ustawione)

	eeram.write(LAST_SCREEN, screen);											// Zapamietaj aktualny ekran
	screen_t screen_buf;
	memcpy_P(&screen_buf, &screen_data[screen], sizeof(screen_buf));			// Kopiuj opis z tabeli do bufora

	if (screen_buf.scr_close_exe) closeScr = screen_buf.scr_close_exe;			// Jezeli jest funkcja zamkniecia ekranu, to ustaw wskaznik
	else closeScr = NULL;

	if (screen_buf.scr_open_exe) screen_buf.scr_open_exe();						// Uruchom funkcje skojarzona z otwarciem nowego ekranu

	ctrl_list.clear();															// Wyczysc stara liste przyciskow

	// Jezeli sa jakies przyciski, to utworz z nich liste
	for (uint8_t btn = 0; btn < sizeof(ctrls_data) / sizeof(btn_t); btn++)		// Przejrzyj wszystkie przyciski we FLASH
	{
		memcpy_P(&key_buf, &ctrls_data[btn], sizeof(key_buf));					// Kopiowanie definicji przycisku z FLASH do RAM

		if (key_buf.screen_id == screen) ctrl_list.push_back(key_buf);			// Dodaj przycisk do listy jezeli nalezy do ekranu
	}

	renderScreen(screen);
	/* 	// DEBUG
		SimpleList<btn_t>::iterator idx = ctrl_list.begin();
		debugI("Lista: %d", ctrl_list.size());

		for (uint8_t i = 0; i < ctrl_list.size(); i++, idx++)
			debugI("Scr: %d Key_id: %d Label: %s", idx->screen_id, idx->key_id, idx->lbl);
		//
	 */
}

void renderScreen(enum SCREENS scr)
{
	debugI("Ekran: %d Przyciskow: %d", scr, ctrl_list.size());
	tft.fillRect(0, 32, 160, 96, TFT_BLACK);									// Wyczysc ekran poza toolbarem
	tft.drawRect(0, 32, 160, 96, TFT_YELLOW);									// Ramka - sygnalizuje przelaczanie ekranow
	tft.setTextColor(TFT_WHITE);
	tft.drawCentreString("Ekran " + String(screen), tft.width() / 2, tft.height() / 2, 2);
}

void renderToolbar(enum TOOLBAR_ITEMS item)
{
	String x;
	switch (item)
	{
		case WIFI_XOFF:
			tft.drawBitmap(TBARX_WIFI, 0, wifi_sym, 32, 32, TFT_LIGHTGREY);
			break;

		case WIFI_XAP:
			break;

		case WIFI_XSTA:
			break;

		case GPS_DATETIME:
			tft.setTextColor(TFT_LIGHTGREY);									// Wyswietl czas na szaro
			//tft.printf("%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
			x = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
			tft.drawString(x, TBARX_DATETIME, 0, 1);
			break;

		case GPS_FIX:
			tft.drawBitmap(TBARX_GPS, 0, satellite_sym, 32, 32, TFT_GREEN);
			break;

		case GPS_NOFIX:
			tft.drawBitmap(TBARX_GPS, 0, satellite_sym, 32, 32, TFT_RED);
			break;

		case MEMORY:
			tft.drawBitmap(TBARX_MEMORY, 0, memory_sym, 32, 32, TFT_YELLOW);
			break;

		case SD_OK:
			tft.drawBitmap(TBARX_SD, 0, sd_sym, 32, 32, TFT_YELLOW);
			break;

		case SD_NOOK:
			break;

		case SD_OFF:
			break;

		default:
			break;
	}
}
