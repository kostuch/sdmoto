#include <Arduino.h>
#include <Wire.h>																// I2C
#include <SPI.h>																// SPI
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
#include <TFT_eSPI.h>															// TFT (ST7735)
#include <RemoteDebug.h>														// https://github.com/JoaoLopesF/RemoteDebug
#include "sdmoto.h"																// Konfiguracja kompilacji

RemoteDebug Debug;																// Zdalny debug
PCF857x	pcf8575(0x20, &Wire);													// Ekspander PCF8574T
TFT_eSPI tft = TFT_eSPI();														// Wyswietlacz TFT
SdFat sd;																		// Karta SD
File dir;																		// Katalog na SD
File file;																		// Plik na SD
ESP8266WebServer web_server;													// Web server
WebConfig conf;																	// Konfigurator webowy
fs::File SPIFFS_file;															// Plik na SPIFFS

#define SD_CS_PIN 0 															// Fake value dla SdFat
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)							// Magia dla SdFat
void sdCsInit(SdCsPin_t pin) {}													// Inicjalizacja CS (Puste!)

void sdCsWrite(SdCsPin_t pin, bool level)
{
	pcf8575.write(SDC_CS_PIN, level);											// CS dla SDCard
}

void tft_cs(bool enabled)
{
	pcf8575.write(TFT_CS_PIN, !enabled);										// CS dla TFT
}

String SPIFFS_list(void);
bool handleFileRead(String path);
String getContentType(String filename);

String handleCalibration(void);
String HTMLHeader(void);
String HTMLFooter(void);
uint16_t kalibracja = 100;
WiFiEventHandler SAPstationConnectedHandler;
WiFiEventHandler SAPstationDisconnectedHandler;
WiFiEventHandler STAstationGotIPHandler;
WiFiEventHandler STAstationDisconnectedHandler;
WiFiEventHandler wifiModeChanged;

uint32_t czas = millis();
uint32_t test = millis();
uint16_t pwm_val = 0;

void setup()
{
	Serial.begin(115200);														// Init UART
	SPIFFS.begin();																// Init SPIFFS
	readConf();																	// Odczyt konfiguracji urzadzenia
	startWebServer();															// Start Serwera web
	Wire.setClock(400000);														// Predkosc I2C (Max!!!)
	Wire.begin(SDA_PAD, SCL_PAD);												// I2C
	pcf8575.begin();															// Inicjalizacja expandera
	pcf8575.write(MUX_PIN, LOW);												// Przelaczenie multipleksera
	setupPins();																// Ustawienie pinow GPIO
	i2c_scan();

	// ------------------------- SDCard
	if (!sd.begin(SD_CONFIG)) Serial.println("Karta SD sie zesrala.");
	else
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

	// ------------------------- TFT
	tft.init(NULL, tft_cs, NULL, NULL); // Init TFT (DC func, CS func, RST func, TOUCH_CS, CS via driver)
	tft.setRotation(1);
	tft.fillScreen(TFT_BLACK);
	tft.drawCentreString("Test ST7735!", tft.width() / 2, tft.height() / 2, 2);
	tft.drawCentreString(String(millis()), tft.width() / 2, tft.height() / 4, 2);
	// ------------------------- Remote debug
	Debug.begin(HOST_NAME);														// Init socketa
	Debug.setResetCmdEnabled(true);												// Reset dozwolony
	Debug.showProfiler(true);													// Profiler (pomiar czsasu)
	Debug.showColors(true);														// Kolorki
	Serial.println("Koniec SETUP!");
}

void loop()
{
	if (pcf_signal)																// Dla wykrycia pojedynczego wcisnienia/puszczenia
	{
		debugI("Przerwanie na PCF");
		uint8_t i2c_state = pcf8575.read8();
		debugI("Stan expandera: %d", i2c_state);
		pcf_signal = false;
		// Po ewentualnej obsludze przerwania
		attachInterrupt(I2C_IRQ_PIN, i2c_irq, FALLING);
	}

	if (imp_signal)
	{
		debugI("Przerwanie IMP");
		imp_signal = false;
		// Po ewentualnej obsludze przerwania
		attachInterrupt(IMP_IRQ_PIN, imp_irq, FALLING);
	}

	if ((millis() - test) > 100)												// DEBUG
	{
		analogWrite(BL_PIN, pwm_val);
		test = millis();

		if (pwm_val < 1023) pwm_val += 20;
		else pwm_val = 0;
	}

	if (connected) web_server.handleClient();

	Debug.handle();
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
	imp_signal = true;															// Flaga pojawienia sie impulsu
	detachInterrupt(IMP_IRQ_PIN);												// Odepnij przerwania, bo inaczej CRASH
}

ICACHE_RAM_ATTR void i2c_irq(void)
{
	pcf_signal = true;															// Flaga zdarzenia na ekspanderze
	detachInterrupt(I2C_IRQ_PIN);												// Odepnij przerwania na wszelki wypadek
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

/*
void base_version()
{
	Dir dir = SPIFFS.openDir("/");
	digitalWrite(BUILTIN_LED, LOW);
	File file = SPIFFS.open("/ver_1000.bin", "r");
	uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
	Serial.printf("max free sketchspace = %d\r\n", maxSketchSpace);

	if (!Update.begin(maxSketchSpace, U_FLASH))
	{
		//start with max available size
		Update.printError(Serial);
		Serial.println("ERROR");
	}

	while (file.available())
	{
		uint8_t ibuffer[128];
		file.read((uint8_t *)ibuffer, 128);
		Update.write(ibuffer, sizeof(ibuffer));
	}

	Serial.print("Koniec!");
	Serial.print(Update.end(true));
	digitalWrite(BUILTIN_LED, HIGH);
	file.close();
	ESP.restart();
}
 */

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

	if (wifi_multi.run() == WL_CONNECTED) connected = true;

	if (!connected)
	{
		IPAddress local_IP(10, 0, 0, 1);
		IPAddress gateway(10, 0, 0, 1);
		IPAddress subnet(255, 0, 0, 0);
		WiFi.softAPConfig(local_IP, gateway, subnet);
		//WiFi.mode(WIFI_AP);//192.168.4.1
		WiFi.softAP(conf.getApName(), conf.getValue("dev_pwd"));
		Serial.println(conf.getApName());
		Serial.println(WiFi.softAPIP());
		connected = true;
	}
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

void i2c_scan(void)
{
	uint8_t err, i2c_address;
	int nDevices;
	nDevices = 0;

	for (i2c_address = 1; i2c_address < 127; i2c_address++ )
	{
		Wire.beginTransmission(i2c_address);
		err = Wire.endTransmission();

		if (err == 0)
		{
			Serial.println("0x");

			if (i2c_address < 16)
				Serial.println("0");

			Serial.println(i2c_address, 16);
			Serial.println("  ");
			nDevices++;
		}
		else if (err == 4)
		{
			debugE("Unknow I2C error at address 0x");

			if (i2c_address < 16)
				Serial.println("0");

			Serial.println(i2c_address, 16);
		}
	}

	if (nDevices == 0) Serial.println("No I2C devices!");
}