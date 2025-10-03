/* WSPR beacon with WM-386 (ESP6288 clone),SI5351, SSD1306 0.91, TinyGSP" 

Verion 1.0
Modified and improved script from ESP_WSPR sketch from PD0GB and PA3FBX
  - Web interface for configuration panel
  - Calibration also via the web interface
  - SSD1306 info panel texts completely revised
  - Extra debug messages via Serial
  - WSPR interval configurable via the web interface
  - SI5351 CLK1 fixed at ~10 MHz continuously for stabilization
  - Warmup period removed and running CLK2 continue with 10MHz
  - Cleanup of unused code and variables
  - GPS support on PIO D3 (can conflict during boot / reset)

Using a WM-386 (an ESP6288 clone with limited GPIO pins).
With this, we can build a WSPR transmitter with:
  - Up to 3 bands with LPF switching 
  - Use the buildin led - D4 PIO for indicating Webinterface active
  - Use a TinyGPS on PIO D3 
  - Through WiFi sync with NTP and provide a configuration possibility

Time synchronization via WiFi NTP server or optional GPS is selectable

Hardware: Wemos D1 Mini, Si5351 module, 128x64 I2C OLED display, a few LEDs (optional)  

On first startup, if no WiFi credentials are stored, the ESP will switch into Access Point mode.  
SSID = "WSPR_ConfigAP"  no password.  

Connect your phone/laptop to this network the portal is available at 192.168.4.1  
A configuration page appears after can enter or select the avilable WiFi SSID and enter the password.
Click “Save”. The ESP will then connect to your local WiFi.  
 
The unit will restat an show the local WiFi IP adress on the SSD1306 screen during booting.
Log in through your home network to enter the WSPR credetials and parameters:
  - Callsign
  - Locator
  - dBm (output power setting)
  - Calibration factor  (update through calibration routine)
  - Transmission interval in minutes after start of message or cycle
  - GPS enable/disable
  - Band mask x y z selecting 40 meter, 20 meter, 10 meter or multiple
  - Reset WiFi - set to 9 to reseter the local WiFi SSID and Password 
You can also switch the unit into Calibration or Test mode.


//Used libraries  
Etherkit Si5351 by Jason Mildrum, version 2.2.0  
Etherkit JTEncode by Jason Mildrum, version 1.3.1  
NTPtimeESP  
TimeLib by Michael Margolis, version 1.6.1  
WiFiManager by tzapu, version 2.0.17  
Adafruit GFX Library by Adafruit, version 1.12.1  
Adafruit SSD1306 by Adafruit, version 2.5.15  
Adafruit BusIO (I2C)  
EspSoftwareSerial by Dirk Kaar, version 8.1.0  
TinyGPSPlus by Mikal Hart, version 1.0.3  
Maidenhead by Mateusz Salwach, version 1.0.1  

Version: 2025-10-03
by Johan, PA3ANG
https://github.com/pa3ang

*/


#include <si5351.h>
#include "Wire.h"
#include <JTEncode.h>
#include <int.h>
#include <TimeLib.h>
#include <NTPtimeESP.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Maidenhead.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// OLED SSD1306
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int show_OLED_delay = 1000;

// WSPR timing
#define TONE_SPACING 146
#define WSPR_DELAY 683
#define SYMBOL_COUNT WSPR_SYMBOL_COUNT

// ESP-WSPR LPF pins
#define TX_band1_LED_PIN D7   // green from pin D7 to signal TX_band1 transmittion.    40meter
#define TX_band2_LED_PIN D6   // yellow LED from pin D6 to signal TX_band3 transmittion. 20meter
#define TX_band3_LED_PIN D5   // red LED from pin D5 to signal TX_band3 transmittion.  10meter

// NTP server
#define SEND_INTV 10
#define RECV_TIMEOUT 10
#define TIME_ZONE -1.0f       // Voor NL

// define TinyGPS pins
#define RX_pin D3             // input voor GPS TX.
#define TX_pin D0             // input voor GPS TX.

// EEPROM definition
#define EEPROM_SIZE 512
struct Config {
  char call[10];
  char loc[10];
  int dbm;
  int cal;
  int gps;
  int band;
  int oled;
  int interval;
  int ssid;
} config;

// Common variables
const char ver[10]        = "ANG-V1.0";
const char* WiFi_hostname = "ESP-WSPR";
const char* NTP_Server    = "time.google.com";
char timeStr[9];
char dateStr[11];


// WSPR and state-machine parameters
int bandStep = 0;               // waar zitten we in de cycle
int startMinute = -1;           // minuut waarop slot start
const uint32_t band1            = 7040100ULL;     // 40m band frequency op 1 HZ nauwkeurig
const uint32_t band2            = 14097100ULL;    // 20m band frequency op 1 HZ nauwkeurig
const uint32_t band3            = 28126200ULL;    // 10m band frequency op 1 HZ nauwkeurig
uint32_t freq;                  // freq for encode() routine     
uint32_t rx_freq;
const uint32_t calibration_freq = 1000000000ULL;  // 10 MHz, in hundredths of hertz
int32_t cal_factor; 
bool calibrate = false;
uint8_t tx_buffer[SYMBOL_COUNT];

// parameters for LPF switching
const int bandLedPins[] = { TX_band1_LED_PIN, TX_band2_LED_PIN, TX_band3_LED_PIN };
const uint32_t bandFreqs[]   = { band1, band2, band3 };

// TinyGPS
static const uint32_t GPSBaud   = 9600;
int Hour, Minute, Second, Second_oud, Day, Month, Year;
int Time_set;
int Sat;

// define hardare / software
Si5351 si5351;
JTEncode jtencode;

TinyGPSPlus gps;
SoftwareSerial ss(RX_pin, TX_pin);

NTPtime NTPch(NTP_Server);
strDateTime dateTime;

ESP8266WebServer server(80);
WiFiManager wm;

// Generic OLED display routine
void showOLED(String line1, String line2 = "", String line3 = "") {
    display.clearDisplay();
    display.setRotation(config.oled);
    display.setTextSize(2);  // altijd textSize 2
    display.setTextColor(SSD1306_WHITE);

    int lineHeight = 8 * 2;      // hoogte per regel
    int extraSpacing = 2;        // standaard extra spacing

    // Bepaal hoeveel regels er daadwerkelijk gebruikt worden
    int lineCount = 1;
    if (line2.length() > 0) lineCount++;
    if (line3.length() > 0) lineCount++;

    // Als er maar 2 regels zijn, iets meer ruimte ertussen
    if (lineCount == 2) extraSpacing = 6;

    int totalHeight = lineCount * lineHeight + (lineCount - 1) * extraSpacing;
    int yStart = (SCREEN_HEIGHT - totalHeight) / 2;

    int16_t x1, y1; uint16_t w, h;

    // regel 1
    display.getTextBounds(line1, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, yStart);
    display.println(line1);

    // regel 2
    if (lineCount >= 2) {
        display.getTextBounds(line2, 0, 0, &x1, &y1, &w, &h);
        display.setCursor((SCREEN_WIDTH - w) / 2, yStart + lineHeight + extraSpacing);
        display.println(line2);
    }

    // regel 3
    if (lineCount == 3) {
        display.getTextBounds(line3, 0, 0, &x1, &y1, &w, &h);
        display.setCursor((SCREEN_WIDTH - w) / 2, yStart + 2 * (lineHeight + extraSpacing));
        display.println(line3);
    }

    display.display();
}

// EEPROM configuration and routines
void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, config);
  EEPROM.end();
  // als EEPROM leeg of ongeldige waarden bevat → defaults zetten
  if (strlen(config.call) == 0 || config.dbm == -1) {
    strcpy(config.call, "NOCALL");
    strcpy(config.loc, "AA00aa");
    config.dbm = 23;
    config.cal = 0;
    config.gps = 0;
    config.band = 0;
    config.oled = 0;
    config.interval = 0;
    config.ssid = 0;
  }
}

void saveConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(0, config);
  EEPROM.commit();
  EEPROM.end();
}

// Webpage for configuration
String htmlForm() {
  String html = "<!DOCTYPE html>";
  html += "<html lang='en'><head>";
  html += "<meta name='format-detection' content='telephone=no'>";
  html += "<meta charset='UTF-8'>";
  html += "<meta  name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'/>";
  html += "<title>ESP-WSPR</title>";

  // Anti-cache via HTML (voor de browser)
  html += "<meta http-equiv='Cache-Control' content='no-cache, no-store, must-revalidate'>";
  html += "<meta http-equiv='Pragma' content='no-cache'>";
  html += "<meta http-equiv='Expires' content='0'>";
  
  html += "<style>body.c{align-self:auto;max-width:600px;margin:0 auto;text-align:center;font-family:verdana}div,input,select{padding:5px;font-size:1em;margin:5px 0;box-sizing:border-box}";
  html += "input,button,select,.msg{border-radius:.3rem;width: 100%}input[type=radio],input[type=checkbox]{width:auto}";
  html += "button,input[type='button'],input[type='submit']{cursor:pointer;border:0;background-color:#1fa3ec;color:#fff;line-height:2.4rem;font-size:1.2rem;width:100%}";
  html += "input[type='file']{border:1px solid #1fa3ec}";
  html += ".wrap {text-align:left;display:inline-block;min-width:260px;max-width:500px}";
  html += "</style></head><body class='c'><div class='wrap'>";

  html += "<h1>WSPR Config</h1>";

  html += "<form method='POST' action='/save'>";
  html += "Callsign<br><input name='call' value='" + String(config.call) + "' oninput='this.value = this.value.toUpperCase();'><br>";
  html += "Locator<br>  <input name='loc' value='" + String(config.loc)  + "' oninput='this.value = this.value.toUpperCase();'><br>";
  html += "dBm<br><input name='dbm' value='" + String(config.dbm) + "'><br>";
  html += "Calibration factor<br><input name='cal' value='" + String(config.cal) + "'><br>";
  html += "Interval (2,4,6,8,10,12,14,16) minutes<br><input name='interval' value='" + String(config.interval) + "'><br>";
  String maskStr = "";
  for (int i = 2; i >= 0; i--) {
    maskStr += (config.band & (1 << i)) ? '1' : '0';
    if (i > 0) maskStr += " ";  // spatie tussen bits, maar niet na de laatste
  } 
  html += "Bandmask (xyz (0/1) x=40m,y=20m,z=10m)<br><input name='band' value='" + maskStr + "'><br>";
  html += "GPS (0/1)<br><input name='gps' value='" + String(config.gps) + "'><br>";
  html += "OLED rotate (0/2)<br><input name='oled' value='" + String(config.oled) + "'><br>";
  html += "Reset WiFi (9)<br><input name='ssid' value='" + String(config.ssid) + "'><br>";
  html += "<input type='submit' value='Save'></form><br>";

  html += "<form method='POST' action='/testloop'>";
  html += "<input type='submit' value=Testloop></form><br>";

  html += "<form method='POST' action='/calibrate'>";
  html += "<input type='submit' value='Calibrate'></form>";

  html += "</body></html>";
  return html;
}

// Root page
void handleRoot() {
  server.send(200, "text/html", htmlForm());
}

// Save page
void handleSave() {
  // read values from form
  strncpy(config.call, server.arg("call").c_str(), sizeof(config.call));
  strncpy(config.loc, server.arg("loc").c_str(), sizeof(config.loc));
  config.dbm      = server.arg("dbm").toInt();
  config.cal      = server.arg("cal").toInt();
  config.gps      = server.arg("gps").toInt();
  String input    = server.arg("band");
  input.replace(" ", ""); config.band = strtoul(input.c_str(), NULL, 2);
  config.oled     = server.arg("oled").toInt();
  config.interval = server.arg("interval").toInt();
  config.ssid     = server.arg("ssid").toInt();
  // intercept wrong values
  if ((config.oled != 0) && (config.oled != 2))  { config.oled = 0; } 
  if (config.ssid != 9) { config.ssid = 0; } 
  if ((config.gps  != 0) && (config.gps != 1 ))  { config.gps  = 0; }
  if ((config.band  < 1) || (config.band > 7 ))  { config.band = 2; }
  // check is minimal interval is correct in combination with the selected number of bands and is figure is even.
  int min_interval = (config.band == 3 || config.band == 5 || config.band == 6) ? 4 : (config.band == 7) ? 6 : 2;
  if (config.interval < min_interval || (config.interval % 2 != 0)) config.interval = min_interval;
  else if (config.interval > 16) config.interval = 16;

  saveConfig();

  server.send(200, "text/html", "<h1>Saved! Rebooting...</h1>");
  delay(1000);
  ESP.restart();
}

// Calibration page
String htmlCalibrate() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>ESP-WSPR Calibrate</title>";
  html += "<style>body{font-family:verdana;text-align:center;}button{margin:5px;padding:10px;font-size:1.2em;}</style>";
  html += "</head><body><h1>Calibrate</h1>";
  html += "<p>Target frequency: " + String(calibration_freq/1000.0,3) + " kHz</p>";
  html += "<p>Current cal_factor: " + String(cal_factor) + "</p>";

  html += "<form action='/calibrate' method='POST'>";
  html += "<button name='step' value='r'>+0.01 Hz</button>";
  html += "<button name='step' value='f'>-0.01 Hz</button><br>";
  html += "<button name='step' value='t'>+0.1 Hz</button>";
  html += "<button name='step' value='g'>-0.1 Hz</button><br>";
  html += "<button name='step' value='y'>+1 Hz</button>";
  html += "<button name='step' value='h'>-1 Hz</button><br>";
  html += "<button name='step' value='u'>+10 Hz</button>";
  html += "<button name='step' value='j'>-10 Hz</button><br>";
  html += "<button name='step' value='i'>+100 Hz</button>";
  html += "<button name='step' value='k'>-100 Hz</button><br>";
  html += "<button name='step' value='o'>+1kHz</button>";
  html += "<button name='step' value='l'>-1kHz</button><br>";
  html += "<button name='step' value='p'>+10kHz</button>";
  html += "<button name='step' value=';'>-10kHz</button><br><br>";
  html += "<button name='step' value='save'>Save & Reboot</button>";
  html += "</form>";

  html += "</body></html>";
  return html;
}

// Calibration routine live interaction with webpage
void handleCalibrate() {
  static bool cal_init = false;

  if (!cal_init) {
    
    // start rx_freq op calibration_freq als nog niet ingesteld
    if (rx_freq == 0) rx_freq = calibration_freq;

    // bereken cal_factor op basis van config.cal
    cal_factor = (int32_t)(calibration_freq - rx_freq) + config.cal;

    si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351.set_freq(calibration_freq, SI5351_CLK0);
    si5351.set_clock_pwr(SI5351_CLK0, 1);
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);

    // LED / OLED update
    digitalWrite(TX_band1_LED_PIN, LOW); digitalWrite(TX_band2_LED_PIN, HIGH); digitalWrite(TX_band3_LED_PIN, LOW);
  
    showOLED("TX ACTIVE", String(calibration_freq),"CALIBRATE");
    calibrate= true;
    cal_init = true;
  }

  // POST? verwerk knop-actie
  if (server.method() == HTTP_POST && server.hasArg("step")) {
    String step = server.arg("step");

    if (step == "save") {
      // schrijf cal_factor naar config.cal en sla op
      config.cal = cal_factor;
      saveConfig();

      server.send(200, "text/html", "<h1>Saved! Rebooting...</h1>");
      delay(1000);
      ESP.restart();
      return;
    }

    // pas rx_freq aan volgens knop
    char c = step.length() ? step.charAt(0) : 0;
    int32_t stepVal = 0;
    switch (c) {
      case 'r': stepVal = 1; break;
      case 'f': stepVal = -1; break;
      case 't': stepVal = 10; break;
      case 'g': stepVal = -10; break;
      case 'y': stepVal = 100; break;
      case 'h': stepVal = -100; break;
      case 'u': stepVal = 1000; break;
      case 'j': stepVal = -1000; break;
      case 'i': stepVal = 10000; break;
      case 'k': stepVal = -10000; break;
      case 'o': stepVal = 100000; break;
      case 'l': stepVal = -100000; break;
      case 'p': stepVal = 1000000; break;
      case ';': stepVal = -1000000; break;
    }

    rx_freq += stepVal;
    int32_t diff = (int32_t)(calibration_freq - rx_freq);
    cal_factor = diff + config.cal;

    // pas direct toe op Si5351
    si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351.pll_reset(SI5351_PLLA);
    si5351.set_freq(calibration_freq, SI5351_CLK0);
  
    // debug
    Serial.println(F("--- step applied ---"));
    Serial.print(F("step char: ")); Serial.println(c);
    Serial.print(F("rx_freq: ")); Serial.println(rx_freq);
    Serial.print(F("cal_factor: ")); Serial.println(cal_factor);
  }

  // show to the webinterface
  server.send(200, "text/html", htmlCalibrate());
}

// page testloop
void handleTestloop() {
  server.send(200, "text/html", "<h1>Going into the testloop. Reset to exit.</h1>");
  tester();
}

// start script and general setup routines
void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(TX_band1_LED_PIN, OUTPUT);
  pinMode(TX_band2_LED_PIN, OUTPUT);
  pinMode(TX_band3_LED_PIN, OUTPUT);
  digitalWrite(TX_band1_LED_PIN, LOW);
  digitalWrite(TX_band2_LED_PIN, LOW);
  digitalWrite(TX_band3_LED_PIN, LOW);
 
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 error!"));
    for (;;);
  }

  // first load config from EEPROM
  loadConfig();
  // create maskString 
  String bandMaskString = "";
  for (int i = 2; i >= 0; i--) {   // van MSB (40m) naar LSB (10m)
    if (config.band & (1 << i)) bandMaskString += "1";
    else bandMaskString += "0";
  }

  showOLED("WSPR TX", ver);
  delay(2000);
  Serial.println("Config loaded:");
  Serial.printf("Call=%s, Loc=%s, dBm=%d, Interval=%d, Cal=%d, GPS=%d, Bandmask=%s, OLED=%d, WiFi Reset=%d\n",config.call, config.loc, config.dbm, config.interval, config.cal, config.gps, bandMaskString.c_str(), config.oled, config.ssid);

 

  // initialise the Si5351
  cal_factor = config.cal; 
  si5351.init(SI5351_CRYSTAL_LOAD_10PF, 0, 0);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_freq(freq * 100, SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);  // SI5351_DRIVE_8MA=5 mw / SI5351_DRIVE_2MA= 1mw/ SI5351_DRIVE_4MA= 2mw
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  Serial.println("SI5351 initialised");

  // put signal on CLK2 for stability.
  si5351.set_freq(1000000000ULL, SI5351_CLK2); // zet CLK1 op 10 MHz
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_4MA);
  si5351.output_enable(SI5351_CLK2, 1);   // activeer de uitgang
  Serial.println("SI5351 CLK2 set to 10MHz, 4MA.");

  // WiFi routines and start with WiFiManager 
  // SSID wipe out if asked for
  if (config.ssid == 9) { 
    wm.resetSettings();
    config.ssid = 0;
    saveConfig();
  }

  // give ConfigAP message on OLED if local WiFi is not found
  showOLED("WIFI", "ConfigAP"); 
  WiFi.mode(WIFI_STA);
  if (!wm.autoConnect("WSPR_ConfigAP")) {
    showOLED("WIFI FAIL", "REBOOT");
    Serial.println("WiFi connect failed, reboot...");
    ESP.restart();
  }

  Serial.println("Connected to WiFi!"); Serial.print("IP address: "); Serial.println(WiFi.localIP());
  showOLED("LOCAL IP",
         WiFi.localIP().toString().substring(0, WiFi.localIP().toString().indexOf('.', WiFi.localIP().toString().indexOf('.') + 1) + 1),
         WiFi.localIP().toString().substring(WiFi.localIP().toString().indexOf('.', WiFi.localIP().toString().indexOf('.') + 1) + 1));
  delay(5000);
  
  showOLED(config.call, config.loc, String(config.dbm) + "|" + bandMaskString + "|" + String(config.interval));
  delay(5000);
 
  // --- Start WebServer and point to the individual pages ---
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/calibrate", HTTP_POST, handleCalibrate);
  server.on("/testloop", HTTP_POST, handleTestloop);
  server.begin();
  Serial.println("Webserver started. Access via http://" + WiFi.localIP().toString());

  // based on usage get time from internet of from GPS
  if (config.gps == 0)    { 
    //setSyncProvider(epochUnixNTP); // old code for interval NTP update
    epochUnixNTP();  // get NTP time in one shot at boot 
  }
  if (config.gps == 1)    { 
    ss.begin(GPSBaud);
    Serial.println("start GPS");
    Gps_read(); 
    //while ((!gps.time.isValid())||(!gps.date.isValid()) || (!gps.location.isValid()) || (Time_set==0) || (Sat < 5 )) Gps_read(); 
  }

  // builin led will light up during webconnect
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Setup complete.");
}

void loop() {
  server.handleClient();

  if (!calibrate) {  // alleen als niet in calibratie
    int h = hour(); int m = minute(); int s = second();

    // read band configuration
    int mask = config.band;
    int bitCount = __builtin_popcount(mask);
        
    // Initialize StartMinute; this is immediately the first even minute after startup.
    if (startMinute < 0 ) {  
      //On startup, not immediately the next even minute, but 2 minutes later for web access possiblity
      startMinute = (m + 2);
      if (startMinute % 2 != 0) startMinute++;  // round to next  even minute
      if (startMinute >= 60) startMinute -= 60; // wrap around the hour
    }

    // Determine the band based on the bandmask. 
    int stepCounter = 0;
    String nextBandName;
    for (int i = 2; i >= 0; i--) { // bits: 0=10m,1=20m,2=40m
      if (mask & (1 << i)) {
        if (stepCounter == bandStep) {
          switch (i) {
            case 0: freq = band3; nextBandName = "10m"; break;
            case 1: freq = band2; nextBandName = "20m"; break;
            case 2: freq = band1; nextBandName = "40m"; break;
          }
          break;
        }
        stepCounter++;
      }
    }

    // Start TX on second 0
    if (s == 0 && m == startMinute) {
      encode(); 
      bandStep++;
      
      // calculate next slot
      if (bitCount == 1) { // single-band
        startMinute = startMinute  + config.interval;
        bandStep = 0; // reset next band search
      } else { // multi-band
        if (bandStep >= bitCount) { // cycle klaar → wachten tot nieuw interval
          startMinute = (startMinute  + 2 + config.interval) - (bitCount * 2);
          bandStep = 0; // reset next band search
        } else { // cycle busy → next even minute
          startMinute = (m + 2 );
        }
      }  
      if (startMinute >= 60) startMinute -= 60; // wrap rond uur
    }

    int deltaMin = startMinute - m;
    if (deltaMin < 0) deltaMin += 60;
    int countdownSec = deltaMin * 60 - s;
     
    // OLED update 
    showOLED("IDLE " + String(countdownSec) + " s", String(h) + ":" + (m<10?"0":"") + String(m) + ":" + (s<10?"0":"") + String(s), "next:" + nextBandName
    );
  }
}

time_t epochUnixNTP() {
  // message on OLED
  showOLED("CONNECTING", "TO NTP");
  Serial.print("Connecting to NTP Server: ");
  Serial.println(NTP_Server);

  NTPch.setSendInterval(SEND_INTV);
  NTPch.setRecvTimeout(RECV_TIMEOUT);

  do {
    dateTime = NTPch.getNTPtime(TIME_ZONE, 1);
    delay(1);
  } while (!dateTime.valid);

  setTime(dateTime.hour, dateTime.minute, dateTime.second, dateTime.day, dateTime.month, dateTime.year);

  char buf[60];
  sprintf(buf, "%02d:%02d:%02d  %02d-%02d-%04d",
        hour(), minute(), second(),
        day(), month(), year());
  Serial.println(buf);
   
  return 0;
}

void encode() {
  // dim build in led because webconnect will stop during Transmission
  digitalWrite(LED_BUILTIN, HIGH);

  // encode call, locator and power into a wspr arrysymbols
  jtencode.wspr_encode(config.call, config.loc, config.dbm, tx_buffer);
  
  // select correct LPF
  for (int i = 0; i < 3; i++) {
    // Turn pin HIGH only if freq matches this band
    digitalWrite(bandLedPins[i], (freq == bandFreqs[i]) ? HIGH : LOW);
  }

  // Show TX message and draw progress bar
  showOLED("TX ACTIVE", String(freq / 1000000.0, 6));
  display.drawRect(10, 54, 110, 8, SSD1306_WHITE);

  char msg[128];
  sprintf(msg, "- TX ON - STARTING TRANSMISSION AT : %02d+%02d - %luHz %s|%s|%d",   hour(), minute(), (unsigned long)freq, config.call, config.loc, config.dbm);
  Serial.println(msg);

  // initialise Si5351 CLK0
  si5351.set_freq(freq * 100, SI5351_CLK0);
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);  // SI5351_DRIVE_8MA=5 mw / SI5351_DRIVE_2MA= 1mw/ SI5351_DRIVE_4MA= 2mw

  unsigned long mStart = millis();
  // transmit WSPR message
  for (uint8_t i = 0; i < SYMBOL_COUNT; i++) {
    si5351.set_freq((freq * 100) + (tx_buffer[i] * TONE_SPACING), SI5351_CLK0);
    unsigned long tStart = millis();
               
    // draw progress bar
    int filled = (long)(i+1) * 110 / SYMBOL_COUNT;
    display.fillRect(10, 54, filled, 8, SSD1306_WHITE);
    display.display();
  
    while (millis() - tStart < WSPR_DELAY);
  }

  // TX off
  si5351.set_clock_pwr(SI5351_CLK0, 0);

  // all LPF (and optional led) turned off 
  for (int i = 0; i < 3; i++) {
    digitalWrite(bandLedPins[i], LOW);
  }

  //debug  
  unsigned long mEnd = millis();   // eindtijd
  Serial.print(F("WSPR doorlooptijd: "));
  Serial.print((mEnd - mStart) / 1000.0, 3); // in seconden, met 3 decimalen
  Serial.println(F(" sec"));
  Serial.println("- TX OFF - END OF TRANSMISSION...");
  // buildin led on again because webconnection reestablished.
  digitalWrite(LED_BUILTIN, LOW);
}

void Gps_read()
{ 
  showOLED("CONNECTING", "TO GPS");
  unsigned long start = millis();
  while (!Time_set) {   // alleen als de tijd nog niet gezet is
    
    while (ss.available() > 0) { 
      gps.encode(ss.read());
    }

    if (gps.date.isValid() && gps.time.isValid()) {
      Hour   = gps.time.hour();
      Minute = gps.time.minute();
      Second = gps.time.second();
      Day    = gps.date.day();
      Month  = gps.date.month();
      Year   = gps.date.year();
      Sat    = gps.satellites.value();

      // Zet de ESP systeemtijd
      setTime(Hour, Minute, Second, Day, Month, Year);
      Time_set = 1;   // vlag: tijd is gezet!

      showOLED("CONNECTED", "TO GPS");
      delay(2000);

      Serial.print(F("GPS tijd gezet: ")); Serial.println(timeStr); Serial.print(F("Satellieten: ")); Serial.println(Sat);


    }
    // Controleer geldige locatie
    if (gps.location.isValid() && Time_set == 1) {
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      char* GPSlocator = get_mh(lat, lon, 6); // 6 tekens precisie
      
      Serial.print(F("Lat/Lon: ")); Serial.print(lat,6); Serial.print(", "); Serial.print(lon,6); Serial.print(" -> Maidenhead: "); Serial.println(GPSlocator);
      
      if (strcmp(config.loc, GPSlocator) != 0) {
        strncpy(config.loc, GPSlocator, sizeof(config.loc) - 1);
        config.loc[sizeof(config.loc) - 1] = '\0';  // always null-terminator
        showOLED("LOC ADJUST", "TO", config.loc);
        delay(5000);
      }
    }

  if (millis() - start > 60000) {  // 60 sec timeout
      Serial.println(F("GPS sync mislukt (timeout)"));
      break;
    }
  }
}

void tester() {
  Serial.println(F("Transmitter tester going into a loop."));
  
  // Define arrays for bands and LEDs
  const uint32_t bandFreqs[] = { band1, band2, band3 };
  const int bandLeds[] = { TX_band1_LED_PIN, TX_band2_LED_PIN, TX_band3_LED_PIN };
    
  int teller=0;

  while(true) { //endless loop

    
    // Select LPF
    for (int i = 0; i < 3; i++) {
      digitalWrite(bandLeds[i], (i == teller) ? HIGH : LOW);
    }

    freq = bandFreqs[teller];                  // frequency from array
    // Show active band on OLED
    showOLED("TX ACTIEVE", String(freq / 1000000.0, 6),"TESTLOOP");
    
    // Enable output
    si5351.set_freq(freq * 100, SI5351_CLK0);  // load SI5351 with the frequency and enable output
    si5351.set_clock_pwr(SI5351_CLK0, 1);
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);

    delay(10000);

   // Disable output and turn off LEDs
    si5351.set_clock_pwr(SI5351_CLK0, 0);
    for (int i = 0; i <3; i++) {
      digitalWrite(bandLeds[i], LOW);
    }
 
    // Increment teller (loop 0 → 2)
    teller++;
    if (teller > 2) teller = 0;
  }
}
