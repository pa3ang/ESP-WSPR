// WSPR beacon with WM-386 (ESP6288 clone), SI5351, SSD1306 0.91" or SH1106 1.3", TinyGSP (optional)
// Original idea from Benno PA3FBX and adapted by PD8GB, PA3FEX and PA3ANG


// Below lines are to configure the display and the used PCB.
// =====================================================================================================================

// ==== SELECT DISPLAY TYPE ====  COMMENT OUT THE USED DISPLAY TYPE COMMENT THE NOT USED
//#define USE_SH1106     
#define USE_SSD1306

// ==== SELECT PCB TYPE ====  THE PA3FBX DESIGN HAS 3 LPF DESIGNED. THE PA3FEX PCB HAS THE POSSIBILTY FOR 7 LPF.
#define USE_PA3FBX_PCB
//#define USE_PA3FEX_PCB
// =====================================================================================================================


/* 
Version 1.5
  - choice of display type between SSD1306 and SH1106.
  - improved selection of used PCB.
  - BIAS reintroduced on Pin D8. 
  - improved interval of transmissions: 
      Interval given by the user is used to calculate the next transmission after finalizing a complete cycle.
      If only one band is active this counts also a completing the cycle. 
      During the cycle no interval is used meaning that all bands are following directly after each other on the next even minute.  
  - cleanup of some unused code.
    
Version 1.4
  - changed claibration literals and removed .1 and .01 Hz steps. 
  - improved valid date and time checking at TinyGPS startup.

Version 1.3
  - removed bandmask
  - created proper band configuration into the sketch stored in EEPROM and supported with a webpage.
  - changed void loop() for new band config and fixed frequency table.
  - changed void tester() to apply given band config.
  - LPF can handle the hardware version of PA3FBX with 3 pins and LPF 1, 2 and 3 individual connected to the ESP8266
      and PA3FEX version with 4028 multiplexer assuming LPF number from 1-7  (0b000 - 0b110 on lpfPins) with a possible 7 LPF assembled.
  - fixed frequency table has all band frequency on the center of the WSPR band. but due to the fact that the Si5351 is not GPS locked
      the frequency will deviate around the center given the environment temperature the unit is in.
  - improved and cleanup some code.

    
Version 1.2
  - changed gps/wifi routines 
    1. if gps is enabled in configuration (default) then program will try to lock and get the time from the gps and update the locator
          and if the gps is ok the program will create an AP to serve the configuration webpage on host ESP-WSPR IP: 192.168.4.1 
    2. if no gps or no lock the WiFimanager will try to login to the given SSID network
          and if done it will run the NTP routine and start a webservers on the assigned local IP address.
    3. if no network connection established the program will create a captive page on ESP-WSPR 192.168.4.1 to cature the 
          SSID and password. Therafter it will resync.
  - changed pin assignments D5 and D7 vor LPF

Version 1.1
  - Added dloading default values when EEPROM has not ben used before
  - corrected maidenhead.h library name
  - improved calibration html page
  - improved comments
  - some cosmetic code changes

Verion 1.0
Modified and improved script from ESP_WSPR sketch from PD0GB and PA3FBX
  - Web interface for configuration panel
  - Calibration also via the web interface
  - SSD1306 info panel texts completely revised
  - Extra debug messages via Serial
  - WSPR interval configurable via the web interface
  - SI5351 CLK2 fixed at 10 MHz continuously for stabilization
  - Warmup period removed 
  - Cleanup of unused code and variables
  - GPS support on PIO D3 (can conflict during boot / reset)
  - Read locator from GPS when GPS is enabled

Using a WM-386 (an ESP6288 clone variant).
With this, we can build a WSPR transmitter with:
  - Up to 3 bands with LPF switching (d5, D6, D7)
  - Use the buildin led - D4 PIO for indicating Webinterface active
  - Use a TinyGPS on PIO D3 
  - Time Sync using NTP
  - Provide a configuration possibility through WiFi accessable webwerver

Time synchronization is selectable via WiFi NTP server or optional GPS.

Hardware: Wemos D1 Mini, Si5351 module, 128x64 I2C OLED display, a few LEDs (optional)  

On first startup, if no WiFi credentials are stored, the ESP will switch into Access Point mode.  
SSID = "ESP-WSPR"  no password.  

Connect your phone/laptop to this network the captive portal is available at 192.168.4.1  
A configuration page appears after can enter or select the avilable WiFi SSID and enter the password.
Click “Save”. The ESP will then connect to your local WiFi.  
 
The unit will restat an show the local WiFi IP adress on the SSD1306 screen during booting.
Log in through your home network to enter the WSPR credetials and parameters:
  - Callsign
  - Locator
  - dBm (output power setting)
  - Calibration factor  (update through calibration routine)
  - Transmission interval in minutes 
  - GPS enable/disable
  - GPS enable
  - OLED rotate
  - Reset WiFi - set to 9 to reseter the local WiFi SSID and Password 
You can also switch the unit into Calibration or Testloop.

Used libraries  
  - Etherkit Si5351 by Jason Mildrum, version 2.2.0  
  - Etherkit JTEncode by Jason Mildrum, version 1.3.1  
  - NTPtimeESP  
  - TimeLib by Michael Margolis, version 1.6.1  
  - WiFiManager by tzapu, version 2.0.17  
  - Adafruit GFX Library by Adafruit, version 1.12.1  
  - Adafruit SSD1306 by Adafruit, version 2.5.15
  - Adafruit SH110X by Adafruit, version 2.1.14  
  - Adafruit BusIO (I2C)  
  - EspSoftwareSerial by Dirk Kaar, version 8.1.0  
  - TinyGPSPlus by Mikal Hart, version 1.0.3  
  - Maidenhead by Mateusz Salwach, version 1.0.1  

Version: 2025-10-25
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
#include <EEPROM.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <maidenhead.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// OLED SH1106 or SSD1306
#define I2C_ADDRESS 0x3c  // initialize with the I2C addr 0x3C Typically eBay OLED's
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // QT-PY / XIAO

#ifdef USE_SH1106
  #include <Adafruit_SH110X.h>
  #define OLED_WHITE SH110X_WHITE
  Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

#ifdef USE_SSD1306
  #include <Adafruit_SSD1306.h>
  #define OLED_WHITE SSD1306_WHITE
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

// WSPR timing
#define TONE_SPACING 146
#define WSPR_DELAY 683
#define SYMBOL_COUNT WSPR_SYMBOL_COUNT

// ESP-WSPR LPF pins according PA3FBX design
#define LPF1_PIN D5     // green from pin D7 to signal        40meter
#define LPF2_PIN D6     // yellow LED from pin D6 to signal   20meter
#define LPF3_PIN D7     // red LED from pin D5 to signal      10meter
#define TX_BIAS_PIN D8  // output Bias voor transmittion.

// NTP server
#define SEND_INTV 10
#define RECV_TIMEOUT 10
#define TIME_ZONE -1.0f       // Voor NL

// define TinyGPS pins
#define RX_pin D3             // input voor GPS TX.
#define TX_pin D0             // input voor GPS TX. (not connected physically)

// EEPROM definition
#define EEPROM_SIZE 512
#define TOTAL_BANDS 7

// --- Band structure ---
struct BandConfig {
    uint8_t nr;          // band number
    char bandname[5];    // 4 characters + null-terminator
    uint8_t filter;      // LPF number
    bool active;         // true/false
};

// --- Main configuration ---
struct Config {
    uint16_t magic;           // EEPROM validation
    char call[10];
    char loc[10];
    int dbm;
    int cal;
    int gps;
    int band;
    int oled;
    int interval;
    int ssid;
    BandConfig bands[TOTAL_BANDS];  // 7 bands
} config;

// Common variables
const char ver[10]        = "ANG-V1.5";
const char* WiFi_hostname = "ESP-WSPR";
const char* NTP_Server    = "time.google.com";

struct BandFreq {
    const char* name;
    uint32_t freqHz;
};

const BandFreq standardBands[] = {
    {"160m", 1838100},
    {"80m",  3570100},
    {"60m",  5366200},
    {"40m",  7040100},
    {"30m", 10140200},
    {"20m", 14097100},
    {"17m", 18106100},
    {"15m", 21096100},
    {"12m", 24926100},
    {"10m", 28126100},
};
const int numStandardBands = sizeof(standardBands) / sizeof(standardBands[0]);

// WSPR and state-machine parameters
int bandStep = 0;               // pointer for next band in multiband
int activeBands = 0;            // number is active bands
const char* nextBandName;       // holds band name  
int selectedFilter;             // holds LPT selected
int startMinute = -1;

// used during calibration
uint32_t freq;                  // freq for encode() routine  
uint32_t rx_freq;               // used in calibration routine
const uint32_t calibration_freq = 1000000000ULL;  // 10 MHz, in hundredths of hertz
int32_t cal_factor; 
bool calibrate = false;

// used in for encode
uint8_t tx_buffer[SYMBOL_COUNT];

// parameters for LPF switching   
const int lpfPins[] = { LPF1_PIN, LPF2_PIN, LPF3_PIN };

uint8_t mapLPF(uint8_t logicalLPF) {
  #ifdef USE_PA3FBX_PCB
    #undef TOTAL_BANDS
    #define TOTAL_BANDS 3
    switch (logicalLPF) {
      case 1: return 2;   // menu LPF1 → physical LPF2
      case 2: return 3;   // menu LPF2 → physical LPF3
      case 3: return 5;   // menu LPF3 → physical LPF5
      default: return 2;  // fallback
    }
  #else
    return logicalLPF;      // full hardware: direct mapping
  #endif
}

// TinyGPS
static const uint32_t GPSBaud   = 9600;
int Hour, Minute, Second, Day, Month, Year;
int Time_set;

// define hardare / software
Si5351 si5351;
JTEncode jtencode;

TinyGPSPlus gps;
SoftwareSerial ss(RX_pin, TX_pin);

NTPtime NTPch(NTP_Server);
strDateTime dateTime;

ESP8266WebServer server(80);
WiFiManager wm;

void setLPFLEDs(uint8_t lpf) {
    uint8_t value = lpf - 1; // convert 1-based LPF to 0–7
    for (int i = 0; i < 3; i++) {
        digitalWrite(lpfPins[i], (value & (1 << i)) ? HIGH : LOW);
    }
}
// Generic OLED display routine
void showOLED(String line1, String line2 = "", String line3 = "") {
    display.clearDisplay();
    display.setRotation(config.oled);
    display.setTextSize(2);  // altijd textSize 2
    display.setTextColor(OLED_WHITE);

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
  // if EEPROM has eronous values it's probably an absolute first start or EEPROM has been erased
  if (strlen(config.loc) != 6 ) {
    strcpy(config.call, "NOCALL");
    strcpy(config.loc, "AA00aa");
    config.dbm = 23;
    config.cal = -20000;
    config.gps = 1;
    config.oled = 0;
    config.interval = 4;
    config.ssid = 0;
     // First 3 bands filled  is the original PA3FBX hardware
    strcpy(config.bands[0].bandname, "40m");  config.bands[0].nr = 0; config.bands[0].filter = 1; config.bands[0].active = true;
    strcpy(config.bands[1].bandname, "20m");  config.bands[1].nr = 1; config.bands[1].filter = 2; config.bands[1].active = true;
    strcpy(config.bands[2].bandname, "10m");  config.bands[2].nr = 2; config.bands[2].filter = 3; config.bands[2].active = true;
    // Remaining bands empty/inactive
    for (int i = 3; i < TOTAL_BANDS; i++) {
        config.bands[i].nr = i;
        strcpy(config.bands[i].bandname, "");
        config.bands[i].filter = 0;
        config.bands[i].active = false;
    }
  }
}

void saveConfigToEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(0, config);   // save the whole struct
    EEPROM.commit();
    EEPROM.end();

    Serial.println(F("Config including bands saved to EEPROM"));
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

  html += "<h1>ESP-WSPR Config</h1>";
  html += "<form method='POST' action='/save'>";
  html += "Callsign<br><input name='call' value='" + String(config.call) + "' oninput='this.value = this.value.toUpperCase();'><br>";
  html += "Locator<br>  <input name='loc' value='" + String(config.loc)  + "' oninput='this.value = this.value.toUpperCase();'><br>";
  html += "dBm<br><input name='dbm' value='" + String(config.dbm) + "'><br>";
  html += "Interval (2,4,6, max 16 ) minutes<br><input name='interval' value='" + String(config.interval) + "'><br>";
  html += "Calibration factor<br><input name='cal' value='" + String(config.cal) + "'><br>";
  html += "GPS (0/1)<br><input name='gps' value='" + String(config.gps) + "'><br>";
  html += "OLED rotate (0/2)<br><input name='oled' value='" + String(config.oled) + "'><br>";
  html += "Reset WiFi (9)<br><input name='ssid' value='" + String(config.ssid) + "'><br>";
  html += "<input type='submit' value='Save'></form>";
  html += "<form method='POST' action='/bands'>";
  html += "<input type='submit' value='Band Config'></form>";
  html += "<form method='POST' action='/calibrate'>";
  html += "<input type='submit' value='Calibration'></form>";
  html += "<form method='POST' action='/testloop'>";
  html += "<input type='submit' value='Test Loop'></form>";
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
  config.oled     = server.arg("oled").toInt();
  config.interval = server.arg("interval").toInt();
  config.ssid     = server.arg("ssid").toInt();
  // intercept wrong values
  if ((config.oled != 0) && (config.oled != 2))  { config.oled = 0; } 
  if (config.ssid  != 9) { config.ssid = 0; } 
  if ((config.gps  != 0) && (config.gps != 1 ))  { config.gps  = 0; }
  if (config.interval < 0 || config.interval > 16 || (config.interval % 2 != 0)) { config.interval = 8; }

  saveConfig();

  server.send(200, "text/html", "<h1>Saved! Rebooting...</h1>");
  delay(1000);
  ESP.restart();
}

String htmlBandConfigPage() {
  String html = "<!DOCTYPE html>";
  html += "<html lang='en'><head>";
  html += "<meta name='format-detection' content='telephone=no'>";
  html += "<meta charset='UTF-8'>";
  html += "<meta  name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'/>";
  html += "<title>Band Configuration</title>";

  html += "<style>";
  html += "body { align-self: auto; max-width: 600px; margin: 0 auto; text-align: center; font-family: verdana; }";
  html += "table { border-collapse: collapse; margin: auto; }";
  html += "th, td { padding: 8px 12px; text-align: center; }";
  html += "th { background-color: #1fa3ec; color: white; }";
  html += "input[type='submit'] { cursor: pointer; border: 0; background-color: #1fa3ec; color: #fff; line-height: 2.4rem; font-size: 1.2rem; width: 200px; border-radius: 12px; }";
  html += "button { margin: 5px; padding: 10px; font-size: 1.2em; background-color: #1fa3ec; border: 0; width: 150px; border-radius: .3rem; color: #fff; }";
  html += "</style>";
  
  html += "</head><body><h1>Band Config</h1>";
    
  html += "<form method='POST' action='/saveBands'>";
  html += "<table border='1' cellpadding='5' cellspacing='0'>";
  html += "<tr><th>Nr</th><th>Band</th><th>LPF</th><th>Active</th></tr>";
  for (int i = 0; i < TOTAL_BANDS; i++) {
    BandConfig band = config.bands[i];
    html += "<tr>";
    // Band number
    html += "<td>" + String((band.nr+1)) + "</td>";
    // Band dropdown with empty option
    html += "<td><select name='band" + String(i) + "'>";
    html += "<option value=''";
    if (strlen(band.bandname) == 0) html += " selected";
    html += ">---</option>";
    for (int b = 0; b < numStandardBands; b++) {
      html += "<option value='" + String(standardBands[b].name) + "'";
      if (strcmp(band.bandname, standardBands[b].name) == 0) html += " selected";
      html += ">" + String(standardBands[b].name) + "</option>";
    }
    html += "</select></td>";
    // LPF dropdown
    #ifdef USE_PA3FBX_PCB
    const int maxLPF = 3;   // only 1–3 logical filters
    #else
    const int maxLPF = 7;   // full hardware
    #endif

    html += "<td><select name='filter" + String(i) + "'>";
    for (int f = 1; f <= maxLPF; f++) {
      html += "<option value='" + String(f) + "'";
      if (band.filter == f)
      html += " selected";
      html += ">LPF " + String(f) + "</option>";
    }
    html += "</select></td>";
    // Active checkbox
    html += "<td><input type='checkbox' name='active" + String(i) + "'";
    if (band.active) html += " checked";
    html += "></td></tr>";
  }

  html += "</table><br>";
  html += "<input type='submit' value='Save Bands'>";
  html += "</form></body></html>";

  return html;
}

void handleBandConfig() {
    String html = htmlBandConfigPage();  
    server.send(200, "text/html", html);
}

void handleSaveBands() {
    for (int i = 0; i < TOTAL_BANDS; i++) {
        // Band name from dropdown
        if (server.hasArg("band" + String(i))) {
            strncpy(config.bands[i].bandname, server.arg("band" + String(i)).c_str(), sizeof(config.bands[i].bandname)-1);
            config.bands[i].bandname[sizeof(config.bands[i].bandname)-1] = '\0';  // ensure null-terminated
        }

        // LPF value
        if (server.hasArg("filter" + String(i))) {
            config.bands[i].filter = server.arg("filter" + String(i)).toInt();
        }

        // Active checkbox
        config.bands[i].active = server.hasArg("active" + String(i));
    }

    saveConfigToEEPROM();

    String response = "<!DOCTYPE html><html><body>";
    response += "<h3>Bands saved successfully!</h3>";
    response += "</body></html>";

    server.send(200, "text/html", response);
}

// Calibration page
String htmlCalibrate() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>ESP-WSPR Calibrate</title>";

  html += "<style>";
  html += "body{font-family:Arial,Helvetica,sans-serif;text-align:center;background:#fff;margin:0;padding:0;}";
  html += "h1{margin-top:20px;color:#003366;}";
  html += "p{font-size:1.1em;color:#333;}";
  html += "form{display:inline-block;margin-top:20px;}";
  html += "button{";
  html += "display:inline-block;margin:5px;width:140px;height:45px;";
  html += "font-size:16px;font-weight:bold;";
  html += "background-color:#1fa3ec;color:white;";
  html += "border:none;border-radius:10px;";
  html += "box-shadow:0 2px 4px rgba(0,0,0,0.2);";
  html += "transition:background 0.2s,transform 0.1s;";
  html += "}";
  html += "button:hover{background-color:#007acc;transform:scale(1.03);}";
  html += "</style>";

  html += "</head><body>";
  html += "<h1>Si5351 Calibrate</h1>";
  html += "<p>Target frequency: " + String(calibration_freq / 1000.0, 3) + " kHz</p>";
  html += "<p>Current cal_factor: " + String(cal_factor) + "</p>";

  html += "<form method='POST'>";
  html += "<button name='step' value='i'>+1 Hz</button>";
  html += "<button name='step' value='k'>-1 Hz</button><br>";
  html += "<button name='step' value='o'>+10 Hz</button>";
  html += "<button name='step' value='l'>-10 Hz</button><br>";
  html += "<button name='step' value='p'>+100 Hz</button>";
  html += "<button name='step' value=';'>-100 Hz</button><br><br>";
  html += "<button name='step' value='save'>Save & Reboot</button>";
  html += "</form>";

  html += "</body></html>";
  return html;
}

// Calibration routine live interaction with webpage
void handleCalibrate() {
  static bool cal_init = false;

  if (!cal_init) {
    if (rx_freq == 0) rx_freq = calibration_freq;

    cal_factor = (int32_t)(calibration_freq - rx_freq) + config.cal;

    si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351.set_freq(calibration_freq, SI5351_CLK0);
    si5351.set_clock_pwr(SI5351_CLK0, 1);
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);

    digitalWrite(LPF1_PIN, LOW);
    digitalWrite(LPF2_PIN, HIGH);
    digitalWrite(LPF3_PIN, LOW);
    digitalWrite(TX_BIAS_PIN, HIGH);

    showOLED("TX ACTIVE", String(calibration_freq), "CALIBRATE");

    calibrate = true;
    cal_init = true;
  }

  // Process POST requests
  if (server.method() == HTTP_POST && server.hasArg("step")) {
    String step = server.arg("step");

    if (step == "save") {
      config.cal = cal_factor;
      saveConfig();
      server.send(200, "text/html", "<meta charset='UTF-8'><h1>Saved! Rebooting...</h1>");
      delay(1000);
      ESP.restart();
      return;
    }

    // step adjustment
    char c = step.charAt(0);
    int32_t stepVal = 0;
    switch (c) {
      case 'i': stepVal = 100; break;
      case 'k': stepVal = -100; break;
      case 'o': stepVal = 1000; break;
      case 'l': stepVal = -1000; break;
      case 'p': stepVal = 10000; break;
      case ';': stepVal = -10000; break;
    }

    rx_freq += stepVal;
    int32_t diff = (int32_t)(calibration_freq - rx_freq);
    cal_factor = diff + config.cal;

    si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351.pll_reset(SI5351_PLLA);
    si5351.set_freq(calibration_freq, SI5351_CLK0);
  }

  // Always send fresh HTML (safe for iPhone Safari)
  String html = htmlCalibrate();
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.send(200, "text/html", html);
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

  pinMode(LPF1_PIN, OUTPUT);
  pinMode(LPF2_PIN, OUTPUT);
  pinMode(LPF3_PIN, OUTPUT);
  pinMode(TX_BIAS_PIN, OUTPUT);
  
  digitalWrite(TX_BIAS_PIN, LOW);
  digitalWrite(LPF1_PIN, LOW);
  digitalWrite(LPF2_PIN, LOW);
  digitalWrite(LPF3_PIN, LOW);

  #ifdef USE_SH1106
    Serial.println("Initialiseer SH1106...");
    if (!display.begin(I2C_ADDRESS, true)) {
      Serial.println(F("Fout bij SH1106-init!"));
      for (;;);
    }
  #endif

  #ifdef USE_SSD1306
    Serial.println("Initialiseer SSD1306...");
    if (!display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS)) {
      Serial.println(F("Fout bij SSD1306-init!"));
      for (;;);
    }
  #endif
 
  // first load config from EEPROM
  loadConfig();
  
  showOLED("WSPR TX", ver);
  delay(2000);
  Serial.println("Config loaded:");
  Serial.printf("Call=%s, Loc=%s, dBm=%d, Interval=%d, Cal=%d, GPS=%d, OLED=%d, WiFi Reset=%d\n",config.call, config.loc, config.dbm, config.interval, config.cal, config.gps, config.oled, config.ssid);
  
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

  // WiFi routines and start with WiFiManager or embedded WiFi
  // SSID wipe out if asked for
  if (config.ssid == 9) { 
    wm.resetSettings();
    config.ssid = 0;
    saveConfig();
  }

  // WIFI setup and start either local, newtoerk, or captive
  // start GPS if flagged as available in configuration
  if (config.gps == 1) {
    ss.begin(GPSBaud);
    Gps_read();  
  }
  
  if (Time_set) {
    // WiFi AP on ESP device
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP-WSPR", "");
    WiFiServer server(80);
    Serial.println("WiFi AP created");
    showOLED("ESP-WSPR", "192.168.", "4.1");
    delay(5000);
  } else {
    // Try to find given SSID on local network
    showOLED("CONNECTING", "WIFI"); 
    WiFi.mode(WIFI_STA);
    if (!wm.autoConnect("ESP-WSPR")) {
      showOLED("WIFI FAIL", "REBOOT");
      delay(5000);
      Serial.println("WiFi connect failed, reboot...");
      ESP.restart();
    }
    Serial.println("Connected to WiFi!"); Serial.print("IP address: "); Serial.println(WiFi.localIP());
    showOLED("NETWRK IP",
         WiFi.localIP().toString().substring(0, WiFi.localIP().toString().indexOf('.', WiFi.localIP().toString().indexOf('.') + 1) + 1),
         WiFi.localIP().toString().substring(WiFi.localIP().toString().indexOf('.', WiFi.localIP().toString().indexOf('.') + 1) + 1));
         delay(5000);
  }
  
  // no GPS connection then get time from NTP server
  if (!Time_set)    { 
    //setSyncProvider(epochUnixNTP); // old code for interval NTP update
    epochUnixNTP();  // get NTP time in one shot at boot 
  }

  showOLED(config.call, config.loc, "dBm:"+ String(config.dbm) + " |" + String(config.interval));
  delay(5000);
 
  // Start WebServer and point to the individual pages 
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/bands", HTTP_POST, handleBandConfig);
  server.on("/saveBands", HTTP_POST, handleSaveBands);
  server.on("/calibrate", HTTP_POST, handleCalibrate);
  server.on("/testloop", HTTP_POST, handleTestloop);
  server.begin();
  Serial.println("Webserver started. Access via http://" + WiFi.localIP().toString());
 
  // builin led will light up during webconnect
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Setup complete.");
}

void loop() {
  server.handleClient();
  // there must be a valid callsign
  if (strcmp(config.call, "NOCALL") == 0) {
      showOLED("ENTER", "CALLSIGN", "IN BROWSER");
      return;
  }
  // do not start when in Calibrate mode
  if (!calibrate) {  // alleen als niet in calibratie
    // get current time 
    int h = hour(); int m = minute(); int s = second();
    
    // Initialize StartMinute; this is immediately the first even minute after startup.
    if (startMinute < 0 ) {  
      //This routine is called at startup so calculate next start with 2 minutes idle time at least to permit web access
      startMinute = (m + 2);
      if (startMinute % 2 != 0) startMinute++;  // round to next  even minute
      if (startMinute >= 60) startMinute -= 60; // wrap around the hour
    }
    
    // find next band to transmit to show on OLED
    // Count active bands
    activeBands=0;
    for (int i = 0; i < TOTAL_BANDS; i++) {
      if (config.bands[i].active) activeBands++;
    }

    if (activeBands == 0) {
      Serial.println("⚠️ No active bands configured!");
    } else {

      // Wrap if bandStep exceeds activeBands means cycle ended and reset to first active band
      if (bandStep >= activeBands) bandStep = 0;

      // Find the next active band and get the frequency 
      int stepCounter = 0; 
      for (int i = 0; i < TOTAL_BANDS; i++) {
        BandConfig band = config.bands[i];
        if (band.active) {
          if (stepCounter == bandStep) {
            nextBandName = band.bandname;
            selectedFilter = band.filter;
            break;
          }
          stepCounter++;
        }
      }
      // Find frequency for this band from standardBands[]
      freq = 0;
      for (int j = 0; j < numStandardBands; j++) {
        if (strcmp(standardBands[j].name, nextBandName) == 0) {
          freq = standardBands[j].freqHz;
          break;
        }
      }   
      // so all parameters are set and can be changed without restart.
    }
    // Start TX on second 0
    if (s == 0 && m == startMinute && activeBands !=0) {
      encode(); 
      bandStep++;
      
      // calculate next slot based on the interval set by the user
      // if bandStep is greater than actve bands then the cycle restartes after de set interval
      if (bandStep >= activeBands) {
        startMinute = (startMinute +2) + config.interval;
      } else {
        startMinute = (startMinute +2);
      }
      if (startMinute >= 60) startMinute -= 60; // wrap rond uur
    }

    int deltaMin = startMinute - m;
    if (deltaMin < 0) deltaMin += 60;
    int countdownSec = deltaMin * 60 - s;

    // OLED update 
    if (activeBands == 0) {
      showOLED("NO BAND!","CHECK","CONFIG");
    } else {
      showOLED("IDLE " + String(countdownSec) + " s", String(h) + ":" + (m<10?"0":"") + String(m) + ":" + (s<10?"0":"") + String(s), String("Nxt:") + nextBandName + String("|") + String(selectedFilter));
    }
  }
}

void setLPF(uint8_t lpf) {
    // LPF 1 → 0b000, LPF 2 → 0b001, ..., LPF 8 → 0b111
    uint8_t value = lpf - 1; // convert LPF 1-8 to 0-7 binary

    for (int i = 0; i < 3; i++) {
        digitalWrite(lpfPins[i], (value & (1 << i)) ? HIGH : LOW);
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
  digitalWrite(TX_BIAS_PIN, HIGH);

  // encode call, locator and power into a wspr arrysymbols
  jtencode.wspr_encode(config.call, config.loc, config.dbm, tx_buffer);
  
  uint8_t physicalLPF = mapLPF(selectedFilter);
  setLPFLEDs(physicalLPF);

  // Show TX message and draw progress bar
  showOLED("TX ACTIVE", String(freq / 1000000.0, 6));
  display.drawRect(10, 54, 110, 8, OLED_WHITE);

  char msg[128];
  sprintf(msg, "- TX ON - STARTING TRANSMISSION AT : %02d:%02d - %luHz %s|%s|%d LPF: %d",   hour(), minute(), (unsigned long)freq, config.call, config.loc, config.dbm, selectedFilter);
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
    display.fillRect(10, 54, filled, 8, OLED_WHITE);
    display.display();
  
    while (millis() - tStart < WSPR_DELAY);
  }

  // TX off
  si5351.set_clock_pwr(SI5351_CLK0, 0);

  // all LPF turned off 
  for (int i = 0; i < 3; i++) {
    digitalWrite(lpfPins[i], LOW);
  }

  //debug  
  unsigned long mEnd = millis();   // eindtijd
  Serial.print(F("WSPR doorlooptijd: "));
  Serial.print((mEnd - mStart) / 1000.0, 3); // in seconden, met 3 decimalen
  Serial.println(F(" sec"));
  Serial.println("- TX OFF - END OF TRANSMISSION...");
  digitalWrite(TX_BIAS_PIN, LOW);
  // buildin led on again because webconnection reestablished.
  digitalWrite(LED_BUILTIN, LOW);
}

void Gps_read()
{ 
  showOLED("CONNECTING", "TO GPS");
  unsigned long start = millis();
  while (!Time_set) {   // only if time has not been set before
    
    while (ss.available() > 0) { 
      gps.encode(ss.read());
    }

    // valid date & tima and enough satellites available
    if (gps.date.isValid() && gps.time.isValid() && gps.date.year() > 2020 &&
        !(gps.time.hour() == 0 && gps.time.minute() == 0 && gps.time.second() == 0) &&
        gps.satellites.isValid() && gps.satellites.value() >= 3) {  // at least 3 satellites = 2D fix
          
      Hour   = gps.time.hour();
      Minute = gps.time.minute();
      Second = gps.time.second();
      Day    = gps.date.day();
      Month  = gps.date.month();
      Year   = gps.date.year();
      int Sat    = gps.satellites.value();

      // Move to ESP systeem time
      setTime(Hour, Minute, Second, Day, Month, Year);
      Time_set = 1;   // vlag: tijd is gezet!

      showOLED("CONNECTED", "TO GPS");
      delay(2000);

      char buf[60];
      sprintf(buf, "%02d:%02d:%02d  %02d-%02d-%04d",
              hour(), minute(), second(),
              day(), month(), year());

      Serial.print(F("GPS tijd gezet : ")); Serial.println(buf); Serial.print(F("Satellieten: ")); Serial.println(Sat);
    }

    // Check for valid location
    if (gps.location.isValid() && Time_set == 1) {
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      char* GPSlocator = get_mh(lat, lon, 6); // 6 tekens precisie
      
      Serial.print(F("Lat/Lon: ")); Serial.print(lat,6); Serial.print(", "); Serial.print(lon,6); Serial.print(" -> Maidenhead: "); Serial.println(GPSlocator);

      // if locator is different from config, then use this run the gps found loaction
      if (strcmp(config.loc, GPSlocator) != 0) {
        strncpy(config.loc, GPSlocator, sizeof(config.loc) - 1);
        config.loc[sizeof(config.loc) - 1] = '\0';  // always null-terminator
        showOLED("LOC ADJUST", "TO", config.loc);
        delay(5000);
      }
    }

  if (millis() - start > 60000) {  // 60 sec timeout
      Serial.println(F("GPS sync failed (timeout)"));
      break;
    }
  }
}

void tester() {
  Serial.println(F("Transmitter tester going into a loop."));
  digitalWrite(TX_BIAS_PIN, HIGH);

  int teller = 0;

  // Count active bands first
  activeBands = 0;
  for (int i = 0; i < TOTAL_BANDS; i++) {
    if (config.bands[i].active) activeBands++;
  }

  if (activeBands == 0) {
    Serial.println(F("No active bands configured!"));
    return;
  }

  while (true) { // endless loop
    // Find next active band
    int currentIndex = -1;
    int count = 0;
    for (int i = 0; i < TOTAL_BANDS; i++) {
      if (config.bands[i].active) {
        if (count == teller) {
        currentIndex = i;
        break;
        }
        count++;
      }
    }
    if (currentIndex < 0) break; // safety check

    BandConfig band = config.bands[currentIndex];

    // Set LPF LEDs
    setLPFLEDs(mapLPF(band.filter));
        
    // Get frequency from standard band table
    uint32_t bandFreq = 0;
    for (int b = 0; b < numStandardBands; b++) {
      if (strcmp(band.bandname, standardBands[b].name) == 0) {
        bandFreq = standardBands[b].freqHz;
        break;
      }
    }

    if (bandFreq == 0) {
      Serial.println(F("Band frequency not found, skipping."));
      teller = (teller + 1) % activeBands;
      continue;
    }

    freq = bandFreq;

    // Show active band on OLED
    showOLED("TX ACTIEVE", String(freq / 1000000.0, 6), "LPF: " + String(band.filter));

    // Enable output
    si5351.set_freq(freq * 100, SI5351_CLK0);
    si5351.set_clock_pwr(SI5351_CLK0, 1);
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);

    delay(10000); // transmit duration

    // Disable output and turn off LEDs
    si5351.set_clock_pwr(SI5351_CLK0, 0);
    for (int i = 0; i < 3; i++) digitalWrite(lpfPins[i], LOW);

    // Increment teller to next active band
    teller = (teller + 1) % activeBands;
  }
}