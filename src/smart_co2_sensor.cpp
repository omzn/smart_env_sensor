/*
Aquatan Generic Environment sensor

Supported controllers
* M5 ATOM Lite / M5 ATOM S3 / M5 ATOM S3 Lite

Supported sensors / relays
* ENV III Unit
* DS18B20
* SCD40 CO2 sensor Unit / MHZ19 
* M5 HUB Switch.D
* TPLink Smart Plug (HS105)
* DC fan (5V) for humidity control
* micro magnetic switch for door open sensor 
*/

#if defined(ARDUINO_M5Stack_ATOM)
#include <M5Atom.h>
//#else 
#elif defined(ARDUINO_M5Stack_ATOMS3)
#include <M5AtomS3.h>
#endif

//#define HAS_DISPLAY 1   // HAS_DISPLAY は platformio.ini で設定する．

#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <FS.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
// #include <WiFiManager.h>
#include <Wire.h>
#include <pgmspace.h>

#include "board.h"
#include "smart_env_esp32.h"  // Configuration parameters

#if defined(ARDUINO_M5Stack_ATOM)
  #define DRAWPIX(pixelcolor)    M5.dis.drawpix(0,pixel_color)
  #define DRAWSCREEN(...)        
#elif defined(ARDUINO_M5Stack_ATOMS3)
  #define DRAWPIX(pixelcolor)    M5.dis.drawpix(pixel_color)
  #if HAS_DISPLAY
    #define DRAWSCREEN(...)        drawScreen(__VA_ARGS__)
    #include "DSEG7Classic-BoldItalic12pt7b.h"
    #include "DSEG7Classic-BoldItalic14pt7b.h"
    #include "DSEG7Classic-BoldItalic16pt7b.h"
    #include "DSEG7Classic-BoldItalic24pt7b.h"
    #include "DSEG7Classic-BoldItalic32pt7b.h"
    #include "DSEG7Classic-BoldItalic9pt7b.h"
    #define FONT_7SEG_LARGE M5.Lcd.setFreeFont(&DSEG7Classic_BoldItalic32pt7b)
    #define FONT_7SEG_BIG M5.Lcd.setFreeFont(&DSEG7Classic_BoldItalic24pt7b)
    #define FONT_7SEG M5.Lcd.setFreeFont(&DSEG7Classic_BoldItalic16pt7b)
    #define FONT_7SEG_MEDIUM M5.Lcd.setFreeFont(&DSEG7Classic_BoldItalic14pt7b)
    #define FONT_7SEG_SMALL M5.Lcd.setFreeFont(&DSEG7Classic_BoldItalic12pt7b)
    #define FONT_7SEG_TINY M5.Lcd.setFreeFont(&DSEG7Classic_BoldItalic9pt7b)
  #else
    #define DRAWSCREEN(...)
    // Include the header files that contain the icons
  #endif    
#endif

#ifdef USE_MHZ19
#include "MHZ19.h"
#endif
#ifdef USE_SCD4X
#include <SparkFun_SCD4x_Arduino_Library.h>  //Click here to get the library: http://librarymanager/All#SparkFun_SCD4x
#endif
#ifdef USE_DS18B20
#include <DallasTemperature.h>
#include <OneWire.h>
#endif
#if defined(USE_HAT_ENV3) || defined(USE_GROVE_ENV3)
#include "QMP6988.h"
#include "SHT3X.h"
#endif

#ifdef USE_SGP30
#include <Adafruit_SGP30.h>
#endif

#include "debugmacros.h"
#include "DataLog.h"

#include "fan.h"
#include "ntp.h"
#include "relay.h"

enum { FONT_SMALL = 0, FONT_MEDIUM, FONT_BIG, FONT_LARGE };
enum { DOOR_OPEN = 0, DOOR_CLOSE };

const String boolstr[2] = {"false", "true"};

String website_name = "sensor";
const char *apSSID = "WIFI_SENSOR";
DNSServer dnsServer;

const IPAddress apIP(192, 168, 1, 1);
boolean wifi_ap_mode = false;
String ssid = "hoge";
String wifipass = "piyo";
uint32_t wifi_connect_timer;

String url_endpoint = "";
uint32_t timer_count = 0;
uint32_t p_millis;
uint8_t rtcint, btnint;

uint8_t lcd_rotation = 0;
uint8_t use_postdb = 0;

uint8_t use_thermo = 0;
uint8_t use_humidity = 0; 
uint8_t use_pressure = 0;
uint8_t use_co2 = 0;
uint8_t use_doorsensor = 0;
uint8_t use_fan = 0;
uint8_t use_relay = 0;
uint8_t use_extrelay = 0;
uint8_t use_tplug = 0;
uint8_t use_tvoc = 0;

String url_extrelay = "";

uint8_t manage_fan = 0;
uint8_t manage_relay1 = 0;
uint8_t manage_relay2 = 0;

String host_tplug = "";

String co2_id = "co2";
String temp_id = "temp";
String humid_id = "humid";
String press_id = "pressure";
String door_id = "door";
String tvoc_id = "tvoc";

uint8_t door = DOOR_OPEN, prev_door = DOOR_OPEN;
uint32_t door_count;

// WiFiManager wifiManager;
Preferences prefs;
HTTPClient http;
WiFiClient client;
WiFiUDP udp;

WebServer webServer(80);
NTP ntp("ntp.nict.go.jp");

#ifdef USE_DS18B20
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature ds18b20(&oneWire);
#endif

#if defined(USE_HAT_ENV3) || defined(USE_GROVE_ENV3)
SHT3X sht30;
QMP6988 qmp6988;
#endif

#ifdef USE_SGP30
Adafruit_SGP30 sgp;
#endif

#ifdef USE_MHZ19
HardwareSerial mySerial(1);
MHZ19 mhz19(&mySerial);
#endif

#ifdef USE_SCD4X
SCD4x co2sensor;
#endif

Relay *relay1;//(RELAY1_PIN, 0xff0000);
Relay *relay2;//(RELAY2_PIN, 0x00ff00);
uint32_t pixel_color = 0;
TPLinkSmartPlug *tplug;

Fan fan(FAN_PIN,70);

String error_message;

DataLog *templog, *humidlog, *co2log, *pressurelog, *tvoclog;


/*
                          888888888888                                       
                         ,d    88                                            
                         88    88                                            
 ,adPPYb,d8  ,adPPYba, MM88MMM 88  ,adPPYba, 88,dPYba,,adPYba,  8b,dPPYba,   
a8"    `Y88 a8P_____88   88    88 a8P_____88 88P'   "88"    "8a 88P'    "8a  
8b       88 8PP"""""""   88    88 8PP""""""" 88      88      88 88       d8  
"8a,   ,d88 "8b,   ,aa   88,   88 "8b,   ,aa 88      88      88 88b,   ,a8"  
 `"YbbdP"Y8  `"Ybbd8"'   "Y888 88  `"Ybbd8"' 88      88      88 88`YbbdP"'   
 aa,    ,88                                                     88           
  "Y8bbdP"                                                      88           

 */

// temp
void getTemp() {
  if (use_thermo) {
#ifdef USE_DS18B20
    ds18b20.requestTemperatures();
    temp = ds18b20.getTempCByIndex(0);
    templog->add(temp);
#endif
    DPRINT("temp:");
    DPRINTLN(templog->latest());
  }
}

/*
                                                                                            
                               88        88                                88          88  
                         ,d    88        88                                ""          88  
                         88    88        88                                            88  
 ,adPPYb,d8  ,adPPYba, MM88MMM 88aaaaaaaa88 88       88 88,dPYba,,adPYba,  88  ,adPPYb,88  
a8"    `Y88 a8P_____88   88    88""""""""88 88       88 88P'   "88"    "8a 88 a8"    `Y88  
8b       88 8PP"""""""   88    88        88 88       88 88      88      88 88 8b       88  
"8a,   ,d88 "8b,   ,aa   88,   88        88 "8a,   ,a88 88      88      88 88 "8a,   ,d88  
 `"YbbdP"Y8  `"Ybbd8"'   "Y888 88        88  `"YbbdP'Y8 88      88      88 88  `"8bbdP"Y8  
 aa,    ,88                                                                                
  "Y8bbdP"                                                                                 

 */

// humid
void getHumid() {
  float temp, humid;
  if (use_humidity) {
#if defined(USE_HAT_ENV3) || defined(USE_GROVE_ENV3)
    int r = sht30.get();
    if (r == 0) {
      if (!use_thermo) {
        temp = sht30.readTemperature();
        templog->add(temp);
        DPRINT("temp:");
        DPRINTLN(templog->latest());
      }
      humid = sht30.readHumidity();
      humidlog->add(humid);
      DPRINT("humid:");
      DPRINTLN(humidlog->latest());
    } else {
      DPRINTLN("Cannot find SHT3X");
    }
#endif
  }
}


/*
                                                                                                                         
                               88888888ba                                                                               
                         ,d    88      "8b                                                                              
                         88    88      ,8P                                                                              
 ,adPPYb,d8  ,adPPYba, MM88MMM 88aaaaaa8P' 8b,dPPYba,  ,adPPYba, ,adPPYba, ,adPPYba, 88       88 8b,dPPYba,  ,adPPYba,  
a8"    `Y88 a8P_____88   88    88""""""'   88P'   "Y8 a8P_____88 I8[    "" I8[    "" 88       88 88P'   "Y8 a8P_____88  
8b       88 8PP"""""""   88    88          88         8PP"""""""  `"Y8ba,   `"Y8ba,  88       88 88         8PP"""""""  
"8a,   ,d88 "8b,   ,aa   88,   88          88         "8b,   ,aa aa    ]8I aa    ]8I "8a,   ,a88 88         "8b,   ,aa  
 `"YbbdP"Y8  `"Ybbd8"'   "Y888 88          88          `"Ybbd8"' `"YbbdP"' `"YbbdP"'  `"YbbdP'Y8 88          `"Ybbd8"'  
 aa,    ,88                                                                                                             
  "Y8bbdP"                                                                                                              

 */

// pressure
void getPressure() {
  float pressure;
  if (use_pressure) {
#if defined(USE_HAT_ENV3) || defined(USE_GROVE_ENV3)
    pressure = qmp6988.calcPressure() / 100.0;
    pressurelog->add(pressure);
#endif
    DPRINT("press:");
    DPRINTLN(pressure);
  }
}
/*
                                                                        
                                 ,ad8888ba,   ,ad8888ba,   ad888888b,  
                         ,d     d8"'    `"8b d8"'    `"8b d8"     "88  
                         88    d8'          d8'        `8b        a8P  
 ,adPPYb,d8  ,adPPYba, MM88MMM 88           88          88     ,d8P"   
a8"    `Y88 a8P_____88   88    88           88          88   a8P"      
8b       88 8PP"""""""   88    Y8,          Y8,        ,8P a8P'        
"8a,   ,d88 "8b,   ,aa   88,    Y8a.    .a8P Y8a.    .a8P d8"          
 `"YbbdP"Y8  `"Ybbd8"'   "Y888   `"Y8888Y"'   `"Y8888Y"'  88888888888  
 aa,    ,88                                                            
  "Y8bbdP"                                                             

 */

/* get CO2 */
void getCO2() {
  float co2;
  if (use_co2) {
#ifdef USE_SCD4X
    co2 = co2sensor.getCO2();
    if (co2 == 0) return;
    co2log->add(co2);
    DPRINT("co2:");
    DPRINTLN(co2log->latest());
    if (!use_thermo && !use_humidity) {
      temp = co2sensor.getTemperature();
      templog->add(temp);
      DPRINT("temp:");
      DPRINTLN(templog->latest());
    }
    if (!use_humidity) {
      humid = co2sensor.getHumidity();
      humidlog->add(humid);
      DPRINT("humid:");
      DPRINTLN(humidlog->latest());
    }
#endif
#ifdef USE_MHZ19
    MHZ19_RESULT response = mhz19.retrieveData();
    if (response == MHZ19_RESULT_OK) {
      float c = mhz19.getCO2();
      if (c == 0) {
        ESP.restart();
        while (1)
          ;
      } else {
        co2 = c;
        co2log->add(co2);
        DPRINT("co2:");
        DPRINTLN(co2log->latest());
      }
    } else {
      DPRINT(F("Error, code: "));
      DPRINTLN(response);
    }
#endif
  }
}

void getTVOC() {
  float tvoc;
  if (use_tvoc) {
#ifdef USE_SGP30
    if (! sgp.IAQmeasure()) {
      Serial.println("Measurement failed");
      return;
    }
    tvoc = sgp.TVOC;
    tvoclog->add(tvoc);
    DPRINTF("tvoc: raw %d   log %f \n",sgp.TVOC,tvoclog->latest());
//    DPRINTLN(tvoclog->latest());
#endif
  }
}

/*
                                                                                  
                               88888888ba,                                       
                         ,d    88      `"8b                                      
                         88    88        `8b                                     
 ,adPPYb,d8  ,adPPYba, MM88MMM 88         88  ,adPPYba,   ,adPPYba,  8b,dPPYba,  
a8"    `Y88 a8P_____88   88    88         88 a8"     "8a a8"     "8a 88P'   "Y8  
8b       88 8PP"""""""   88    88         8P 8b       d8 8b       d8 88          
"8a,   ,d88 "8b,   ,aa   88,   88      .a8P  "8a,   ,a8" "8a,   ,a8" 88          
 `"YbbdP"Y8  `"Ybbd8"'   "Y888 88888888Y"'    `"YbbdP"'   `"YbbdP"'  88          
 aa,    ,88                                                                      
  "Y8bbdP"                                                                       

 */

// door status
void getDoor() {
  if (use_doorsensor) {
    door = digitalRead(DOORSENSOR_PIN);
    DPRINT("door:");
    DPRINTLN(door);
    if (door == DOOR_OPEN) {
      if (door_count == 0) {
        post_note(door_id, "open");
      }
      door_count++;
    } else {
      if (door_count > 0) {
        post_note(door_id, "close");
      }
      door_count = 0;
    }
  }
}
/*
                                                                                            
                                ad88888ba                                                  
                         ,d    d8"     "8b ,d                 ,d                           
                         88    Y8,         88                 88                           
 ,adPPYb,d8  ,adPPYba, MM88MMM `Y8aaaaa, MM88MMM ,adPPYYba, MM88MMM 88       88 ,adPPYba,  
a8"    `Y88 a8P_____88   88      `"""""8b, 88    ""     `Y8   88    88       88 I8[    ""  
8b       88 8PP"""""""   88            `8b 88    ,adPPPPP88   88    88       88  `"Y8ba,   
"8a,   ,d88 "8b,   ,aa   88,   Y8a     a8P 88,   88,    ,88   88,   "8a,   ,a88 aa    ]8I  
 `"YbbdP"Y8  `"Ybbd8"'   "Y888  "Y88888P"  "Y888 `"8bbdP"Y8   "Y888  `"YbbdP'Y8 `"YbbdP"'  
 aa,    ,88                                                                                
  "Y8bbdP"                                                                                 

 */

// 別のセンサーの状態
int getStatus(float *t, float *h, String url) {
  //  HTTPClient http;
  String body;
  url += "/status";

  for (int i = 0; i < 1; i++) {
    http.begin(url);
    http.setTimeout(1000);
    int httpCode = http.GET();
    body = http.getString();
    http.end();
    if (httpCode != 200) {
      DPRINTLN("Cannot obtain " + url + " " + String(httpCode));
      //      if (i == 2)
      return 1;
      //      else
      //        delay(500);
    } else {
      if (i > 0) DPRINTLN("Success: " + url + " " + String(httpCode));
      break;
    }
  }

  const size_t capacity = 512;
  DynamicJsonBuffer jsonBuffer(capacity);
  JsonObject &root = jsonBuffer.parseObject(body);
  if (!root.success()) {
    DPRINTLN(F("Parsing failed!"));
    return 1;
  } else {
    *t = root["air_temp"];
    *h = root["air_humid"];
  }
  return 0;
}

#if defined(ARDUINO_M5Stack_ATOMS3) && HAS_DISPLAY
/*
                                                                                                                      
   ad88                                                                                88 88             88          
  d8"                                                         ,d                       88 ""             ""   ,d     
  88                                                          88                       88                     88     
MM88MMM ,adPPYba,  8b,dPPYba, 88,dPYba,,adPYba,  ,adPPYYba, MM88MMM            ,adPPYb,88 88  ,adPPYb,d8 88 MM88MMM  
  88   a8"     "8a 88P'   "Y8 88P'   "88"    "8a ""     `Y8   88              a8"    `Y88 88 a8"    `Y88 88   88     
  88   8b       d8 88         88      88      88 ,adPPPPP88   88              8b       88 88 8b       88 88   88     
  88   "8a,   ,a8" 88         88      88      88 88,    ,88   88,             "8a,   ,d88 88 "8a,   ,d88 88   88,    
  88    `"YbbdP"'  88         88      88      88 `"8bbdP"Y8   "Y888            `"8bbdP"Y8 88  `"YbbdP"Y8 88   "Y888  
                                                                                              aa,    ,88             
                                                                   888888888888                "Y8bbdP"              

 */

/* 浮動小数点数を適切な文字列に変換する．7SEG用． */
String format_digit(float f, int digits, int decimal = 0) {
  int divnum = pow(10, digits - 1 + decimal);
  int zeroflag = 1;
  int negativeflag = 0;
  String s = "";
  int num = (int)(f * pow(10, decimal));
  if (num < 0) {
    num = -num;
    negativeflag = 1;
  }
  //  DPRINT("num=");
  //  DPRINTLN(num);
  for (int i = 0; i < digits + decimal; i++) {
    if (num / divnum == 0) {
      if (zeroflag == 1) {
        if (i == digits - 1) {
          zeroflag = 0;
          s += "0";
          if (decimal > 0) {
            s += ".";
          }
        } else {
          s += "!";
        }
      } else {
        s += "0";
        if (i == digits - 1 && decimal > 0) {
          s += ".";
        }
      }
    } else {
      s += String(num / divnum);
      if (i == digits - 1 && decimal > 0) {
        s += ".";
      }
      zeroflag = 0;
    }
    num %= divnum;
    divnum /= 10;
    //    DPRINTLN(s);
  }
  if (negativeflag) {
    int i = s.lastIndexOf('!');
    if (i >= 0) {
      s.setCharAt(i, '-');
    } else {
      s = '-' + s;
    }
  }
  return s;
}


/*
                                                                                                                                                                  
         88                                          88                                    88 88          88 8b           d8          88                         
         88                                          88                                    88 ""          88 `8b         d8'          88                         
         88                                          88                                    88             88  `8b       d8'           88                         
 ,adPPYb,88 8b,dPPYba, ,adPPYYba, 8b      db      d8 88 8b,dPPYba,  8b       d8 ,adPPYYba, 88 88  ,adPPYb,88   `8b     d8' ,adPPYYba, 88 88       88  ,adPPYba,  
a8"    `Y88 88P'   "Y8 ""     `Y8 `8b    d88b    d8' 88 88P'   `"8a `8b     d8' ""     `Y8 88 88 a8"    `Y88    `8b   d8'  ""     `Y8 88 88       88 a8P_____88  
8b       88 88         ,adPPPPP88  `8b  d8'`8b  d8'  88 88       88  `8b   d8'  ,adPPPPP88 88 88 8b       88     `8b d8'   ,adPPPPP88 88 88       88 8PP"""""""  
"8a,   ,d88 88         88,    ,88   `8bd8'  `8bd8'   88 88       88   `8b,d8'   88,    ,88 88 88 "8a,   ,d88      `888'    88,    ,88 88 "8a,   ,a88 "8b,   ,aa  
 `"8bbdP"Y8 88         `"8bbdP"Y8     YP      YP     88 88       88     "8"     `"8bbdP"Y8 88 88  `"8bbdP"Y8       `8'     `"8bbdP"Y8 88  `"YbbdP'Y8  `"Ybbd8"'  
                                                                                                                                                                 
                                                                                                                                                                 

 */

/* 無効な数値を表示する */
void drawInvalidValue(int x, int y, int fontsize = FONT_MEDIUM) {
  String sint = "!--.";
  String sdec = "-";
  if (fontsize == FONT_SMALL) {
    FONT_7SEG_MEDIUM;
  } else if (fontsize == FONT_MEDIUM) {
    FONT_7SEG;
  } else if (fontsize == FONT_BIG) {
    FONT_7SEG_BIG;
  } else {
    FONT_7SEG_LARGE;
  }
  int intWidth = M5.Lcd.textWidth(sint);
  int intHeight = M5.Lcd.fontHeight();
  M5.Lcd.drawString(sint, x, y);
  if (fontsize == FONT_SMALL) {
    FONT_7SEG_TINY;
  } else if (fontsize == FONT_MEDIUM) {
    FONT_7SEG_SMALL;
  } else {
    FONT_7SEG;
  }
  int decHeight = M5.Lcd.fontHeight();
  M5.Lcd.drawString(sdec, x + intWidth, y + intHeight - decHeight - 2);
}

/*
                                                                                                                                                      
         88                                          88888888888 88                              8b           d8          88                         
         88                                          88          88                          ,d  `8b         d8'          88                         
         88                                          88          88                          88   `8b       d8'           88                         
 ,adPPYb,88 8b,dPPYba, ,adPPYYba, 8b      db      d8 88aaaaa     88  ,adPPYba,  ,adPPYYba, MM88MMM `8b     d8' ,adPPYYba, 88 88       88  ,adPPYba,  
a8"    `Y88 88P'   "Y8 ""     `Y8 `8b    d88b    d8' 88"""""     88 a8"     "8a ""     `Y8   88     `8b   d8'  ""     `Y8 88 88       88 a8P_____88  
8b       88 88         ,adPPPPP88  `8b  d8'`8b  d8'  88          88 8b       d8 ,adPPPPP88   88      `8b d8'   ,adPPPPP88 88 88       88 8PP"""""""  
"8a,   ,d88 88         88,    ,88   `8bd8'  `8bd8'   88          88 "8a,   ,a8" 88,    ,88   88,      `888'    88,    ,88 88 "8a,   ,a88 "8b,   ,aa  
 `"8bbdP"Y8 88         `"8bbdP"Y8     YP      YP     88          88  `"YbbdP"'  `"8bbdP"Y8   "Y888     `8'     `"8bbdP"Y8 88  `"YbbdP'Y8  `"Ybbd8"'  
                                                                                                                                                     
                                                                                                                                                     

 */

/* 浮動小数点値を表示する */
void drawFloatValue(float value, int x, int y, int fontsize = FONT_MEDIUM) {
  String str = format_digit(value, 3, 1);
  String sint = str.substring(0, str.length() - 1);
  String sdec = str.substring(str.length() - 1);
  if (fontsize == FONT_SMALL) {
    FONT_7SEG_MEDIUM;
  } else if (fontsize == FONT_MEDIUM) {
    FONT_7SEG;
  } else if (fontsize == FONT_BIG) {
    FONT_7SEG_BIG;
  } else {
    FONT_7SEG_LARGE;
  }
  int intWidth = M5.Lcd.textWidth(sint);
  int intHeight = M5.Lcd.fontHeight();
  M5.Lcd.drawString(sint, x, y);
  if (fontsize == FONT_SMALL) {
    FONT_7SEG_TINY;
  } else if (fontsize == FONT_MEDIUM) {
    FONT_7SEG_SMALL;
  } else {
    FONT_7SEG;
  }
  int decHeight = M5.Lcd.fontHeight();
  M5.Lcd.drawString(sdec, x + intWidth, y + intHeight - decHeight - 2);
}

/*
                                                                                                                                              
         88                                            ,ad8888ba,                        8b           d8          88                         
         88                                           d8"'    `"8b                       `8b         d8'          88                         
         88                                          d8'        `8b                       `8b       d8'           88                         
 ,adPPYb,88 8b,dPPYba, ,adPPYYba, 8b      db      d8 88          88 8b,dPPYba,   ,adPPYba, `8b     d8' ,adPPYYba, 88 88       88  ,adPPYba,  
a8"    `Y88 88P'   "Y8 ""     `Y8 `8b    d88b    d8' 88          88 88P'   `"8a a8P_____88  `8b   d8'  ""     `Y8 88 88       88 a8P_____88  
8b       88 88         ,adPPPPP88  `8b  d8'`8b  d8'  Y8,        ,8P 88       88 8PP"""""""   `8b d8'   ,adPPPPP88 88 88       88 8PP"""""""  
"8a,   ,d88 88         88,    ,88   `8bd8'  `8bd8'    Y8a.    .a8P  88       88 "8b,   ,aa    `888'    88,    ,88 88 "8a,   ,a88 "8b,   ,aa  
 `"8bbdP"Y8 88         `"8bbdP"Y8     YP      YP       `"Y8888Y"'   88       88  `"Ybbd8"'     `8'     `"8bbdP"Y8 88  `"YbbdP'Y8  `"Ybbd8"'  
                                                                                                                                             
                                                                                                                                             

 */

// 1つの値を表示
void drawOneValue(int x, int y, float v1, uint16_t color1,
                  int fontsize = FONT_MEDIUM) {
  M5.Lcd.setTextColor(color1, BGCOLOR);
  drawFloatValue(v1, x, y, fontsize);
}


/*
                                                                                                                           
         88                                           ad88888ba                                                           
         88                                          d8"     "8b                                                          
         88                                          Y8,                                                                  
 ,adPPYb,88 8b,dPPYba, ,adPPYYba, 8b      db      d8 `Y8aaaaa,    ,adPPYba, 8b,dPPYba,  ,adPPYba,  ,adPPYba, 8b,dPPYba,   
a8"    `Y88 88P'   "Y8 ""     `Y8 `8b    d88b    d8'   `"""""8b, a8"     "" 88P'   "Y8 a8P_____88 a8P_____88 88P'   `"8a  
8b       88 88         ,adPPPPP88  `8b  d8'`8b  d8'          `8b 8b         88         8PP""""""" 8PP""""""" 88       88  
"8a,   ,d88 88         88,    ,88   `8bd8'  `8bd8'   Y8a     a8P "8a,   ,aa 88         "8b,   ,aa "8b,   ,aa 88       88  
 `"8bbdP"Y8 88         `"8bbdP"Y8     YP      YP      "Y88888P"   `"Ybbd8"' 88          `"Ybbd8"'  `"Ybbd8"' 88       88  
                                                                                                                          
                                                                                                                          

 */

/* 画面描画 */
void drawScreen(bool value_only = false) {
  if (!value_only) {
    M5.Lcd.pushImage(0, 0, th128Width, th128Height, th128);
    M5.Lcd.setTextFont(0);
    M5.Lcd.setCursor(58, 5);
//    M5.Lcd.print(WiFi.localIP()[3]);
    M5.Lcd.print(website_name);
  }
  if (use_thermo || use_humidity)
    drawOneValue(9, 28, templog->latest(), (use_relay || use_extrelay || use_tplug) ? (relay1->state() ? TFT_RED : TFT_GREEN) : TFT_GREEN, FONT_SMALL);
  if (use_humidity)
    drawOneValue(9, 84, humidlog->latest(), use_fan ? (fan.fan() ? TFT_BLUE : TFT_CYAN) : TFT_CYAN, FONT_SMALL);
}
#endif


/*
                                                                                                                   
                                                                           I8,        8        ,8I 88    ad88 88  
                                                                       ,d  `8b       d8b       d8' ""   d8"   ""  
                                                                       88   "8,     ,8"8,     ,8"       88        
 ,adPPYba,  ,adPPYba,  8b,dPPYba,  8b,dPPYba,   ,adPPYba,  ,adPPYba, MM88MMM Y8     8P Y8     8P   88 MM88MMM 88  
a8"     "" a8"     "8a 88P'   `"8a 88P'   `"8a a8P_____88 a8"     ""   88    `8b   d8' `8b   d8'   88   88    88  
8b         8b       d8 88       88 88       88 8PP""""""" 8b           88     `8a a8'   `8a a8'    88   88    88  
"8a,   ,aa "8a,   ,a8" 88       88 88       88 "8b,   ,aa "8a,   ,aa   88,     `8a8'     `8a8'     88   88    88  
 `"Ybbd8"'  `"YbbdP"'  88       88 88       88  `"Ybbd8"'  `"Ybbd8"'   "Y888    `8'       `8'      88   88    88  
                                                                                                                  
                                                                                                                  

 */

/* WiFiに接続 */
bool connectWifi() {
  bool state = true;
  int i = 0;

  DPRINTLN("Connecting to WiFi");
  // WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), wifipass.c_str());

  // Wait for connection
  DPRINTLN("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DPRINT(".");
    if (i > 30) {
      state = false;
      break;
    }
    i++;
  }
  DPRINTLN("");
  if (state) {
    DPRINT("Connected to ");
    DPRINTLN(ssid);
    DPRINT("IP address: ");
    DPRINTLN(WiFi.localIP());
  } else {
    DPRINTLN("Connection failed.");
  }
  return state;
}

/*
                                                                                                                                                                                                  
                                              I8,        8        ,8I          88           ad88888ba                                                                                            
            ,d                            ,d  `8b       d8b       d8'          88          d8"     "8b                                                                                           
            88                            88   "8,     ,8"8,     ,8"           88          Y8,                                                                                                   
,adPPYba, MM88MMM ,adPPYYba, 8b,dPPYba, MM88MMM Y8     8P Y8     8P  ,adPPYba, 88,dPPYba,  `Y8aaaaa,    ,adPPYba, 8b,dPPYba, 8b       d8  ,adPPYba, 8b,dPPYba,           ,adPPYYba, 8b,dPPYba,   
I8[    ""   88    ""     `Y8 88P'   "Y8   88    `8b   d8' `8b   d8' a8P_____88 88P'    "8a   `"""""8b, a8P_____88 88P'   "Y8 `8b     d8' a8P_____88 88P'   "Y8           ""     `Y8 88P'    "8a  
 `"Y8ba,    88    ,adPPPPP88 88           88     `8a a8'   `8a a8'  8PP""""""" 88       d8         `8b 8PP""""""" 88          `8b   d8'  8PP""""""" 88                   ,adPPPPP88 88       d8  
aa    ]8I   88,   88,    ,88 88           88,     `8a8'     `8a8'   "8b,   ,aa 88b,   ,a8" Y8a     a8P "8b,   ,aa 88           `8b,d8'   "8b,   ,aa 88                   88,    ,88 88b,   ,a8"  
`"YbbdP"'   "Y888 `"8bbdP"Y8 88           "Y888    `8'       `8'     `"Ybbd8"' 8Y"Ybbd8"'   "Y88888P"   `"Ybbd8"' 88             "8"      `"Ybbd8"' 88                   `"8bbdP"Y8 88`YbbdP"'   
                                                                                                                                                                                    88           
                                                                                                                                                              888888888888          88           

 */

/* APモードでWiFi情報を設定 */
void startWebServer_ap() {
  //  webServer.on("/pure.css", handleCss);
  webServer.on("/setap", []() {
    ssid = webServer.arg("ssid");
    wifipass = webServer.arg("pass");
    website_name = webServer.arg("site");
    prefs.putString("hostname", website_name);
    prefs.putString("ssid", ssid);
    prefs.putString("wifipass", wifipass);

    String s = "<h2>Setup complete</h2><p>Device will be connected to \"";
    s += ssid;
    s += "\" after restart.</p><p>Your computer also need to re-connect to "
         "\"";
    s += ssid;
    s += R"=====(
".</p><p><button class="pure-button" onclick="return quitBox();">Close</button></p>
<script>function quitBox() { open(location, '_self').close();return false;};setTimeout("quitBox()",10000);</script>
)=====";
    webServer.send(200, "text/html", makePage("Wi-Fi Settings", s));
    ESP.restart();
    while (1) {
      delay(0);
    }
  });
  webServer.onNotFound([]() {
    int n = WiFi.scanNetworks();
    delay(100);
    String ssidList = "";
    for (int i = 0; i < n; ++i) {
      ssidList += "<option value=\"";
      ssidList += WiFi.SSID(i);
      ssidList += "\">";
      ssidList += WiFi.SSID(i);
      ssidList += "</option>";
    }
    String s = R"=====(
<div class="l-content">
<div class="l-box">
<h3 class="if-head">WiFi Setting</h3>
<p>Please enter your password by selecting the SSID.<br />
You can specify site name for accessing a name like http://aquamonitor.local/</p>
<form class="pure-form pure-form-stacked" method="get" action="setap" name="tm"><label for="ssid">SSID: </label>
<select id="ssid" name="ssid">
)=====";
    s += ssidList;
    s += R"=====(
</select>
<label for="pass">Password: </label><input id="pass" name="pass" length=64 type="password">
<label for="site" >Site name: </label><input id="site" name="site" length=32 type="text" placeholder="Site name">
<button class="pure-button pure-button-primary" type="submit">Submit</button></form>
</div>
</div>
)=====";
    webServer.send(200, "text/html", makePage("Wi-Fi Settings", s));
  });
  webServer.begin();
}

/*
                                                                                                                                                                 
                                              I8,        8        ,8I          88           ad88888ba                                                           
            ,d                            ,d  `8b       d8b       d8'          88          d8"     "8b                                                          
            88                            88   "8,     ,8"8,     ,8"           88          Y8,                                                                  
,adPPYba, MM88MMM ,adPPYYba, 8b,dPPYba, MM88MMM Y8     8P Y8     8P  ,adPPYba, 88,dPPYba,  `Y8aaaaa,    ,adPPYba, 8b,dPPYba, 8b       d8  ,adPPYba, 8b,dPPYba,  
I8[    ""   88    ""     `Y8 88P'   "Y8   88    `8b   d8' `8b   d8' a8P_____88 88P'    "8a   `"""""8b, a8P_____88 88P'   "Y8 `8b     d8' a8P_____88 88P'   "Y8  
 `"Y8ba,    88    ,adPPPPP88 88           88     `8a a8'   `8a a8'  8PP""""""" 88       d8         `8b 8PP""""""" 88          `8b   d8'  8PP""""""" 88          
aa    ]8I   88,   88,    ,88 88           88,     `8a8'     `8a8'   "8b,   ,aa 88b,   ,a8" Y8a     a8P "8b,   ,aa 88           `8b,d8'   "8b,   ,aa 88          
`"YbbdP"'   "Y888 `"8bbdP"Y8 88           "Y888    `8'       `8'     `"Ybbd8"' 8Y"Ybbd8"'   "Y88888P"   `"Ybbd8"' 88             "8"      `"Ybbd8"' 88          
                                                                                                                                                                
                                                                                                                                                                

 */

/* Web server for normal operation */
void startWebServer() {
  DPRINT("Starting Web Server at ");
  DPRINTLN(WiFi.localIP());
  webServer.on("/", handleRoot);
  webServer.on("/pure.css", handleCss);
  webServer.on("/reboot", handleReboot);
  webServer.on("/status", handleStatus);
  webServer.on("/config", handleConfig);
  webServer.on("/function", handleFunction);
  if (use_fan) {
    webServer.on("/fan", handleFan);
  }
  if (use_relay || use_extrelay || use_tplug) {
    webServer.on("/relay", handleRelay);
  }
  webServer.begin();
}

/*
                                                                                                                          
88                                          88 88             ad88888ba                                                  
88                                          88 88            d8"     "8b ,d                 ,d                           
88                                          88 88            Y8,         88                 88                           
88,dPPYba,  ,adPPYYba, 8b,dPPYba,   ,adPPYb,88 88  ,adPPYba, `Y8aaaaa, MM88MMM ,adPPYYba, MM88MMM 88       88 ,adPPYba,  
88P'    "8a ""     `Y8 88P'   `"8a a8"    `Y88 88 a8P_____88   `"""""8b, 88    ""     `Y8   88    88       88 I8[    ""  
88       88 ,adPPPPP88 88       88 8b       88 88 8PP"""""""         `8b 88    ,adPPPPP88   88    88       88  `"Y8ba,   
88       88 88,    ,88 88       88 "8a,   ,d88 88 "8b,   ,aa Y8a     a8P 88,   88,    ,88   88,   "8a,   ,a88 aa    ]8I  
88       88 `"8bbdP"Y8 88       88  `"8bbdP"Y8 88  `"Ybbd8"'  "Y88888P"  "Y888 `"8bbdP"Y8   "Y888  `"YbbdP'Y8 `"YbbdP"'  
                                                                                                                         
                                                                                                                         

 */

/*status 表示*/
void handleStatus() {
  String message = "", argname, argv;
  DynamicJsonBuffer jsonBuffer;

  if (webServer.args() > 0) {
    for (int i = 0; i < webServer.args(); i++) {
      argname = webServer.argName(i);
      argv = webServer.arg(i);
      DPRINT("argname:");
      DPRINT(argname);
      DPRINT(" = ");
      DPRINTLN(argv);
      if (argname == "temperature") {
        if (templog) {
          message = String(templog->average());
        }
      } else if (argname == "humidity") {
        if (humidlog) {
          message = String(humidlog->average());
        }
      } else if (argname == "pressure") {
        if (pressurelog) {
          message = String(pressurelog->average());
        }
      } else if (argname == "co2") {
        if (co2log) {
          message = String(co2log->average());
        }
      } else if (argname == "door") {
        message = String(door);
      } else if (argname == "relay1") {
        message = relay1->state() ? "on" : "off";
      } else if (argname == "relay2") {
        message = relay2->state() ? "on" : "off";
      }
    }
  }
  if (message == "") {
    JsonObject &json = jsonBuffer.createObject();

    if (use_thermo) {
      json["temperature"] = templog->latest();
    }
    if (use_humidity) {
      json["air_humid"] = humidlog->latest();
      if (!use_thermo) {
        json["air_temp"] = templog->latest();
      }
    }
    if (use_pressure) {
      json["pressure"] = pressurelog->latest();
    }
    if (use_tvoc) {
      json["tvoc"] = tvoclog->latest();
    }
    if (use_co2) {
      json["co2"] = co2log->latest();
      if (!use_humidity) {
        json["air_humid"] = humidlog->latest();
      }
      if (!use_humidity && !use_thermo) { 
        json["air_temp"] = templog->latest();
      }
    }
    if (use_doorsensor) {
      json["door_sensor"] = door;
      json["door_count"] = door_count;
      json["state"] = door ? "locked" : "unlocked";
      json["battery"] = 100;
    }
    if (use_fan) {
      json["fan"] = fan.fan();
    }
    if (use_relay) {
      JsonObject &r1 = json.createNestedObject("relay1");
      JsonObject &r2 = json.createNestedObject("relay2");

      r1["name"] = relay1->name();
      r1["state"] = relay1->state() ? "on" : "off";
      r2["name"] = relay2->name();
      r2["state"] = relay2->state() ? "on" : "off";
    }
    if (use_tplug || use_extrelay) {
      JsonObject &r1 = json.createNestedObject("relay1");
      r1["name"] = relay1->name();
      r1["state"] = relay1->state() ? "on" : "off";
    }
    json.printTo(message);
  }
  webServer.send(200, "application/json", message);
}

/*
                                                                                                                                             
88                                          88 88            88888888888                                        88                          
88                                          88 88            88                                           ,d    ""                          
88                                          88 88            88                                           88                                
88,dPPYba,  ,adPPYYba, 8b,dPPYba,   ,adPPYb,88 88  ,adPPYba, 88aaaaa 88       88 8b,dPPYba,   ,adPPYba, MM88MMM 88  ,adPPYba,  8b,dPPYba,   
88P'    "8a ""     `Y8 88P'   `"8a a8"    `Y88 88 a8P_____88 88""""" 88       88 88P'   `"8a a8"     ""   88    88 a8"     "8a 88P'   `"8a  
88       88 ,adPPPPP88 88       88 8b       88 88 8PP""""""" 88      88       88 88       88 8b           88    88 8b       d8 88       88  
88       88 88,    ,88 88       88 "8a,   ,d88 88 "8b,   ,aa 88      "8a,   ,a88 88       88 "8a,   ,aa   88,   88 "8a,   ,a8" 88       88  
88       88 `"8bbdP"Y8 88       88  `"8bbdP"Y8 88  `"Ybbd8"' 88       `"YbbdP'Y8 88       88  `"Ybbd8"'   "Y888 88  `"YbbdP"'  88       88  
                                                                                                                                            
                                                                                                                                            

 */

// Function表示
void handleFunction() {
  String message, argname, argv;
  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();
  int changed = 0;
  for (int i = 0; i < webServer.args(); i++) {
    argname = webServer.argName(i);
    argv = webServer.arg(i);
    DPRINT("argname:");
    DPRINT(argname);
    DPRINT(" = ");
    DPRINTLN(argv);

    if (argname == "use_thermo") {
      use_thermo = argv == "true" ? 1 : 0;
      prefs.putUInt("use_thermo", use_thermo);
      changed++;
    } else if (argname == "use_humidity") {
      use_humidity = argv == "true" ? 1 : 0;
      prefs.putUInt("use_humidity", use_humidity);
      changed++;
    } else if (argname == "use_pressure") {
      use_pressure = argv == "true" ? 1 : 0;
      prefs.putUInt("use_pressure", use_pressure);
      changed++;
    } else if (argname == "use_co2") {
      use_co2 = argv == "true" ? 1 : 0;
      prefs.putUInt("use_co2", use_co2);
      changed++;
    } else if (argname == "use_tvoc") {
      use_tvoc = argv == "true" ? 1 : 0;
      prefs.putUInt("use_tvoc", use_tvoc);
      changed++;
    } else if (argname == "use_doorsensor") {
      use_doorsensor = argv == "true" ? 1 : 0;
      prefs.putUInt("use_doorsensor", use_doorsensor);
      changed++;
    } else if (argname == "use_fan") {
      use_fan = argv == "true" ? 1 : 0;
      prefs.putUInt("use_fan", use_fan);
      changed++;
    } else if (argname == "use_relay") {
      use_relay = argv == "true" ? 1 : 0;
      prefs.putUInt("use_relay", use_relay);
      changed++;
    } else if (argname == "use_extrelay") {
      use_extrelay = argv == "true" ? 1 : 0;
      prefs.putUInt("use_extrelay", use_extrelay);
      changed++;
    } else if (argname == "use_tplug") {
      use_tplug = argv == "true" ? 1 : 0;
      prefs.putUInt("use_tplug", use_tplug);
      changed++;
    }
  }

  json["use_thermo"] = use_thermo > 0 ? "true" : "false";
  json["use_humidity"] = use_humidity > 0 ? "true" : "false";
  json["use_pressure"] = use_pressure > 0 ? "true" : "false";
  json["use_co2"] = use_co2 > 0 ? "true" : "false";
  json["use_tvoc"] = use_tvoc > 0 ? "true" : "false";
  json["use_doorsensor"] = use_doorsensor > 0 ? "true" : "false";
  json["use_fan"] = use_fan > 0 ? "true" : "false";
  json["use_relay"] = use_relay > 0 ? "true" : "false";
  json["use_extrelay"] = use_extrelay > 0 ? "true" : "false";
  json["use_tplug"] = use_tplug > 0 ? "true" : "false";

  json.printTo(message);
  webServer.send(200, "application/json", message);
  if (changed > 0) {
    delay(500);
    ESP.restart();
    while (1) {
      delay(0);
    }
  }
}

/*
                                                                                                                           
88                                          88 88              ,ad8888ba,                            ad88 88              
88                                          88 88             d8"'    `"8b                          d8"   ""              
88                                          88 88            d8'                                    88                    
88,dPPYba,  ,adPPYYba, 8b,dPPYba,   ,adPPYb,88 88  ,adPPYba, 88             ,adPPYba,  8b,dPPYba, MM88MMM 88  ,adPPYb,d8  
88P'    "8a ""     `Y8 88P'   `"8a a8"    `Y88 88 a8P_____88 88            a8"     "8a 88P'   `"8a  88    88 a8"    `Y88  
88       88 ,adPPPPP88 88       88 8b       88 88 8PP""""""" Y8,           8b       d8 88       88  88    88 8b       88  
88       88 88,    ,88 88       88 "8a,   ,d88 88 "8b,   ,aa  Y8a.    .a8P "8a,   ,a8" 88       88  88    88 "8a,   ,d88  
88       88 `"8bbdP"Y8 88       88  `"8bbdP"Y8 88  `"Ybbd8"'   `"Y8888Y"'   `"YbbdP"'  88       88  88    88  `"YbbdP"Y8  
                                                                                                              aa,    ,88  
                                                                                                               "Y8bbdP"   

 */

// Config表示
void handleConfig() {
  String message, argname, argv;
  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();

  for (int i = 0; i < webServer.args(); i++) {
    argname = webServer.argName(i);
    argv = webServer.arg(i);
    DPRINT("argname:");
    DPRINT(argname);
    DPRINT(" = ");
    DPRINTLN(argv);

    if (argname == "hostname") {
      website_name = argv;
      prefs.putString("hostname", website_name);
      WiFi.setHostname(website_name.c_str());
    } else if (argname == "enable_push") {
      use_postdb = argv == "true" ? 1 : 0;
      prefs.putUInt("use_postdb", use_postdb);
    } else if (argname == "manage_fan") {
      manage_fan = argv == "true" ? 1 : 0;
      prefs.putUInt("manage_fan", manage_fan);
    } else if (argname == "target_humid") {
      fan.targetHumid(argv.toFloat());
      prefs.putFloat("target_humid", fan.targetHumid());
    } else if (argname == "temp_id") {
      temp_id = argv;
      prefs.putString("templabel", temp_id);
    } else if (argname == "humidity_id") {
      humid_id = argv;
      prefs.putString("humiditylabel", humid_id);
    } else if (argname == "pressure_id") {
      press_id = argv;
      prefs.putString("pressurelabel", press_id);
    } else if (argname == "co2_id") {
      co2_id = argv;
      prefs.putString("co2label", co2_id);
    } else if (argname == "tvoc_id") {
      tvoc_id = argv;
      prefs.putString("tvoclabel", tvoc_id);
    } else if (argname == "door_id") {
      door_id = argv;
      prefs.putString("doorlabel", door_id);
    } else if (argname == "url_endpoint") {
      url_endpoint = argv;
      prefs.putString("url_endpoint", url_endpoint);
    } else if (argname == "url_extrelay") {
      url_extrelay = argv;
      prefs.putString("url_extrelay", url_extrelay);
    } else if (argname == "host_tplug") {
      host_tplug = argv;
      prefs.putString("host_tplug", host_tplug);
    } else if (argname == "manage_relay1") {
      manage_relay1 = (argv == "true") ? 1 : 0;
      prefs.putUChar("manage_relay1", manage_relay1);
    } else if (argname == "manage_relay2") {
      manage_relay2 = (argv == "true") ? 1 : 0;
      prefs.putUChar("manage_relay2", manage_relay2);
    } else if (argname == "relay1_on_temp") {
      relay1->onTemp(argv.toFloat());
      prefs.putFloat("r1_on_temp", relay1->onTemp());
    } else if (argname == "relay1_off_temp") {
      relay1->offTemp(argv.toFloat());
      prefs.putFloat("r1_off_temp", relay1->offTemp());
    } else if (argname == "relay2_on_temp") {
      relay2->onTemp(argv.toFloat());
      prefs.putFloat("r2_on_temp", relay2->onTemp());
    } else if (argname == "relay2_off_temp") {
      relay2->offTemp(argv.toFloat());
      prefs.putFloat("r2_off_temp", relay2->offTemp());
    } else if (argname == "relay1_id") {
      relay1->name(argv);
      prefs.putString("relay1_id", relay1->name());
    } else if (argname == "relay2_id") {
      relay2->name(argv);
      prefs.putString("relay2_id", relay2->name());
    } else if (argname == "fan_id") {
      fan.name(argv);
      prefs.putString("fan_id", fan.name());
    }
  }

  json["hostname"] = website_name;
  json["enable_push"] = use_postdb > 0 ? "true" : "false";
  json["url_endpoint"] = url_endpoint;
  if (use_fan) {
    json["manage_fan"] = manage_fan > 0 ? "true" : "false";
    json["target_humid"] = fan.targetHumid();
    json["fan_id"] = fan.name();
  }
  if (use_relay) {
    json["manage_relay1"] = manage_relay1 > 0 ? "true" : "false";
    json["relay1_on_temp"] = relay1->onTemp();
    json["relay1_off_temp"] = relay1->offTemp();
    json["manage_relay2"] = manage_relay2 > 0 ? "true" : "false";
    json["relay2_on_temp"] = relay2->onTemp();
    json["relay2_off_temp"] = relay2->offTemp();
    json["relay1_id"] = relay1->name();
    json["relay2_id"] = relay2->name();
  }
  if (use_extrelay) {
    json["url_extrelay"] = url_extrelay;
    json["manage_relay1"] = manage_relay1 > 0 ? "true" : "false";
    json["relay1_on_temp"] = relay1->onTemp();
    json["relay1_off_temp"] = relay1->offTemp();
    json["relay1_id"] = relay1->name();
  }
  if (use_tplug) {
    json["host_tplug"] = host_tplug;
    json["manage_relay1"] = manage_relay1 > 0 ? "true" : "false";
    json["relay1_on_temp"] = relay1->onTemp();
    json["relay1_off_temp"] = relay1->offTemp();
    json["relay1_id"] = relay1->name();
  }
  if (use_humidity) {
    json["temp_id"] = temp_id;
    json["humidity_id"] = humid_id;
  }
  if (use_pressure) {
    json["pressure_id"] = press_id;
  }
  if (use_co2) {
    json["co2_id"] = co2_id;
  }
  if (use_tvoc) {
    json["tvoc_id"] = tvoc_id;
  }
  if (use_doorsensor) {
    json["door_id"] = door_id;
  }

  json.printTo(message);
  webServer.send(200, "application/json", message);
}

/*
                                                                                              
88                                          88 88            88888888888                     
88                                          88 88            88                              
88                                          88 88            88                              
88,dPPYba,  ,adPPYYba, 8b,dPPYba,   ,adPPYb,88 88  ,adPPYba, 88aaaaa ,adPPYYba, 8b,dPPYba,   
88P'    "8a ""     `Y8 88P'   `"8a a8"    `Y88 88 a8P_____88 88""""" ""     `Y8 88P'   `"8a  
88       88 ,adPPPPP88 88       88 8b       88 88 8PP""""""" 88      ,adPPPPP88 88       88  
88       88 88,    ,88 88       88 "8a,   ,d88 88 "8b,   ,aa 88      88,    ,88 88       88  
88       88 `"8bbdP"Y8 88       88  `"8bbdP"Y8 88  `"Ybbd8"' 88      `"8bbdP"Y8 88       88  
                                                                                             
                                                                                             

 */

// ファン制御
void handleFan() {
  String argname, argv;
  if (!use_fan) return;
  int p = -1;
  for (int i = 0; i < webServer.args(); i++) {
    argname = webServer.argName(i);
    argv = webServer.arg(i);
    if (argname == "power") {
      p = argv.toInt();
    }
  }
  if (p >= 0) {
    fan.fan(p);
  }
  handleStatus();
}
/*
                                                                                                               
88                                          88 88            88888888ba            88                         
88                                          88 88            88      "8b           88                         
88                                          88 88            88      ,8P           88                         
88,dPPYba,  ,adPPYYba, 8b,dPPYba,   ,adPPYb,88 88  ,adPPYba, 88aaaaaa8P' ,adPPYba, 88 ,adPPYYba, 8b       d8  
88P'    "8a ""     `Y8 88P'   `"8a a8"    `Y88 88 a8P_____88 88""""88'  a8P_____88 88 ""     `Y8 `8b     d8'  
88       88 ,adPPPPP88 88       88 8b       88 88 8PP""""""" 88    `8b  8PP""""""" 88 ,adPPPPP88  `8b   d8'   
88       88 88,    ,88 88       88 "8a,   ,d88 88 "8b,   ,aa 88     `8b "8b,   ,aa 88 88,    ,88   `8b,d8'    
88       88 `"8bbdP"Y8 88       88  `"8bbdP"Y8 88  `"Ybbd8"' 88      `8b `"Ybbd8"' 88 `"8bbdP"Y8     Y88'     
                                                                                                     d8'      
                                                                                                    d8'       

 */

// relay制御
void handleRelay() {
  String message, argname, argv;
  if (!use_relay && !use_extrelay && !use_tplug) return;
  for (int i = 0; i < webServer.args(); i++) {
    argname = webServer.argName(i);
    argv = webServer.arg(i);
    DPRINT("argname:");
    DPRINT(argname);
    DPRINT(" = ");
    DPRINTLN(argv);
    if (argname == "1") {
      if (argv == "on") {
        relay1->on();
        if (use_relay) {
          pixel_color = relay1->pixel(pixel_color);
          DRAWPIX(pixel_color);
        }
        prefs.putUChar("relay1_status", relay1->state());
        if (!use_extrelay) {
          post_note(relay1->name(), "on");
        }
      } else if (argv == "off") {
        relay1->off();
        if (use_relay) {
          pixel_color = relay1->pixel(pixel_color);
          DRAWPIX(pixel_color);
        }
        prefs.putUChar("relay1_status", relay1->state());
        if (!use_extrelay) {
          post_note(relay1->name(), "off");
        }
      }
    } else if (argname == "2") {
      if (use_relay) {
        if (argv == "on") {
          relay2->on();
          if (use_relay) {
            pixel_color = relay2->pixel(pixel_color);
            DRAWPIX(pixel_color);
          }
          prefs.putUChar("relay2_status", relay2->state());
          if (!use_extrelay) {
            post_note(relay2->name(), "on");
          }
        } else if (argv == "off") {
          relay2->off();
          if (use_relay) {
            pixel_color = relay2->pixel(pixel_color);
            DRAWPIX(pixel_color);
          }
          prefs.putUChar("relay2_status", relay2->state());
          if (!use_extrelay) {
            post_note(relay2->name(), "off");
          }
        }
      }
    }
  }
  handleStatus();
}

/*******************************************
 *
 * HEREAFTER, LESS TO BE CHANGED
 *
 *******************************************
 */

/*
                                                                                                       
                                                                           88                         
                                    ,d                                     88                         
                                    88                                     88                         
8b,dPPYba,   ,adPPYba,  ,adPPYba, MM88MMM           8b       d8 ,adPPYYba, 88 88       88  ,adPPYba,  
88P'    "8a a8"     "8a I8[    ""   88              `8b     d8' ""     `Y8 88 88       88 a8P_____88  
88       d8 8b       d8  `"Y8ba,    88               `8b   d8'  ,adPPPPP88 88 88       88 8PP"""""""  
88b,   ,a8" "8a,   ,a8" aa    ]8I   88,               `8b,d8'   88,    ,88 88 "8a,   ,a88 "8b,   ,aa  
88`YbbdP"'   `"YbbdP"'  `"YbbdP"'   "Y888               "8"     `"8bbdP"Y8 88  `"YbbdP'Y8  `"Ybbd8"'  
88                                                                                                    
88                                       888888888888                                                 

 */

// サーバに値を登録
void post_value(String label, float value) {
  char params[128];
  if (use_postdb) {
    String apiurl = url_endpoint + "/aqua/add";
    String ipaddress = WiFi.localIP().toString();

    DPRINTLN(apiurl);

    http.begin(apiurl.c_str());
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    sprintf(params, "label=%s&value=%f", label.c_str(), value);
    int httpCode = http.POST(params);
    if (httpCode > 0) {
      DPRINTF("[HTTP] POST value... code: %d\n", httpCode);
      DPRINTF("[HTTP] url: %s params: %s\n", apiurl.c_str(), params);
      //    valid_db_server = true;
    } else {
      DPRINTF("[ERROR] POST value... no connection or no HTTP server: %s\n",
              http.errorToString(httpCode).c_str());
      DPRINTF("[ERROR] url: %s params: %s\n", apiurl.c_str(), params);
      //    valid_db_server = false;
    }
    String s = http.getString();
    DPRINTLN(s);
    http.end();
  }
}

/*
                                                                                               
                                                                                              
                                    ,d                                       ,d               
                                    88                                       88               
8b,dPPYba,   ,adPPYba,  ,adPPYba, MM88MMM           8b,dPPYba,   ,adPPYba, MM88MMM ,adPPYba,  
88P'    "8a a8"     "8a I8[    ""   88              88P'   `"8a a8"     "8a  88   a8P_____88  
88       d8 8b       d8  `"Y8ba,    88              88       88 8b       d8  88   8PP"""""""  
88b,   ,a8" "8a,   ,a8" aa    ]8I   88,             88       88 "8a,   ,a8"  88,  "8b,   ,aa  
88`YbbdP"'   `"YbbdP"'  `"YbbdP"'   "Y888           88       88  `"YbbdP"'   "Y888 `"Ybbd8"'  
88                                                                                            
88                                       888888888888                                         

 */

// サーバに文字列を登録
void post_note(String label, String value) {
  char params[128];
  if (use_postdb) {
    String apiurl = url_endpoint + "/aqua/add";
    String ipaddress = WiFi.localIP().toString();

    DPRINTLN(apiurl);

    http.begin(apiurl.c_str());
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    sprintf(params, "label=%s&note=%s", label.c_str(), value.c_str());
    int httpCode = http.POST(params);
    if (httpCode > 0) {
      DPRINTF("[HTTP] POST note... code: %d\n", httpCode);
      DPRINTF("[HTTP] url: %s params: %s\n", apiurl.c_str(), params);
      //    valid_db_server = true;
    } else {
      DPRINTF("[ERROR] POST note... no connection or no HTTP server: %s\n",
              http.errorToString(httpCode).c_str());
      DPRINTF("[ERROR] url: %s params: %s\n", apiurl.c_str(), params);
      //    valid_db_server = false;
    }
    String s = http.getString();
    DPRINTLN(s);
    http.end();
  }
}

/*
                                                                                              
                                                               88 88                         
                                    ,d                         88 ""                         
                                    88                         88                            
8b,dPPYba,   ,adPPYba,  ,adPPYba, MM88MMM           ,adPPYYba, 88 88 8b       d8  ,adPPYba,  
88P'    "8a a8"     "8a I8[    ""   88              ""     `Y8 88 88 `8b     d8' a8P_____88  
88       d8 8b       d8  `"Y8ba,    88              ,adPPPPP88 88 88  `8b   d8'  8PP"""""""  
88b,   ,a8" "8a,   ,a8" aa    ]8I   88,             88,    ,88 88 88   `8b,d8'   "8b,   ,aa  
88`YbbdP"'   `"YbbdP"'  `"YbbdP"'   "Y888           `"8bbdP"Y8 88 88     "8"      `"Ybbd8"'  
88                                                                                           
88                                       888888888888                                        

 */

// サーバに生存情報を登録
void post_alive() {
  char params[128];

  if (use_postdb) {
    String apiurl = url_endpoint + "/aqua/alive";
    String ipaddress = WiFi.localIP().toString();

    DPRINTLN(apiurl);

    http.begin(apiurl.c_str());
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    sprintf(params, "ipaddress=%s&label=%s", ipaddress.c_str(),
            website_name.c_str());
    int httpCode = http.POST(params);
    if (httpCode > 0) {
      DPRINTF("[HTTP] POST alive... code: %d\n", httpCode);
      DPRINTF("[HTTP] url: %s params: %s\n", apiurl.c_str(), params);
      //    valid_db_server = true;
    } else {
      DPRINTF("[ERROR] POST alive... no connection or no HTTP server: %s\n",
              http.errorToString(httpCode).c_str());
      DPRINTF("[ERROR] url: %s params: %s\n", apiurl.c_str(), params);
      //    valid_db_server = false;
    }
    String s = http.getString();
    DPRINTLN(s);
    http.end();
  }
}

void handleRoot() { sendFs("/index.html", "text/html"); }

void handleCss() { sendFs("/pure.css", "text/css"); }

void handleReboot() {
  String message;
  message = "{reboot:\"done\"}";
  webServer.send(200, "application/json", message);
  delay(500);
  ESP.restart();
  while (1) {
    delay(0);
  }
}

void sendFs(String path, String contentType) {
  if (SPIFFS.exists(path)) {
    File file = SPIFFS.open(path, "r");
    size_t sent = webServer.streamFile(file, contentType);
    file.close();
  } else {
    webServer.send(500, "text/plain", "BAD PATH");
  }
}

/*
                                                                                                  
                              88                  88888888ba                                     
                              88                  88      "8b                                    
                              88                  88      ,8P                                    
88,dPYba,,adPYba,  ,adPPYYba, 88   ,d8  ,adPPYba, 88aaaaaa8P' ,adPPYYba,  ,adPPYb,d8  ,adPPYba,  
88P'   "88"    "8a ""     `Y8 88 ,a8"  a8P_____88 88""""""'   ""     `Y8 a8"    `Y88 a8P_____88  
88      88      88 ,adPPPPP88 8888[    8PP""""""" 88          ,adPPPPP88 8b       88 8PP"""""""  
88      88      88 88,    ,88 88`"Yba, "8b,   ,aa 88          88,    ,88 "8a,   ,d88 "8b,   ,aa  
88      88      88 `"8bbdP"Y8 88   `Y8a `"Ybbd8"' 88          `"8bbdP"Y8  `"YbbdP"Y8  `"Ybbd8"'  
                                                                          aa,    ,88             
                                                                           "Y8bbdP"              

 */

//makepage
String makePage(String title, String contents) {
  String s = R"=====(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<link rel="stylesheet" href="/pure.css">
)=====";
  s += "<title>";
  s += title;
  s += "</title></head><body>";
  s += contents;
  s += R"=====(
<div class="footer l-box">
<p>atom env sensor by @omzn 2023</p>
</div>
)=====";
  s += "</body></html>";
  return s;
}

/*
                                                                                                   
                       88 88888888ba,                                              88             
                       88 88      `"8b                                             88             
                       88 88        `8b                                            88             
88       88 8b,dPPYba, 88 88         88  ,adPPYba,  ,adPPYba,  ,adPPYba,   ,adPPYb,88  ,adPPYba,  
88       88 88P'   "Y8 88 88         88 a8P_____88 a8"     "" a8"     "8a a8"    `Y88 a8P_____88  
88       88 88         88 88         8P 8PP""""""" 8b         8b       d8 8b       88 8PP"""""""  
"8a,   ,a88 88         88 88      .a8P  "8b,   ,aa "8a,   ,aa "8a,   ,a8" "8a,   ,d88 "8b,   ,aa  
 `"YbbdP'Y8 88         88 88888888Y"'    `"Ybbd8"'  `"Ybbd8"'  `"YbbdP"'   `"8bbdP"Y8  `"Ybbd8"'  
                                                                                                  
                                                                                                  

 */

// urldecode
String urlDecode(String input) {
  String s = input;
  s.replace("%20", " ");
  s.replace("+", " ");
  s.replace("%21", "!");
  s.replace("%22", "\"");
  s.replace("%23", "#");
  s.replace("%24", "$");
  s.replace("%25", "%");
  s.replace("%26", "&");
  s.replace("%27", "\'");
  s.replace("%28", "(");
  s.replace("%29", ")");
  s.replace("%30", "*");
  s.replace("%31", "+");
  s.replace("%2C", ",");
  s.replace("%2E", ".");
  s.replace("%2F", "/");
  s.replace("%2C", ",");
  s.replace("%3A", ":");
  s.replace("%3A", ";");
  s.replace("%3C", "<");
  s.replace("%3D", "=");
  s.replace("%3E", ">");
  s.replace("%3F", "?");
  s.replace("%40", "@");
  s.replace("%5B", "[");
  s.replace("%5C", "\\");
  s.replace("%5D", "]");
  s.replace("%5E", "^");
  s.replace("%5F", "-");
  s.replace("%60", "`");
  return s;
}

/*
                                                                                                                                                                           
                                                           db                            88             88                           ,ad8888ba, 888888888888   db         
                       ,d                                 d88b                           88             ""                          d8"'    `"8b     88       d88b        
                       88                                d8'`8b                          88                                        d8'        `8b    88      d8'`8b       
,adPPYba,  ,adPPYba, MM88MMM 88       88 8b,dPPYba,     d8'  `8b     8b,dPPYba,  ,adPPYb,88 88       88 88 8b,dPPYba,   ,adPPYba,  88          88    88     d8'  `8b      
I8[    "" a8P_____88   88    88       88 88P'    "8a   d8YaaaaY8b    88P'   "Y8 a8"    `Y88 88       88 88 88P'   `"8a a8"     "8a 88          88    88    d8YaaaaY8b     
 `"Y8ba,  8PP"""""""   88    88       88 88       d8  d8""""""""8b   88         8b       88 88       88 88 88       88 8b       d8 Y8,        ,8P    88   d8""""""""8b    
aa    ]8I "8b,   ,aa   88,   "8a,   ,a88 88b,   ,a8" d8'        `8b  88         "8a,   ,d88 "8a,   ,a88 88 88       88 "8a,   ,a8"  Y8a.    .a8P     88  d8'        `8b   
`"YbbdP"'  `"Ybbd8"'   "Y888  `"YbbdP'Y8 88`YbbdP"' d8'          `8b 88          `"8bbdP"Y8  `"YbbdP'Y8 88 88       88  `"YbbdP"'    `"Y8888Y"'      88 d8'          `8b  
                                         88                                                                                                                               
                                         88                                                                                                                               

 */
void setupArduinoOTA() {
  // ArduinoOTA.setPort(8266);
  //  Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(website_name.c_str());
  ArduinoOTA.onStart([]() {
    String type;
    rtcint = 0;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
    // using SPIFFS.end()
    DPRINTLN("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() { DPRINTLN("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int pgs = (progress / (total / 100));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    if (error == OTA_AUTH_ERROR) {
      DPRINTLN("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      DPRINTLN("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      DPRINTLN("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      DPRINTLN("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      DPRINTLN("End Failed");
    }
  });
  ArduinoOTA.begin();
  delay(100);
}

/*
                                                       
                                                      
                       ,d                             
                       88                             
,adPPYba,  ,adPPYba, MM88MMM 88       88 8b,dPPYba,   
I8[    "" a8P_____88   88    88       88 88P'    "8a  
 `"Y8ba,  8PP"""""""   88    88       88 88       d8  
aa    ]8I "8b,   ,aa   88,   "8a,   ,a88 88b,   ,a8"  
`"YbbdP"'  `"Ybbd8"'   "Y888  `"YbbdP'Y8 88`YbbdP"'   
                                         88           
                                         88           

 */

// setup
void setup() {
#if defined(ARDUINO_M5Stack_ATOM)
  M5.begin(true,true,true);
#elif defined(ARDUINO_M5Stack_ATOMS3)
  M5.begin(true,true,true,true);
//  M5.IMU.begin();
#else
  M5.begin();
#endif
  DSERIALBEGIN(115200);

  // ATOM LiteのM5.begin()でI2Cがデフォルトでenableになっているので，一旦止める．
  // ATOM S3 のM5.begin()ではI2Cがデフォルトでdisableになっているので，有効にした上で，一旦止める．
  Wire.end(); 
  Wire.begin(PIN_SDA, PIN_SCL);

  prefs.begin("env", false);
  //  prefs.clear();
  website_name = prefs.getString("hostname", website_name);
  ssid = prefs.getString("ssid", ssid);
  wifipass = prefs.getString("wifipass", wifipass);

#if defined(ARDUINO_M5Stack_ATOMS3)
#if HAS_DISPLAY
  lcd_rotation = prefs.getUChar("lcd_rotate",0);
  M5.Lcd.setRotation(lcd_rotation);
#endif
#endif

  url_endpoint = prefs.getString("url_endpoint", url_endpoint);
  use_postdb = prefs.getUInt("use_postdb", use_postdb);
  use_thermo = prefs.getUInt("use_thermo", use_thermo);
  use_humidity = prefs.getUInt("use_humidity", use_humidity);
  use_pressure = prefs.getUInt("use_pressure", use_pressure);
  use_doorsensor = prefs.getUInt("use_doorsensor", use_doorsensor);
  use_co2 = prefs.getUInt("use_co2", use_co2);
  use_tvoc = prefs.getUInt("use_tvoc", use_tvoc);

  use_fan = prefs.getUInt("use_fan", use_fan);
  manage_fan = prefs.getUInt("manage_fan", manage_fan);
  fan.targetHumid(prefs.getFloat("target_humid", 100));
  fan.name(prefs.getString("fan_id", "fan"));

  use_relay = prefs.getUInt("use_relay", use_relay);
  manage_relay1 = prefs.getUChar("manage_relay1", manage_relay1);
  manage_relay2 = prefs.getUChar("manage_relay2", manage_relay2);
  use_extrelay = prefs.getUInt("use_extrelay", use_extrelay);
  url_extrelay = prefs.getString("url_extrelay", url_extrelay);
  use_tplug = prefs.getUInt("use_tplug", use_tplug);

  temp_id = prefs.getString("templabel", temp_id);
  humid_id = prefs.getString("humiditylabel", humid_id);
  press_id = prefs.getString("pressurelabel", press_id);
  co2_id = prefs.getString("co2label", co2_id);
  tvoc_id = prefs.getString("tvoclabel", tvoc_id);
  door_id = prefs.getString("doorlabel", door_id);

  if (use_relay) {
    relay1 = new Relay(RELAY1_PIN, 0xff0000);
    relay2 = new Relay(RELAY2_PIN, 0x00ff00);
    pinMode(RELAY1_PIN, OUTPUT);
    pinMode(RELAY2_PIN, OUTPUT);
    relay1->name(prefs.getString("relay1_id", "relay1"));
    relay2->name(prefs.getString("relay2_id", "relay2"));
    relay1->relay((bool)prefs.getUChar("relay1_state", 0));
    pixel_color = relay1->pixel(pixel_color);
    DRAWPIX(pixel_color);
    relay2->relay(prefs.getUChar("relay2_state", 0));
    pixel_color = relay2->pixel(pixel_color);
    DRAWPIX(pixel_color);
    relay1->onTemp(prefs.getFloat("r1_on_temp", 0));
    relay1->offTemp(prefs.getFloat("r1_off_temp", 0));
    relay2->onTemp(prefs.getFloat("r2_on_temp", 0));
    relay2->offTemp(prefs.getFloat("r2_off_temp", 0));
  }

  delay(100);

  // WiFi.setHostname(website_name.c_str());
  // wifiManager.setTimeout(60);
  // wifiManager.autoConnect(apSSID);
  if (!connectWifi()) {
    WiFi.mode(WIFI_AP);
    DPRINTLN("Wifi AP mode");
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(apSSID);
    dnsServer.start(53, "*", apIP);
    wifi_ap_mode = true;
  } else {
    // WiFi.setSleep(false);
    ArduinoOTA.setHostname(website_name.c_str());
    ArduinoOTA
        .onStart([]() {
          String type;
          if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
          else  // U_SPIFFS
            type = "filesystem";
          // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
          // using SPIFFS.end()
          DPRINTLN("Start updating " + type);
        })
        .onEnd([]() { DPRINTLN("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total) {
          DPRINTF("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
          DPRINTF("Error[%u]: ", error);
          if (error == OTA_AUTH_ERROR)
            DPRINTLN("Auth Failed");
          else if (error == OTA_BEGIN_ERROR)
            DPRINTLN("Begin Failed");
          else if (error == OTA_CONNECT_ERROR)
            DPRINTLN("Connect Failed");
          else if (error == OTA_RECEIVE_ERROR)
            DPRINTLN("Receive Failed");
          else if (error == OTA_END_ERROR)
            DPRINTLN("End Failed");
        });
    ArduinoOTA.begin();
    DPRINTLN("ArduinoOTA began");
  }
  DPRINTLN("WiFi connected.");
  DPRINTLN(WiFi.localIP());

  if (use_extrelay) {
    relay1 = new Relay(url_extrelay);
    relay1->name(prefs.getString("relay1_id", "relay1"));
    relay1->relay(prefs.getUChar("relay1_state", 0));
    relay1->onTemp(prefs.getFloat("r1_on_temp", 0));
    relay1->offTemp(prefs.getFloat("r1_off_temp", 0));
  }
  if (use_tplug) {
    host_tplug = prefs.getString("host_tplug", host_tplug);
    tplug = new TPLinkSmartPlug();
    tplug->begin(client,udp);
    tplug->setTarget(host_tplug);
    relay1 = new Relay(tplug);
    relay1->name(prefs.getString("relay1_id", "relay1"));
    relay1->relay(prefs.getUChar("relay1_state", 0));
    relay1->onTemp(prefs.getFloat("r1_on_temp", 0));
    relay1->offTemp(prefs.getFloat("r1_off_temp", 0));
  }

  if (use_fan) {
    pinMode(FAN_PIN, OUTPUT);
    analogWrite(FAN_PIN, 0);
    //digitalWrite(FAN_PIN, LOW);
  }

  if (use_pressure) {
#if defined(USE_HAT_ENV3) || defined(USE_GROVE_ENV3)
    qmp6988.init();
    pressurelog = new DataLog();
    //  if (!bmp.begin(BMP280_ADDRESS_ALT)) {
    //     DPRINTLN("BMP280 is not found");
    //   } else {
    //     DPRINTLN("Sensors are found");
    //   }
#endif
  }
  if (use_humidity) {
    if (!use_thermo) {
      templog = new DataLog();
    }
    humidlog = new DataLog(6 * 5 * 5);
  }
  if (use_tvoc) {
#ifdef USE_SGP30
    if (! sgp.begin()){
      DPRINTLN("SGP30 Sensor not found :(");
    }
    DPRINTLN("SGP30 Sensor Started :)");
#endif
    tvoclog = new DataLog(6 * 5 * 5);
  }
  if (use_co2) {
    co2log = new DataLog();
    if (!use_humidity && !use_thermo) {
      templog = new DataLog();
    }
    if (!use_humidity) {
      humidlog = new DataLog();
    }
    //    M5.dis.drawpix(0, 0x7f0000);
#ifdef USE_MHZ19
    mySerial.begin(MHZ19_BAUDRATE, SERIAL_8N1, MHZ19_PIN_RX, MHZ19_PIN_TX);
    mhz19.begin(mySerial);
    mhz19.setAutoCalibration(false);
#endif
#ifdef USE_SCD4X
    if (!co2sensor.begin()) {
      DPRINTLN("SCD40 is not found");
    } else {
      DPRINTLN("SCD40 is found");
    }
#endif
  }
  if (use_thermo) {
    templog = new DataLog();
#ifdef DS18B20
    ds18b20.begin();
    if (ds18b20.getDS18Count() == 0) {
      DPRINTLN("DS18B20 is not found");
    }
#endif
  }
  if (use_doorsensor) {
    pinMode(DOORSENSOR_PIN, INPUT_PULLUP);
  }
  DRAWSCREEN();

#ifdef USE_DEEPSLEEP
  esp_sleep_enable_timer_wakeup(10 * 60 * 1000 * 1000);  // wakeup every 10 mins
  esp_deep_sleep_start();
#else
  if (!wifi_ap_mode) {
    startWebServer();
  } else {
    startWebServer_ap();
    wifi_connect_timer = millis();
  }
#endif
}

/*
                                         
88                                      
88                                      
88                                      
88  ,adPPYba,   ,adPPYba,  8b,dPPYba,   
88 a8"     "8a a8"     "8a 88P'    "8a  
88 8b       d8 8b       d8 88       d8  
88 "8a,   ,a8" "8a,   ,a8" 88b,   ,a8"  
88  `"YbbdP"'   `"YbbdP"'  88`YbbdP"'   
                           88           
                           88           

 */

// loop
void loop() {
#ifndef USE_DEEPSLEEP
  webServer.handleClient();
  if (wifi_ap_mode) {
    dnsServer.processNextRequest();
    if (millis() - wifi_connect_timer > 180 * 1000) {
      ESP.restart();
      while (1) {
        delay(1);
      }
    }
    return;
  } else {
    ArduinoOTA.handle();
  }

  ArduinoOTA.handle();

  M5.update();

  if (M5.Btn.wasReleased()) {
    //    M5.dis.drawpix(0, 0x007f00);
    //    if (use_co2) {
    //      mhz19.calibrateZero();
    //      delay(1000);
    //    }
    //    M5.dis.drawpix(0, 0x7f0000);
#if defined(ARDUINO_M5Stack_ATOMS3)
  #if HAS_DISPLAY 
    lcd_rotation++;
    lcd_rotation %= 4;
    M5.Lcd.setRotation(lcd_rotation);
    DRAWSCREEN();
    prefs.putUChar("lcd_rotate",lcd_rotation);
  #endif
#endif
  }

  if (WiFi.status() != WL_CONNECTED) {
    ESP.restart();
    while (1) {
      delay(0);
    }
  }

  if (millis() - p_millis > 1000) {
    p_millis = millis();
    rtcint = 1;
  }

  if (rtcint == 1) {
    rtcint = 0;

    getDoor();

    if (timer_count % 2 == 0) {
      getHumid();
      if (manage_fan && use_humidity) {
        if (fan.manageByHumid(humidlog->latest())) {
          if (fan.record()) 
            post_note(fan.name(), fan.fan() ? "on" : "off");
        }
      }
    }

    if (timer_count % (10) == 0) {
      if (use_relay || use_extrelay || use_tplug) {
        if (manage_relay1 && (use_thermo || use_humidity || use_co2)) {
          if (relay1->manageByTemperature(templog->latest())) {
            if (use_relay) {
              pixel_color = relay1->pixel(pixel_color);
              DRAWPIX(pixel_color);
            }
            prefs.putUChar("relay1_status", relay1->state());
            if (!use_extrelay) {
              post_note(relay1->name(), relay1->state() ? "on" : "off");
            }
          }
        }
      }
      if (use_relay) {
        if (manage_relay2 && (use_thermo || use_humidity || use_co2)) {
          if (relay2->manageByTemperature(templog->latest())) {
            pixel_color = relay2->pixel(pixel_color);
            DRAWPIX(pixel_color);
            prefs.putUChar("relay2_status", relay2->state());
            post_note(relay2->name(), relay2->state() ? "on" : "off");
          }
        }
      }
    }
 
    if (timer_count % (10) == 2) {
      getCO2();
    }

    if (timer_count % (10) == 4) {
      getTemp();
    }

    if (timer_count % (10) == 6) {
      getTVOC();
    }

    if (timer_count % (10) == 8) {
      getPressure();
    }

    if (timer_count % 2 == 0) {  // 2秒おき
      // getRelayStatus(&heater_status, url_relay);
      // getRelayStatus(&heater2_status, url_relay2);
      DRAWSCREEN(true);
    }

    if (timer_count % (600) == 10) {  // 10分
      if (use_co2) {
        if (co2log->latest() > 0) {
          post_value(co2_id, co2log->average());
          DPRINTF("co2 avg: %.1f\n",co2log->average());
#ifdef USE_SCD4X
          if (!use_humidity) {
            post_value(humid_id, humidlog->average());
            DPRINTF("humid avg: %.1f\n",humidlog->average());
          }
          if (!use_thermo && !use_humidity) {
            post_value(temp_id, templog->average());
            DPRINTF("temp avg: %.1f\n",templog->average());
          }
#endif
        }
      }
      if (use_thermo) {
        post_value(temp_id, templog->average());
      }
      if (use_humidity) {
        post_value(humid_id, humidlog->average());
        if (!use_thermo) {
          post_value(temp_id, templog->average());
        }
      }
      if (use_pressure) {
        post_value(press_id, pressurelog->average());
      }
      if (use_tvoc) {
        post_value(tvoc_id, tvoclog->average());
      }
    }

    if (timer_count % (60) == 0) {  // 1分
      post_alive();
    }

    timer_count++;
    timer_count %= (86400UL);
  }
  delay(5);
#endif
}
