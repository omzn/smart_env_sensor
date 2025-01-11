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
#include <RTClib.h>
// #include <WiFiManager.h>
#include <Wire.h>
#include <pgmspace.h>

#include "lcd16x2.h"
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

#define DEFAULT_CONFIG F("{\"hostname\":\"mysensor\",\"enable_push\":false,\"url_endpoint\":\"http://myserver:3000\",\"fan\":{\"id\":\"fan\",\"manage_by_humid\":false,\"target_humid\":50},\"relay1\":{\"id\":\"relay1\",\"manage_by_temp\":false,\"on_temp\":25,\"off_temp\":30,\"manage_by_time\":false,\"on_time\":\"10:00,12:00\",\"off_time\":\"11:00,18:00\"},\"relay2\":{\"id\":\"relay2\",\"manage_by_temp\":false,\"on_temp\":25,\"off_temp\":30,\"manage_by_time\":false,\"on_time\":\"10:00,11:00\",\"off_time\":\"10:10,11:10\"},\"url_extrelay\":\"http://myextrelay:3000\",\"host_tplug\":\"\",\"temp\":{\"id\":\"mytemp1\"},\"humidity\":{\"id\":\"myhumid1\"},\"pressure\":{\"id\":\"mypressure\"},\"door\":{\"id\":\"mydoor\"},\"co2\":{\"id\":\"myco2\"},\"tvoc\":{\"id\":\"mytvoc\"}}")
#define DEFAULT_FUNCTION F("{\"use_thermo\":false,\"use_humidity\":false,\"use_pressure\":false,\"use_doorsensor\":false,\"use_co2\":false,\"use_tvoc\":false,\"use_fan\":false,\"use_relay\":false,\"use_extrelay\":false,\"use_tplug\":false,\"use_lcd\":false}")

#include "debugmacros.h"
#include "DataLog.h"

#include "fan.h"
#include "ntp.h"
#include "relay.h"

enum { FONT_SMALL = 0, FONT_MEDIUM, FONT_BIG, FONT_LARGE };
enum { DOOR_OPEN = 0, DOOR_CLOSE };

JsonDocument prefs_json;
JsonDocument func_json;

const String boolstr[2] = {"false", "true"};

const char *apSSID = "WIFI_SENSOR";
DNSServer dnsServer;

const IPAddress apIP(192, 168, 1, 1);
boolean wifi_ap_mode = false;
String ssid = "hoge";
String wifipass = "piyo";
uint32_t wifi_connect_timer;

uint32_t timer_count = 0;
uint32_t p_millis;
uint8_t rtcint, btnint;

bool use_thermo = false;
bool use_humidity = false; 
bool use_pressure = false;
bool use_co2 = false;
bool use_doorsensor = false;
bool use_fan = false;
bool use_relay = false;
bool use_extrelay = false;
bool use_tplug = false;
bool use_tvoc = false;
bool use_lcd = false;

bool manage_fan = false;
bool manage_temp_relay1 = false;
bool manage_temp_relay2 = false;
bool manage_time_relay1 = false;
bool manage_time_relay2 = false;

int lcd_rotation = 1;

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

Lcd16x2 lcd;

String error_message;

DataLog *templog, *humidlog, *co2log, *pressurelog, *tvoclog;

RTC_Millis rtc;

void mergeJson(JsonVariant dst, JsonVariantConst src) {
  if (src.is<JsonObjectConst>()) {
    for (JsonPairConst kvp : src.as<JsonObjectConst>()) {
      if (dst[kvp.key()]) 
        mergeJson(dst[kvp.key()], kvp.value());
      else
        dst[kvp.key()] = kvp.value();
    }
  }
  else {
    dst.set(src);
  }
}

String timestamp() {
  String ts;
  DateTime now = rtc.now();
  char str[20];
  sprintf(str, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(),
          now.day(), now.hour(), now.minute(), now.second());
  ts = str;
  return ts;
}

uint32_t timestamp_epoch() {
  String ts;
  DateTime now = rtc.now();
  return now.unixtime();
}

void config() {
  if (use_fan) {
    manage_fan = prefs_json["fan"]["manage_by_humid"];
    fan.targetHumid(prefs_json["fan"]["target_humid"].as<float>());
    fan.name(prefs_json["fan"]["id"].as<String>());
  }

  if (use_relay) {
    relay1 = new Relay(RELAY1_PIN, 0xff0000);
    relay2 = new Relay(RELAY2_PIN, 0x00ff00);
    pinMode(RELAY1_PIN, OUTPUT);
    pinMode(RELAY2_PIN, OUTPUT);
    relay1->name(prefs_json["relay1"]["id"].as<String>());
    relay2->name(prefs_json["relay2"]["id"].as<String>());
    relay1->relay((bool)prefs.getUChar("relay1_state", 0));
    pixel_color = relay1->pixel(pixel_color);
    DRAWPIX(pixel_color);
    relay2->relay((bool)prefs.getUChar("relay2_state", 0));
    pixel_color = relay2->pixel(pixel_color);
    DRAWPIX(pixel_color);
    relay1->onTemp(prefs_json["relay1"]["on_temp"].as<float>());
    relay1->offTemp(prefs_json["relay1"]["off_temp"].as<float>());
    relay2->onTemp(prefs_json["relay2"]["on_temp"].as<float>());
    relay2->offTemp(prefs_json["relay2"]["off_temp"].as<float>());
    manage_temp_relay1 = prefs_json["relay1"]["manage_by_temp"].as<bool>();
    manage_temp_relay2 = prefs_json["relay2"]["manage_by_temp"].as<bool>();
    relay1->onTime(prefs_json["relay1"]["on_time"].as<String>());
    relay1->offTime(prefs_json["relay1"]["off_time"].as<String>());
    manage_time_relay1 = prefs_json["relay1"]["manage_by_time"].as<bool>();
    relay2->onTime(prefs_json["relay2"]["on_time"].as<String>());
    relay2->offTime(prefs_json["relay2"]["off_time"].as<String>());
    manage_time_relay2 = prefs_json["relay2"]["manage_by_time"].as<bool>();
  }

  if (use_extrelay) {
    relay1 = new Relay(prefs_json["url_extrelay"].as<String>());
    relay1->name(prefs_json["relay1"]["id"].as<String>());
    relay1->relay(prefs.getUChar("relay1_state", 0));
    relay1->onTemp(prefs_json["relay1"]["on_temp"].as<float>());
    relay1->offTemp(prefs_json["relay1"]["off_temp"].as<float>());
    manage_temp_relay1 = prefs_json["relay1"]["manage_by_temp"].as<bool>();
    relay1->onTime(prefs_json["relay1"]["on_time"].as<String>());
    relay1->offTime(prefs_json["relay1"]["off_time"].as<String>());
    manage_time_relay1 = prefs_json["relay1"]["manage_by_time"].as<bool>();
  }

  if (use_tplug) {
    tplug = new TPLinkSmartPlug();
    tplug->begin(client,udp);
    tplug->setTarget(prefs_json["host_tplug"].as<String>());
    relay1 = new Relay(tplug);
    relay1->name(prefs_json["relay1"]["id"].as<String>());
    relay1->relay(prefs.getUChar("relay1_state", 0));
    relay1->onTemp(prefs_json["relay1"]["on_temp"].as<float>());
    relay1->offTemp(prefs_json["relay1"]["off_temp"].as<float>());
    manage_temp_relay1 = prefs_json["relay1"]["manage_by_temp"].as<bool>();
    relay1->onTime(prefs_json["relay1"]["on_time"].as<String>());
    relay1->offTime(prefs_json["relay1"]["off_time"].as<String>());
    manage_time_relay1 = prefs_json["relay1"]["manage_by_time"].as<bool>();
  }

}

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
        post_note(prefs_json["door"]["id"], "open");
      }
      door_count++;
    } else {
      if (door_count > 0) {
        post_note(prefs_json["door"]["id"], "close");
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

  JsonDocument root;
  DeserializationError error = deserializeJson(root, body);
  if (error) {
    DPRINTLN("deserializeJson() failed.");
    return -1;
  }
  *t = root["air_temp"];
  *h = root["air_humid"];
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
    String hostname = prefs_json["hostname"].as<String>();  
    M5.Lcd.print(hostname);
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
    String hostname = webServer.arg("site");
    prefs.putString("hostname", hostname);
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
  String message = "", argname, argv, body;
  JsonDocument json;

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
    json["relay1"]["name"] = relay1->name();
    json["relay1"]["state"] = relay1->state() ? "on" : "off";
    json["relay2"]["name"] = relay2->name();
    json["relay2"]["state"] = relay2->state() ? "on" : "off";
  }
  if (use_tplug || use_extrelay) {
    json["relay1"]["name"] = relay1->name();
    json["relay1"]["state"] = relay1->state() ? "on" : "off";
  }
  json["timestamp"] = timestamp();
  serializeJsonPretty(json,message);
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
  String message, argname, argv, body;
  JsonDocument json;
  int changed = 0;

  if (webServer.method() == HTTP_POST) {
    body = webServer.arg("plain");
    deserializeJson(json, body);
    DeserializationError error = deserializeJson(json, body);
    if (error) {
      String estr = error.f_str();
      if (estr != "EmptyInput") {
        message = F("deserializeJson() failed: ");
        message += error.f_str();
        message += "\n";
        webServer.send(200, "text/plain", message);
        return;
      }
    }
  } else {
    for (int i = 0; i < webServer.args(); i++) {
      argname = webServer.argName(i);
      argv = webServer.arg(i);    
      json[argname] = argv == "true" ? true : false;
    }
  }
  if (!json.isNull()) {
    if (json["overwrite"].as<bool>()) {
      json.remove("overwrite");
      func_json = json;
    } else {
      DPRINTLN("Merging json");
      mergeJson(func_json, json);
    }
    prefs.putString("function", func_json.as<String>());
    changed++;
  }
  serializeJson(func_json,message);
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
  String message, argname, argv, body;
  JsonDocument json, filter;

  if (webServer.method() == HTTP_POST) {
    body = webServer.arg("plain");
    DeserializationError error = deserializeJson(json, body);
    if (error) {
      String estr = error.f_str();
      if (estr != "EmptyInput") {
        message = F("deserializeJson() failed: ");
        message += error.f_str();
        message += "\n";
        webServer.send(200, "text/plain", message);
        return;
      }
    }
  }
  if (!json.isNull()) {
    if (json["overwrite"].as<bool>()) {
      json.remove("overwrite");
      prefs_json = json;
    } else {
      DPRINTLN("Merging json");
      mergeJson(prefs_json, json);
    }
    prefs.putString("prefs", prefs_json.as<String>());
    config();
  }

  serializeJson(prefs_json,message);
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

/*
config
{
  "relay1":{
    "id":"relay1",
    "manage_by_temp":true,
    "on_temp":30.0,
    "off_temp":25.0
  },
  "relay2":{
    "id":"relay2",
    "manage_by_temp":true,
    "on_temp":30.0,
    "off_temp":25.0
  },
}

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
  int enable_push = prefs_json["enable_push"].as<bool>();
  if (enable_push) {
    String apiurl = prefs_json["url_endpoint"].as<String>() + "/aqua/add";
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
  int enable_push = prefs_json["enable_push"].as<bool>();
  if (enable_push) {
    String apiurl = prefs_json["url_endpoint"].as<String>() + "/aqua/add";
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
  int enable_push = prefs_json["enable_push"].as<bool>();
  if (enable_push) {
    String apiurl = prefs_json["url_endpoint"].as<String>() + "/aqua/alive";
    String ipaddress = WiFi.localIP().toString();

    DPRINTLN(apiurl);

    http.begin(apiurl.c_str());
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    String hostname = prefs_json["hostname"].as<String>();  
    sprintf(params, "ipaddress=%s&label=%s", ipaddress.c_str(),
            hostname.c_str());
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

/*
                                                                                                                                
88                                          88 88            88888888ba            88                                          
88                                          88 88            88      "8b           88                                   ,d     
88                                          88 88            88      ,8P           88                                   88     
88,dPPYba,  ,adPPYYba, 8b,dPPYba,   ,adPPYb,88 88  ,adPPYba, 88aaaaaa8P' ,adPPYba, 88,dPPYba,   ,adPPYba,   ,adPPYba, MM88MMM  
88P'    "8a ""     `Y8 88P'   `"8a a8"    `Y88 88 a8P_____88 88""""88'  a8P_____88 88P'    "8a a8"     "8a a8"     "8a  88     
88       88 ,adPPPPP88 88       88 8b       88 88 8PP""""""" 88    `8b  8PP""""""" 88       d8 8b       d8 8b       d8  88     
88       88 88,    ,88 88       88 "8a,   ,d88 88 "8b,   ,aa 88     `8b "8b,   ,aa 88b,   ,a8" "8a,   ,a8" "8a,   ,a8"  88,    
88       88 `"8bbdP"Y8 88       88  `"8bbdP"Y8 88  `"Ybbd8"' 88      `8b `"Ybbd8"' 8Y"Ybbd8"'   `"YbbdP"'   `"YbbdP"'   "Y888  
                                                                                                                               
                                                                                                                               

 */
void handleReboot() {
  String message, argname, argv;
  JsonDocument json;
  json["reboot"] = "done";

  for (int i = 0; i < webServer.args(); i++) {
    argname = webServer.argName(i);
    argv = webServer.arg(i);
    DPRINT("argname:");
    DPRINT(argname);
    DPRINT(" = ");
    DPRINTLN(argv);

    if (argname == "init") {
      prefs.putString("prefs", DEFAULT_CONFIG);
      prefs.putString("function", DEFAULT_FUNCTION);
      json["reboot"] = "init";
    }
  }

  serializeJson(json,message);
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
  deserializeJson(func_json, prefs.getString("function",DEFAULT_FUNCTION));
  deserializeJson(prefs_json, prefs.getString("prefs", DEFAULT_CONFIG));

  ssid = prefs.getString("ssid", ssid);
  wifipass = prefs.getString("wifipass", wifipass);

#if defined(ARDUINO_M5Stack_ATOMS3)
#if HAS_DISPLAY
  lcd_rotation = prefs.getUChar("lcd_rotate",0);
  M5.Lcd.setRotation(lcd_rotation);
#endif
#endif

  use_thermo = func_json["use_thermo"].as<bool>();
  use_humidity = func_json["use_humidity"].as<bool>();
  use_pressure = func_json["use_pressure"].as<bool>();
  use_doorsensor = func_json["use_doorsensor"].as<bool>();
  use_co2 = func_json["use_co2"].as<bool>();
  use_tvoc = func_json["use_tvoc"].as<bool>();
  use_fan = func_json["use_fan"].as<bool>();
  use_relay = func_json["use_relay"].as<bool>();
  use_extrelay = func_json["use_extrelay"].as<bool>();
  use_tplug = func_json["use_tplug"].as<bool>();
  use_lcd = func_json["use_lcd"].as<bool>();

  if (use_lcd) {
    lcd.begin();
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
    String hostname = prefs_json["hostname"].as<String>();
    ArduinoOTA.setHostname(hostname.c_str());
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

  config();

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
    ntp.begin();
    //  webServer.begin();
    uint32_t epoch = ntp.getTime();
    if (epoch > 0) {
      rtc.adjust(DateTime(epoch + SECONDS_UTC_TO_JST));
    }
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
        if (manage_temp_relay1 && (use_thermo || use_humidity || use_co2)) {
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
        if (manage_time_relay1) {
          if (relay1->manageByTime(rtc.now())) {
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
        if (manage_temp_relay2 && (use_thermo || use_humidity || use_co2)) {
          if (relay2->manageByTemperature(templog->latest())) {
            pixel_color = relay2->pixel(pixel_color);
            DRAWPIX(pixel_color);
            prefs.putUChar("relay2_status", relay2->state());
            post_note(relay2->name(), relay2->state() ? "on" : "off");
          }
          if (manage_time_relay2) {
            if (relay2->manageByTime(rtc.now())) {
              pixel_color = relay2->pixel(pixel_color);
              DRAWPIX(pixel_color);
              prefs.putUChar("relay2_status", relay2->state());
              post_note(relay2->name(), relay2->state() ? "on" : "off");
            }
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
      if (use_lcd) {
        lcd.setCursor(0, 0);
        lcd.printStr("T:");
        lcd.setCursor(2, 0);
        lcd.printFloat(templog->latest(), 4, 1);
        lcd.setCursor(0, 1);
        lcd.printStr("H:");  
        lcd.setCursor(2, 1);
        lcd.printFloat(humidlog->latest(), 4, 1);
      }
    }

    if (timer_count % (600) == 10) {  // 10分
      if (use_co2) {
        if (co2log->latest() > 0) {
          post_value(prefs_json["co2"]["id"].as<String>(), co2log->average());
          DPRINTF("co2 avg: %.1f\n",co2log->average());
#ifdef USE_SCD4X
          if (!use_humidity) {
            post_value(prefs_json["humidity"]["id"].as<String>(), humidlog->average());
            DPRINTF("humid avg: %.1f\n",humidlog->average());
          }
          if (!use_thermo && !use_humidity) {
            post_value(prefs_json["temp"]["id"].as<String>(), templog->average());
            DPRINTF("temp avg: %.1f\n",templog->average());
          }
#endif
        }
      }
      if (use_thermo) {
        post_value(prefs_json["temp"]["id"].as<String>(), templog->average());
      }
      if (use_humidity) {
        post_value(prefs_json["humidity"]["id"].as<String>(), humidlog->average());
        if (!use_thermo) {
          post_value(prefs_json["temp"]["id"].as<String>(), templog->average());
        }
      }
      if (use_pressure) {
        post_value(prefs_json["pressure"]["id"].as<String>(), pressurelog->average());
      }
      if (use_tvoc) {
        post_value(prefs_json["tvoc"]["id"].as<String>(), tvoclog->average());
      }
      if (use_lcd) {  
        uint32_t fs = prefs_json["fetch_start"][0];
        lcd.setCursor(8,0);
        lcd.printStr("Day:");
        if(fs > 0) {
          float elapsed_days1 =
              fs > 0 ? float(timestamp_epoch() - fs) / (float)SECONDS_PER_DAY : 0;
          lcd.setCursor(12,0);
          lcd.printInt((int)elapsed_days1,3);
        }
        fs = prefs_json["fetch_start"][1].as<int>();
        if(fs > 0) {
          float elapsed_days2 =
              fs > 0 ? float(timestamp_epoch() - fs) / (float)SECONDS_PER_DAY : 0;
          lcd.setCursor(12,1);
          lcd.printInt((int)elapsed_days2,3);
        }
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
