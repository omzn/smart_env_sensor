#ifndef SMART_ENV_ESP32_H
#define SMART_ENV_ESP32_H
#include <Arduino.h>

#define USE_GROVE_ENV3
//#define USE_HAT_ENV3
//#define USE_SCD4X
#define USE_MHZ19
//#define USE_DS18B20
//#define USE_SGP30

#if defined(ARDUINO_M5Stack_ATOM)
  /*
     GROVE.A G, 5V, 26:SDA, 32:SCL
     GROVE.B G, 5V, 25:SDA, 21:SCL
  */
  #define PIN_ONEWIRE   (25)
  // 下はM5StickC用のENV III HATを無理矢理転用した場合．
  // Wire.begin()の前にWire.end()してから設定すると良いらしい(？)
  #if defined(USE_HAT_ENV3)
    #define PIN_SDA          (25) 
    #define PIN_SCL          (21) 
  #else
    #define PIN_SDA          (26)
    #define PIN_SCL          (32)
  #endif

  // M5SwitchD では，2つ目のGROVEポート(25)につなげる
  // M5ATOM本体のGROVEの場合は，26にする．
  #define FAN_PIN       (25)

  // M5SwitchD では，2つ目のGROVEポート(21)につなげる
  // M5ATOM本体のGROVEの場合は，32にする．
  #define DOORSENSOR_PIN     (21)

  #define RELAY1_PIN  (22)
  #define RELAY2_PIN  (19)

  #define MHZ19_PIN_RX   (21) // Rx pin which the MHZ19 Tx pin is attached to
  #define MHZ19_PIN_TX   (25)  // Tx pin which the MHZ19 Rx pin is attached to
  #define MHZ19_BAUDRATE 9600
#elif defined(ARDUINO_M5Stack_ATOMS3)
  /*
     GROVE.A G, 5V, 2:SDA, 1:SCL
     GROVE.B G, 5V, 38:SDA, 39:SCL
  */
  #define PIN_ONEWIRE   (38)
  // 下はM5StickC用のENV III HATを無理矢理転用した場合．
  // Wire.begin()の前にWire.end()してから設定すると良いらしい(？)
  #if defined(USE_HAT_ENV3)
    #define PIN_SDA          (38) 
    #define PIN_SCL          (39) 
  #else
    #define PIN_SDA          (2)
    #define PIN_SCL          (1)
  #endif

  // M5SwitchD では，2つ目のGROVEポート(38)につなげる
  // M5ATOM本体のGROVEの場合は，2にする．
  #define FAN_PIN       (38)

  // M5SwitchD では，2つ目のGROVEポート(39)につなげる
  // M5ATOM本体のGROVEの場合は，1にする．
  #define DOORSENSOR_PIN     (39)

  #define RELAY1_PIN  (5)
  #define RELAY2_PIN  (6)

  #define MHZ19_PIN_RX   (39) // Rx pin which the MHZ19 Tx pin is attached to
  #define MHZ19_PIN_TX   (38)  // Tx pin which the MHZ19 Rx pin is attached to
  #define MHZ19_BAUDRATE 9600
#elif defined(ARDUINO_M5STACK_FIRE)
  /*
     GROVE.A G, 5V, 25:SDA, 32:SCL
     GROVE.B G, 5V, 26:SDA, 33:SCL
     GROVE.C G, 5V, 18:SDA, 19:SCL
  */
  #define PIN_ONEWIRE   (26)  // PORT.B
  #define PIN_SDA          (25) // PORT.A
  #define PIN_SCL          (32) // PORT.A

  #define FAN_PIN       (26) //PORT.B

  #define DOORSENSOR_PIN     (33) // PORT.B

  #define RELAY1_PIN  (-1)
  #define RELAY2_PIN  (-1)

  #define MHZ19_PIN_RX   (-1) // Rx pin which the MHZ19 Tx pin is attached to
  #define MHZ19_PIN_TX   (-1)  // Tx pin which the MHZ19 Rx pin is attached to
  #define MHZ19_BAUDRATE 9600
#elif defined(ARDUINO_M5Stack_CoreInk)
  /*
     GROVE.A G, 5V, 32:SDA, 33:SCL
  */
  #define PIN_ONEWIRE   (38)
  // 下はM5StickC用のENV III HATを無理矢理転用した場合．
  // Wire.begin()の前にWire.end()してから設定すると良いらしい(？)
  #if defined(USE_HAT_ENV3)
    #define PIN_SDA          (25) 
    #define PIN_SCL          (26) 
  #else
    #define PIN_SDA          (32)
    #define PIN_SCL          (33)
  #endif

  // HATポート
  #define FAN_PIN       (36)

  // M5SwitchD では，2つ目のGROVEポート(39)につなげる
  // M5ATOM本体のGROVEの場合は，1にする．
  #define DOORSENSOR_PIN     (22)

  #define RELAY1_PIN  (-1)
  #define RELAY2_PIN  (-1)

  #define MHZ19_PIN_RX   (-1) // Rx pin which the MHZ19 Tx pin is attached to
  #define MHZ19_PIN_TX   (-1)  // Tx pin which the MHZ19 Rx pin is attached to
  #define MHZ19_BAUDRATE 9600
#endif

void post_value(String s, float f);
void post_note(String s, String t);
void post_alive();
bool connectWifi();
void startWebServer();
void startWebServer_ap();
void handleStatus();
void handleFunction();
void handleConfig();
void handleHelp();
void handleRoot();
void handleCss();
void handleReboot();
void handleFan();
void handleRelay();
void handleDoorStatus();
void sendFs(String path, String contentType);
String makePage(String title, String contents);
String urlDecode(String input);
void setupArduinoOTA();

#endif
