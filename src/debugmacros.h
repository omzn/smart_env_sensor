// Variadic macros used to print information in de-bugging mode
// from LarryD, Arduino forum

#pragma once
// un-comment this line to print the debugging statements
#define DEBUG

#if defined(DEBUG) && defined(ARDUINO_M5Stack_ATOM)
  #define DSERIALBEGIN(...) Serial.begin(__VA_ARGS__)
  #define DPRINT(...)    Serial.print(__VA_ARGS__)
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DPRINTF(...)    Serial.printf(__VA_ARGS__)
#elif defined(DEBUG) && defined(ARDUINO_M5Stack_ATOMS3)
  #define DSERIALBEGIN(...) USBSerial.begin(__VA_ARGS__)
  #define DPRINT(...)    USBSerial.print(__VA_ARGS__)
  #define DPRINTLN(...)  USBSerial.println(__VA_ARGS__)
  #define DPRINTF(...)    USBSerial.printf(__VA_ARGS__)
#elif defined(DEBUG) && defined(ARDUINO_M5STACK_FIRE)
  #define DSERIALBEGIN(...) Serial.begin(__VA_ARGS__)
  #define DPRINT(...)    Serial.print(__VA_ARGS__)
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DPRINTF(...)    Serial.printf(__VA_ARGS__)
  //#define DPRINT(...)    M5.Lcd.print(__VA_ARGS__)
  //#define DPRINTLN(...)  M5.Lcd.println(__VA_ARGS__)
  //#define DPRINTF(...)   M5.Lcd.printf(__VA_ARGS__)
#else
  // define blank line
  #define DSERIALBEGIN(...)
  #define DPRINT(...)
  #define DPRINTLN(...)
  #define DPRINTF(...)
#endif
