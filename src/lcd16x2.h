#ifndef LCD16X2_H
#define LCD16X2_H

#include "Arduino.h"
#include <Wire.h>

class Lcd16x2 {
  public:
    Lcd16x2();
    void begin();
    void printStr(const char *s);
    void printFloat(float f, int degits, int decimal);
    void printInt(int d, int degits);
    void setCursor(unsigned char x, unsigned char y) ;

  protected:
    uint8_t i2caddr;
    uint8_t vlcd;
    void cmd(unsigned char x);
    void data(unsigned char x);
};

#endif
