#include "lcd16x2.h"

Lcd16x2::Lcd16x2() {
  i2caddr = (0x3a);
  vlcd = (27);
}

void Lcd16x2::begin() {
  delay(100);
  cmd(0x34);
  delay(5);
  cmd(0x34);
  delay(5);
  cmd(0x34);
  delay(40);

  Wire.beginTransmission(i2caddr);
  Wire.write(0x00); // CO = 0,RS = 0
  Wire.write(0x35);
  Wire.write(0x41);
  Wire.write(0x80 | vlcd);
  Wire.write(0xC0 | vlcd);
  Wire.write(0x34);
  Wire.endTransmission();

  cmd(0x01);
  delay(400);

  cmd(0x0C);
  cmd(0x06);

  delay(500);
}

void Lcd16x2::cmd(unsigned char x) {
  Wire.beginTransmission(i2caddr);
  Wire.write(0b00000000); // CO = 0,RS = 0
  Wire.write(x);
  Wire.endTransmission();
}

void Lcd16x2::data(unsigned char x){
  Wire.beginTransmission(i2caddr);
  Wire.write(0b01000000); // CO = 0, RS = 1
  Wire.write(x ^ 0x80);
  Wire.endTransmission();
}

// 文字の表示
void Lcd16x2::printStr(const char *s) {
  Wire.beginTransmission(i2caddr);
  while (*s) {
    if (*(s + 1)) {
      Wire.write(0b11000000); // CO = 1, RS = 1
      Wire.write(*s ^ 0x80);
    } else {
      Wire.write(0b01000000); // CO = 0, RS = 1
      Wire.write(*s ^ 0x80);
    }
    s++;
  }
  Wire.endTransmission();
}

// Int
void Lcd16x2::printInt(int d, int digits) {
  char formatstr[10], intstr[10];
  char *s;
  sprintf(formatstr,"%%%dd", digits);
  sprintf(intstr,formatstr,d);
  s = intstr;
  Wire.beginTransmission(i2caddr);
  while (*s) {
    if (*(s + 1)) {
      Wire.write(0b11000000); // CO = 1, RS = 1
      Wire.write(*s ^ 0x80);
    } else {
      Wire.write(0b01000000); // CO = 0, RS = 1
      Wire.write(*s ^ 0x80);
    }
    s++;
  }
  Wire.endTransmission();
}

// float
void Lcd16x2::printFloat(float f, int digits, int decimal) {
  char formatstr[10], floatstr[10];
  char *s;
  sprintf(formatstr,"%%%d.%df", digits+decimal, decimal);
  sprintf(floatstr,formatstr,f);
  s = floatstr;
  Wire.beginTransmission(i2caddr);
  while (*s) {
    if (*(s + 1)) {
      Wire.write(0b11000000); // CO = 1, RS = 1
      Wire.write(*s ^ 0x80);
    } else {
      Wire.write(0b01000000); // CO = 0, RS = 1
      Wire.write(*s ^ 0x80);
    }
    s++;
  }
  Wire.endTransmission();
}
 
// 表示位置の指定
void Lcd16x2::setCursor(unsigned char x, unsigned char y) {
  cmd(0x80 | (y * 0x40 + x));
}