/* 
  This is a class library for Fan
*/
#include "fan.h"

// construct a Fan.
//  pin : GPIO pin
//  min_power: ファンの起電力 (pwm 0-255)
Fan::Fan(uint8_t pin, uint8_t min_power) {
  _pin = pin;
  _min_power = min_power;
}

void Fan::fan(uint8_t power) {
  int actual_power = power > 0 ? constrain(map(power,0,255,_min_power,255),0,255) : 0;   
  _power = power;
  analogWrite(_pin, actual_power);
  //digitalWrite(_pin, actual_power ? HIGH : LOW);
}

uint8_t Fan::fan() {
    return _power;
}

void Fan::on() {
  fan(255);
}

void Fan::off() {
  fan(0);
}

void Fan::targetTemp(float t) {
  _target_temp = t;
}

void Fan::targetHumid(float h) {
  _target_humid = h;
}

float Fan::targetHumid() {
  return _target_humid;
}

int Fan::manageByTemp(float t) {
  return 0;
}

int Fan::manageByHumid(float h) {
  const float dt = 2;
  const float kp = 10, ki = 1, kd = 10;

  static float diff_p = 0, diff_c = 0, integral = 0;
  float p, i, d;
  int prev_power = fan();

  diff_p = diff_c;          // 湿度の差 (%)
  diff_c = h - _target_humid;  // 湿度の差 (%)
  integral += (diff_c + diff_p) / 2.0 * dt;
  integral = integral > 50 ? 50 : (integral < -50 ? -50 : integral);

  p = kp * diff_c * (diff_c < 0 ? 5 : 1); // 1 % で pwm 10 :負の時は x 5
  i = ki * integral;                //
  d = kd * (diff_c - diff_p) / dt;  // 1% で pwm 10
  float power = p + i + d;
  int ipower = constrain((int)power, 0, 255);
  fan(ipower);
  return (ipower && !prev_power || !ipower && prev_power);
}

void Fan::name(String n) {
  _name = n;
}

String Fan::name() {
  return _name;
}