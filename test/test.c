#include <stdio.h>

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    const long run = in_max - in_min;
    if(run == 0){
        //log_e("map(): Invalid input range, min == max");
        return -1; // AVR returns -1, SAM returns 0
    }
    const long rise = out_max - out_min;
    const long delta = x - in_min;
    return (delta * rise) / run + out_min;
}

float manageFan(float humid, float target) {
  const float delta = 1;
  const float kp = 2, ki = 2, kd = 1;
  static float diff_p = 0, diff_c = 0, integral = 0;
  float p, i, d;
  diff_p = diff_c;
  diff_c = humid - target;
  integral += (diff_c + diff_p) / 2.0 * delta;

  p = kp * diff_c;
  i = ki * integral;
  d = kd * (diff_c - diff_p) / delta;
  float power = p + i+ d;
  int ipower;
  ipower = map((int)power,-110,110,0,255);
  printf("humid: %f, target: %f, power: %f %d\n", humid, target, power, ipower > 255 ? 255 : (ipower < 0 ? 0 : ipower));
  return power;
}

int main() {
  float h = 100.0, d;
  //while (1) {
  for (int i = 0; i < 100; i++) {
    d = manageFan(h, 85);
    h = h - (d / 50);
    h = h > 100 ? 100 : h;
  }  
  return 0;
}