#include "DataLog.h"

DataLog::DataLog(int numData) {
  logsize = numData;
  log = new float[logsize];
  index = 0;
  current = -1;
  first_cycle = true;
}

DataLog::~DataLog() {
  delete[] log;
}

int DataLog::add(float v) {
  log[index] = v;
  current = index;
  index++;
  if (index > logsize - 1) {
    first_cycle = false;
  }
  index %= logsize;
  return index;
}

float DataLog::average() {
  float sum = 0.0;
  for (int i = 0; i < (first_cycle && index ? index : logsize); i++) {
    sum += log[i];
  }
  return (sum / (first_cycle && index ? index : logsize));
}

float DataLog::latest() {
  if (current == -1) return 0;
  return log[current];  
}
