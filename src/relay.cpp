/*
  This is a class library for M5Switch D (M5 ATOM Lite base)
*/
#include "relay.h"
#include "debugmacros.h"

#include <HTTPClient.h>

Relay::Relay(uint8_t pin, uint32_t pixel_color) {
  _pin = pin;
  _tplug=NULL;
  _api = "";
  _state = false;
  _pixel_color = pixel_color;
}

Relay::Relay(String api) {
  _api = api;
  _tplug=NULL;
  _state = false;
}

Relay::Relay(TPLinkSmartPlug *tp) {
  _tplug = tp;
  _state = false;
}

void Relay::name(String n) { _name = n; }

String Relay::name() { return _name; }

int Relay::_request(bool state) {
  HTTPClient http;
  String body;
  String apiurl = _api + (state ? "on" : "off");
  http.begin(apiurl);
  int httpCode = http.GET();
  body = http.getString();
  http.end();

  if (httpCode != 200) {
    return 1;
  } else {
    return 0;
  }
}

bool Relay::relay(bool state) {
  if (_api != "") {
    _request(state);
  } else if (_tplug != NULL) {
    _tplug->setRelayState(state);
  } else {
    digitalWrite(_pin, state ? HIGH : LOW);
  }
  _state = state;
  return _state;
}

bool Relay::on() { return relay(true); }
bool Relay::off() { return relay(false); }

bool Relay::state() { return _state; }

uint32_t Relay::pixel(uint32_t p) {
  if (_state) {
    p |= _pixel_color;
  } else {
    p &= ~_pixel_color;
  }
  return p;
}

float Relay::onTemp() { return _on_temp; }

float Relay::offTemp() { return _off_temp; }

void Relay::onTemp(float on_t) { _on_temp = on_t; }
void Relay::offTemp(float off_t) { _off_temp = off_t; }

int Relay::manageByTemperature(float t) {
  Serial.printf("relay state: %d temp: %.1f on_temp: %.1f off_temp: %.1f\n",
                _state, t, _on_temp, _off_temp);
  if (t <= _on_temp) {
    if (!_state) {
      on();
      Serial.println("relay on");
      return 1;
    }
  } else if (t >= _off_temp) {
    if (_state) {
      off();
      Serial.println("relay off");
      return 1;
    }
  }
  return 0;
}