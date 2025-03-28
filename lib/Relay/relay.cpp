/*
  This is a class library for M5Switch D (M5 ATOM Lite base)
*/
#include "relay.h"

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

String Relay::name() { 
  return _name + (_api != "" ? " (external)" : (_tplug != NULL ? " (tplug)" : "")); 
}

int Relay::_request(bool state) {
  HTTPClient http;
  String body;
  JsonDocument json;

  String apiurl = _api + (state ? "on" : "off");
  http.begin(apiurl);
  int httpCode = http.GET();
  body = http.getString();
  http.end();
  deserializeJson(json, body);
  if (!json["relay1"].isNull()) {
    _state = json["relay1"]["state"].as<String>() == "on" ? true : false;
    _name = json["relay1"]["name"].as<String>();
    if (json["relay1"]["changed"].as<bool>()) 
      return 1;
    return 0;
  } else if (!json["relay2"].isNull()) {
    _state = json["relay2"]["state"].as<String>() == "on" ? true : false;
    _name = json["relay2"]["name"].as<String>();
    if (json["relay2"]["changed"].as<bool>()) 
      return 1;
    return 0;
  } else {
    return 0;
  }
}

// リモートのリレーの状態(state)を取得
bool Relay::_request_state() {
  HTTPClient http;
  String body;
  JsonDocument json;

  String apiurl = _api ;
  http.begin(apiurl);
  int httpCode = http.GET();
  body = http.getString();
  http.end();
  deserializeJson(json, body);
  if (!json["relay1"].isNull()) {
    _name = json["relay1"]["name"].as<String>();
    return json["relay1"]["state"].as<String>() == "on" ? true : false; 
  } else if (!json["relay2"].isNull()) {
    _name = json["relay2"]["name"].as<String>();
    return json["relay2"]["state"].as<String>() == "on" ? true : false; 
  }
  return false;
}


// 返値は，変更があった場合にtrue
// TPLinkSmartPlugを使っている場合は，TPLinkSmartPlugにリクエストを送信し，
// 成功したかを返す．
bool Relay::relay(bool state) {
  bool ret = false;
  if (_api != "") {
    ret = _request(state);
  } else if (_tplug != NULL) {
    ret = _tplug->setRelayState(state);
  } else {
    if (_state != state) {
      digitalWrite(_pin, state ? HIGH : LOW);
      ret = true;
    }
  }
  _state = state;
  return ret;
}

bool Relay::on() { return relay(true); }
bool Relay::off() { return relay(false); }

bool Relay::state() { 
  JsonDocument json;
  if (_api != "") {
    _state = _request_state();
  } else if (_tplug != NULL) {
    deserializeJson(json, _tplug->getSysInfo());
    _state = json["relay_state"].as<bool>();
  }  
  return _state;   
}

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
  bool ret = false;
  Serial.printf("relay state: %d temp: %.1f on_temp: %.1f off_temp: %.1f\n",
                _state, t, _on_temp, _off_temp);
  if (t <= _on_temp) {
      ret = on();
  } else if (t >= _off_temp) {
      ret = off();
  }
  return ret;
}

void Relay::onTime(String t) { _on_time = t; }

void Relay::offTime(String t) { _off_time = t; }

int Relay::manageByTime(DateTime now) {
  int idx, prev_idx = 0;
  bool ret = false;
  if (_on_time != "") {
    do {
      idx = _on_time.indexOf(",", prev_idx);
      if (idx < 0) {
        idx = _on_time.length();
      }
      String tstring = _on_time.substring(prev_idx, idx);
      if (tstring != "") {
        int tidx = tstring.indexOf(":");
        int hourmin = tstring.substring(0, tidx).toInt() * 60 +
                      tstring.substring(tidx + 1).toInt();
        //        Serial.print("on_relay1:");
        //        Serial.println(hourmin);
        if (now.hour() * 60 + now.minute() == hourmin &&
            _last_on != hourmin) {
          _last_on = hourmin;
          if (!state()) {
            ret = on();
            return ret;
          }
        }
      }
      prev_idx = idx + 1;
    } while (idx < _on_time.length());
  }
  prev_idx = 0;
  if (_off_time != "") {
    do {
      idx = _off_time.indexOf(",", prev_idx);
      if (idx < 0) {
        idx = _off_time.length();
      }
      String tstring = _off_time.substring(prev_idx, idx);
      if (tstring != "") {
        int tidx = tstring.indexOf(":");
        int hourmin = tstring.substring(0, tidx).toInt() * 60 +
                      tstring.substring(tidx + 1).toInt();
        //        Serial.print("on_relay1:");
        //        Serial.println(hourmin);
        if (now.hour() * 60 + now.minute() == hourmin &&
            _last_off != hourmin) {
          _last_off = hourmin;
          if (state()) {
            ret = off();
            return ret;
          }
        }
      }
      prev_idx = idx + 1;
    } while (idx < _off_time.length());
  }
  return ret;
}