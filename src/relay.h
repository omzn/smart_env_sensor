#ifndef RELAY_h 
#define RELAY_h 
#include "Arduino.h"

#include "TPLinkSmartPlug.h"
#include <RTClib.h>

class Relay
{
    public:
        Relay(uint8_t pin=22, uint32_t pixel_color = 0xff0000);
        Relay(String api);
        Relay(TPLinkSmartPlug *tp);
        bool relay(bool state);
        bool on();
        bool off();
        bool state();
        uint32_t pixel(uint32_t p);
        int manageByTemperature(float t);
        int manageByTime(DateTime now);
        float onTemp();
        float offTemp();
        void onTemp(float);
        void offTemp(float);
        void onTime(String);
        void offTime(String);
        void name(String);
        String name();
    private:
        String _name;
        bool _state;
        uint8_t _pin;
        uint32_t _pixel_color;
        float _on_temp;
        float _off_temp;
        String _api = "";
        String _on_time;
        String _off_time;
        int _last_on = 0;
        int _last_off = 0;
        int _request(bool state);
        TPLinkSmartPlug *_tplug = NULL;
};

#endif
