#ifndef RELAY_h 
#define RELAY_h 
#include "Arduino.h"

#include "TPLinkSmartPlug.h"

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
        float onTemp();
        float offTemp();
        void onTemp(float);
        void offTemp(float);
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
        int _request(bool state);
        TPLinkSmartPlug *_tplug = NULL;
};

#endif
