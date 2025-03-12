#ifndef FAN_h 
#define FAN_h 
#include "Arduino.h"

class Fan
{
    public:
        Fan(uint8_t pin=25, uint8_t min_power = 65);
        void fan(uint8_t power);
        uint8_t fan();
        void on();
        void off();
        int manageByHumid(float h);
        void targetHumid(float h);
        float targetHumid();
        int manageByTemp(float h);
        void targetTemp(float t);
        String name();
        void name(String n);
        bool record() {
            return _record;
        };
    private:
        bool _record = 0;
        uint8_t _pin;
        uint8_t _power;
        uint8_t _min_power;
        float _target_temp;
        float _target_humid;
        String _name;
};

#endif
