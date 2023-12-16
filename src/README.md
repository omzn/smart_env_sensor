# Year 2038 problem countdown clock

https://youtu.be/E-os8XCysBw

The Year 2038 problem is a problem in which time values are stored as
a signed 32-bit integer, and this number is interpreted as the number
of seconds since 00:00:00 UTC on 1 January 1970.  In such
implementation, after 2038-01-19 03:14:07(UTC), the variable will
overflow and may cause system failure.
For more, see [wikipedia](https://en.wikipedia.org/wiki/Year_2038_problem).

This device counts down the time to the moment that the year 2038
occurs. It continuously calibrates the internal RTC by accessing an
ntp server.

In this device, the year 2038 problem *do* take place.

```cpp
      if (dispmode == MODE_UNIXTIME) {
        dispDigits(now.unixtime()-SECONDS_UTC_TO_JST);
      } else if (dispmode == MODE_COUNTDOWN) {
        dispDigits((0x7FFFFFFF - (now.unixtime()-SECONDS_UTC_TO_JST))); // y 2038!
      } else if (dispmode == MODE_CLOCK) {
        dispClock(now);
      } else if (dispmode == MODE_DATE) {
        dispDate(now);
      }
```

https://youtu.be/hJBHb5ON73I

Just casting values in `dispDigits()` by `(int32_t)` makes things fine.

https://youtu.be/Mgxpzdlathw

## Materials

* Micro controller: ESP-WROOM-02
* LED driver IC: HT16K33
* 7 segment LED x 4: OSL40562-IB x 2 (anode common)
* 7 segment LED x 1: OSL10561-IB x 2 (anode common)
* RTC: DS1307
* 32kHz Crystal
* push switch (vertical)
* CR2032 battery holder
* PCA9306 I2C level converter
* Original PCB

## Hardware

To say simply, this is a 10-digits 7-segment LED display with WiFi
connection.

## Software

I used Adafruit HT16K33 Arduino library (`Adafruit_LEDBackpack`) for
driving 7 segment LEDs. However, this driver is written for
cathode-common LEDs. So I inherit the `Adafruit_LEDBackPack` class and
modified display method for anode common LEDs (`Anodecommon_7seg`).

The clock is always calibrated by ntp service and RTC with battery
stores time even if the power is not supplied.

This device provides HTTP service and REST API.

| API | parameter| value |   effect         |
|-----|----------|-------|----------------|
| mode| value    |  int   | change mode  |
| status| --     | --     | show status     |

```
$ curl http://esp_7seg.local/status
{"unixtime":1517957705,"countdown":629525942,"mode":0}

$ curl "http://esp_7seg.local/mode?value=2"
{"mode":2}
```

## GitHub

https://github.com/omzn/y2038_countdown
