# Smart Sensor Series

水槽・レオパケージにおける温湿度等計測を簡単にするセンサーシステムです．
M5ATOM シリーズを使った単機能センサーを志向して作っています．

ATOM HUB Switch Dを利用すると，リレーを制御もできます．
また，外部にあるATOM HUB Switch DをWiFi経由で制御する機能も有しています．
これにより，パネルヒーターや照明などのAC機器を温度や時間に基づいて制御することができます．

## Supported Sensors
* SHT3X (Humidity, Temperature)
* DS18B20 (Temperature)
* SCD4X (CO2)
* MHZ-19 (CO2)
* QMP6988 (Air pressure)
* Magnetic switch (Door sensor)
* SGP30 (TVOC, AirQuality)

## Supported Devices
* TPlink HS105 (AC relay)
* Fan
* ATOM HUB Switch D

## How to use

`smart_env_esp32.h`にて利用するデバイスを設定します．


## GitHub

https://github.com/omzn/
