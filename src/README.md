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

## 機能の選択

`/function`にアクセスすると，機能一覧が取得できます．

```
{
  "use_thermo": false,
  "use_humidity": true,
  "use_pressure": false,
  "use_doorsensor": false,
  "use_co2": false,
  "use_tvoc": false,
  "use_fan": true,
  "use_relay": false,
  "use_extrelay": true,
  "use_tplug": false,
  "use_lcd": false
}
```
この中で利用する機能をtrueにして，/functionにPOSTします．

```
curl -XPOST 'http://.../function' -d '{"use_thermo":false,"use_humidity":true,"use_pressure":false,"use_doorsensor":false,"use_co2":false,"use_tvoc":false,"use_fan":true,"use_relay":false,"use_extrelay":true,"use_tplug":false,"use_lcd":false}'
```

* `use_extrelay`は，別のATOM HUB Switch.Dに本ソフトウェアをインストールしたものを，遠隔操作する昨日です．
  * 次節の設定で，`url_extrelay`を適切に設定してください．
* `use_extrelay`は`use_tplug`と排他利用になります．`use_extrelay`が優先されます．

## ヘルプ

`/help`にアクセスすると，APIが表示されます．
適宜，curl等でアクセスすることで，操作が可能です．

## 設定

`/config`にアクセスすると，設定一覧が取得できます．利用しない機能の設定も表示されます．

```
{
  "hostname": "leofetcher"
  "enable_push": true,
  "url_endpoint": "http://myserver.local:3000",
  "fan": {
    "id": "leofetch_fan",
    "manage_by_humid": true,
    "target_humid": 90
  },
  "relay1": {
    "id": "leofetch_heater",
    "manage_by_temp": true,
    "on_temp": 27.8,
    "off_temp": 28,
    "manage_by_time": false,
    "on_time": "10:00,12:00",
    "off_time": "11:00,18:00"
  },
  "relay2": {
    "id": "relay2",
    "manage_by_temp": false,
    "on_temp": 25,
    "off_temp": 30,
    "manage_by_time": false,
    "on_time": "10:00,11:00",
    "off_time": "10:10,11:10"
  },
  "url_extrelay": "http://myrelay.local/relay?1=",
  "host_tplug": "",
  "temp": {
    "id": "leofetch_temp"
  },
  "humidity": {
    "id": "leofetch_humid"
  },
  "pressure": {
    "id": "mypressure"
  },
  "door": {
    "id": "mydoor"
  },
  "co2": {
    "id": "myco2"
  },
  "tvoc": {
    "id": "mytvoc"
  },
  "fetch_start": [
    1733214626,
    0
  ]
}
```

* `url_extrelay`には，M5 HUB Switch.Dを外部からリレー操作するためのURLを指定します．
  * 具体的には，`http://(URL)/relay?(リレーの系統(1または2))=` と記述します．
* `host_tplug`には，TP Link HS105のIPアドレスを指定します．

## GitHub

https://github.com/omzn/
