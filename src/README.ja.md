# あくあたんリレーモジュール

| API     | parameter   | value   |   effect               |
|---------|-------------|---------|------------------------|
| /config | hostname    | String  | change hostname        |
| /config | name_relay1 | String  | change name of relay 1 |
| /config | name_relay2 | String  | change name of relay 2 |
| /status | --          |  --     | show status            |
| /relay  | [1|2]       | [on|off]| change relay           |
| /on     | [1|2]       |         | turn on relay          |
| /off    | [1|2]       |         | turn off relay         |

```
$ curl "http://aquatanrelay.local/status"
$ curl "http://aquatanrelay.local/relay?1=off&2=on"  # turn off relay 1 and turn on relay 2
$ curl "http://aquatanrelay.local/on?1="             # turn on relay 1 (requires "=" after the parameter name "1" )
$ curl "http://aquatanrelay.local/off"               # turn off all relays
```

## GitHub

