# TDM2421 Dual GPS V3

Fixes EG800K `AT+QGPSLOC=2` parsing for responses containing empty CSV fields.

Observed response:
`+QGPSLOC: 030336.00,21.06777,105.81160,1.54,-10.1,3,,0.046,0.025,150926,22`

V2 used `strtok_r()`, which discards the empty COG field between `3,,0.046` and shifts all later columns. V3 uses a manual CSV splitter that preserves empty fields.

Location priority remains:
1. fresh MAVLink location (<= 5 s)
2. fresh EG800K GNSS fix (<= 5 s)
3. NO_LOCATION

Build production firmware:
`make tdm2421`

Flash:
`make upload-TDM2421`
