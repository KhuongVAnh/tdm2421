# TDM2421 VIN power-up fix

Change: assert `PIN_SIM_PWR_EN` (GPIO2) HIGH at the very beginning of `setup()` and wait 500 ms, before `g.init()` and Wi-Fi initialization. This matches the known-good Arduino test sketch power sequence.

The later duplicate GPIO2 initialization was removed. DTR/UART initialization remains in its original location. Added USB debug logs for SIM power, DTR, and EG800K UART startup.

Build note: compilation could not be completed in the isolated build environment because the ESP32 Arduino platform is not installed and network access to Arduino package indexes is unavailable. The source edit itself was applied successfully.
