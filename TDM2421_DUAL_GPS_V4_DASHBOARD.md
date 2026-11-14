# TDM2421 Dual GPS V4 - Full GNSS Dashboard

Adds EG800K GNSS diagnostics to /api/status and the local dashboard:
engine state, valid fix, 2D/3D status, satellites reported, satellites used from NMEA GSA,
PDOP/HDOP/VDOP, UTC/date, fix age, GNSS latitude/longitude/altitude/speed/course.

The firmware enables NMEA acquisition with AT+QGPSCFG="nmeasrc",1 and requests
AT+QGPSGNMEA="GSA" every ~5 s. If GSA is unavailable on a firmware variant, DOP-derived
fields show N/A; QGPSLOC positioning continues normally.

"3D FIX" means a normal 3D GNSS solution. It is not RTK Fixed. This code does not invent
RTK Float/Fixed state when the EG800K does not report RTK status.
