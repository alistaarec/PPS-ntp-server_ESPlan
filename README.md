# ESP32 NTP Server

NTP v3 server implementation using NEO-6MV2 GPS module as a reference clock. Mostly based on https://github.com/sigorilla/arduino-ntp-server with some modifications to work on espressif ESP32 platform and to use the global positioning system (GPS) instead of the russian GLONASS. Specifically, the code is designed to run on an "olimex ESP32-GATEWAY" development board incorporating a RJ45 100MBit ethernet port, but can easily be adapted to work with WiFi connection, if desired. Also, a cheap 128x64 0.96" OLED lcd is used for displaying basic information.

## Hardware

Olimex ESP32-Gateway (or other espressif ESP32 board)
GPS — `GY-NEO6MV2`


### GPS
```
GPS <-> ESP32
-----------------
GND --> GND
Tx  --> PIN10 (Rx)
Rx  --> PIN32 (Tx)
VCC --> +3.3V
```


## Links

* [u-center](https://www.u-blox.com/en/product/u-center-windows)
* [u-blox reference manual](https://www.u-blox.com/sites/default/files/products/documents/u-blox6-GPS-GLONASS-QZSS-V14_ReceiverDescrProtSpec_%28GPS.G6-SW-12013%29_Public.pdf)
* [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus)


