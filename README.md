# ESP32 NTP Server

NTP v4 time server implementation using NEO-6MV2 GPS module as a reference clock. Mostly based on https://github.com/sigorilla/arduino-ntp-server with some modifications to work on espressif ESP32 platform, and to use the global positioning system (GPS) instead of the russian GLONASS. Specifically, the code is designed to run on an "olimex ESP32-GATEWAY" development board incorporating a RJ45 100MBit ethernet port, but can easily be adapted to work with other ESP32 boards using external ethernet connectivity, or WiFi connection, if desired. Also, a cheap 128x64 0.96" Oled LCD is used for displaying basic information. Average precision is around 40 ms, or 0.04 seconds, or 4 centiseconds (the smallest reliable time unit this GPS module can deliver) offset when compared to the NTP server [ptbtime1.ptb.de](https://www.ptb.de/cms/en/ptb/fachabteilungen/abtq/gruppe-q4/ref-q42/time-synchronization-of-computers-using-the-network-time-protocol-ntp.html). "ESP32 NTP server" uses DHCP for IP address retrieval, and replies to NTP queries with stratum 1 flag (test it with w32time.exe: "w32tm.exe /stripchart /computer:<IP-of-NTP-server> /period:1 /packetinfo").

## Hardware

* CPU — `Olimex ESP32-Gateway` (or other espressif ESP32 board)
* GPS — `GY-NEO6MV2`


### Wiring
```
GPS <-> ESP32
-----------------
GND --> GND
Tx  --> PIN10 (Rx)
Rx  --> PIN32 (Tx)
VCC --> +3.3V
```


## Links

* [u-center software](https://www.u-blox.com/en/product/u-center-windows)
* [u-blox command reference manual](https://www.u-blox.com/sites/default/files/products/documents/u-blox6-GPS-GLONASS-QZSS-V14_ReceiverDescrProtSpec_%28GPS.G6-SW-12013%29_Public.pdf)
* [NTP packet reference](https://www.meinbergglobal.com/english/info/ntp-packet.htm)
