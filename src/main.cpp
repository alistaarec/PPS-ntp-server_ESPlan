
//#include <SPI.h>
#include <Wire.h>
#include "DateTime.h"
#include "GPS.h"
#include <ETH.h>
#include <WifiUDP.h>
#include <SSD1306.h>

// GPS Seial debug
#define DEBUG false

// built-in LED
#define LED_PIN 33

// NTP port and packet buffer
unsigned int NTP_PORT = 123;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[ NTP_PACKET_SIZE ];

//GPS UART
//HardwareSerial Serial2(2);

//UDP server
WiFiUDP Udp;

// time of last set or update, so its after gps update
DateTime referenceTime;
DateTime originTime;
DateTime receiveTime;
DateTime transmitTime;

// GPS
static const int RXPin = 32, TXPin = 10;
static const uint32_t GPSBaud = 115200;
GPS gps(RXPin, TXPin, DEBUG);

//initialize i2C OLED LCD on esp32 I2C0 (pins 16 and 17)
SSD1306 display(0x3C, 16, 17); // Addr, SDA, SCL
u8_t lcdOffsetUpper = 8; // width in number of pixels for upper free space on lcd
u8_t lcdOffsetLeft = 5; // width in number of pixels for left free space on lcd

void writeLCD(uint8_t row, String txt)
 {
  display.setColor(BLACK);
  display.fillRect(lcdOffsetLeft, lcdOffsetUpper+(row*15), 128, 16);
  display.setColor(WHITE);
  display.drawString(lcdOffsetLeft, lcdOffsetUpper+(row*15), txt);
  display.display();

 }


// ethernet PHY event handler
static bool eth_connected = false;
void EthEvent(WiFiEvent_t event)
{
  String ip = "";
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      writeLCD(1,"ETH connected");
      digitalWrite(LED_PIN, LOW);
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      ip = ETH.localIP().toString();
      writeLCD(2,"IPv4: "+ip);

      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ip);
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      writeLCD(1,"ETH disconnected");
      writeLCD(2,"");
      digitalWrite(LED_PIN, HIGH);
      eth_connected = false;
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}


void setup() {
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Wire.begin(16,17);
  display.init(); // initialise the OLED
  display.clear();
  display.flipScreenVertically(); // does what is says
  //display.setFont(ArialMT_Plain_10); // does what it says
  
  // Set the origin of text to top left
  //display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
  
  
  Serial.begin(115200);
  
  //initializes UART2
  gps.setup();


  //while (!Serial);
  WiFi.mode(WIFI_OFF); // turn off wifi radio
  btStop(); // turn off ble radio
  WiFi.onEvent(EthEvent);
  ETH.begin();
  Udp.begin(NTP_PORT);

  Serial.println("NTP Server is running.");
  
  writeLCD(0,"ESP32 GPS NTP Server");
  
  
  referenceTime = gps.getZDA();
  //char lcdtime[20];
  //sprintf(lcdtime, "%2d:%2d:%2d.%2d", lcdnow.hour(), lcdnow.minute(), lcdnow.second(), lcdnow.centisecond() );
  //writeLCD(3,lcdtime);
  
  //updateLCDtime();
  
  }

void updateLCDtime()
{
  DateTime lcdnow = gps.now();
  String stime = "lastpkt: ";
  stime += lcdnow.toString();
  stime += " UTC";
  writeLCD(3,stime);

}






/**
 * Double to byte array
 */
void d2ba(double cent, byte *output) {
  // Serial.println(cent);

  int j = 0;

  for (int i = 0; i < 8; i++){
    byte tmp = int(cent * 16);
    cent *= 16;
    cent = cent - floor(cent);
    // Serial.print(cent, 10);
    // Serial.print(", ");
    if (i % 2 == 1) {
      output[j] += tmp;
      j++;
    } else {
      output[j] = tmp * 16;
    }
  }

  // Serial.println("Debug loop");
  // for (int i = 0; i < 4; i++) {
  //   Serial.println(output[i]);
  // }
  // Serial.println("End debug loop");
}


// send NTP reply to the given address
void sendNTPpacket(IPAddress remoteIP, int remotePort) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  // LI: 0, Version: 4, Mode: 4 (server)
  packetBuffer[0] = 0b00100100;
  // Stratum, or type of clock
  packetBuffer[1] = 0b00000001;
  // Polling Interval
  packetBuffer[2] = 6;
  // Peer Clock Precision
  // log2(sec)
  // 0xFA <--> -6 <--> 0.01 s
  packetBuffer[3] = 0xF9;
  
  // 8 bytes for Root Delay & Root Dispersion
  packetBuffer[7] = 0; // root dispersion
  packetBuffer[8] = 0;
  packetBuffer[9] = 0;
  packetBuffer[10] = 0xB;

  packetBuffer[11] = 0; // root delay
  packetBuffer[12] = 0;
  packetBuffer[13] = 0x16;
  packetBuffer[14] = 0;
  
  //time source (namestring)
  packetBuffer[12] = 71; // G
  packetBuffer[13] = 80; // P
  packetBuffer[14] = 83; // S
  packetBuffer[15] = 0;

  // Reference Time
  packetBuffer[16] = (referenceTime.ntptime() & 0xFF000000) >> 24;
  packetBuffer[17] = (referenceTime.ntptime() & 0x00FF0000) >> 16;
  packetBuffer[18] = (referenceTime.ntptime() & 0x0000FF00) >> 8;
  packetBuffer[19] = (referenceTime.ntptime() & 0x000000FF);
  byte refCent[4];
  d2ba((1.0 * referenceTime.centisecond() / 100), refCent);
  packetBuffer[20] = refCent[0];
  packetBuffer[21] = refCent[1];
  packetBuffer[22] = refCent[2];
  packetBuffer[23] = refCent[3];

  // Origin Time
  packetBuffer[24] = (originTime.ntptime() & 0xFF000000) >> 24;
  packetBuffer[25] = (originTime.ntptime() & 0x00FF0000) >> 16;
  packetBuffer[26] = (originTime.ntptime() & 0x0000FF00) >> 8;
  packetBuffer[27] = (originTime.ntptime() & 0x000000FF);
  packetBuffer[28] = (originTime.centisecond() & 0xFF000000) >> 24;
  packetBuffer[29] = (originTime.centisecond() & 0x00FF0000) >> 16;
  packetBuffer[30] = (originTime.centisecond() & 0x0000FF00) >> 8;
  packetBuffer[31] = (originTime.centisecond() & 0x000000FF);

  // Receive Time
  packetBuffer[32] = (receiveTime.ntptime() & 0xFF000000) >> 24;
  packetBuffer[33] = (receiveTime.ntptime() & 0x00FF0000) >> 16;
  packetBuffer[34] = (receiveTime.ntptime() & 0x0000FF00) >> 8;
  packetBuffer[35] = (receiveTime.ntptime() & 0x000000FF);
  byte recCent[4];
  d2ba(1.0 * receiveTime.centisecond() / 100, recCent);
  packetBuffer[36] = recCent[0];
  packetBuffer[37] = recCent[1];
  packetBuffer[38] = recCent[2];
  packetBuffer[39] = recCent[3];

  // Transmit Time
  //Serial.println("polling transmitTime");
  transmitTime = gps.getZDA();
  packetBuffer[40] = (transmitTime.ntptime() & 0xFF000000) >> 24;
  packetBuffer[41] = (transmitTime.ntptime() & 0x00FF0000) >> 16;
  packetBuffer[42] = (transmitTime.ntptime() & 0x0000FF00) >> 8;
  packetBuffer[43] = (transmitTime.ntptime() & 0x000000FF);
  byte traCent[4];
  d2ba(1.0 * transmitTime.centisecond() / 100, traCent);
  packetBuffer[44] = traCent[0];
  packetBuffer[45] = traCent[1];
  packetBuffer[46] = traCent[2];
  packetBuffer[47] = traCent[3];

  // all NTP fields have been given values, now
  // you can send a packet with a timestamp:
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();

}


void loop() {
  // NTP
  IPAddress remoteIP;
  int remotePort;
  int packetSize = Udp.parsePacket();

  if (packetSize) {
    receiveTime = gps.getZDA();
    //Serial.println("polled receiveTime from GPS");

    digitalWrite(LED_PIN, HIGH);

    remoteIP = Udp.remoteIP();
    remotePort = Udp.remotePort();

    Serial.print("Received UDP packet with ");
    Serial.print(packetSize);
    Serial.print(" bytes size - ");
    Serial.print("SourceIP ");
    
    for (int i =0; i < 4; i++)
    {
      Serial.print(remoteIP[i], DEC);
      if (i < 3)
      {
        Serial.print(".");
      }
    }
    //uint16_t port = Udp.remotePort();
    
    Serial.print(", Port ");
    Serial.println(remotePort);

    // We've received a packet, read the data from it
    // read the packet into the buffer
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:
    unsigned long highWordSecond = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWordSecond = word(packetBuffer[42], packetBuffer[43]);
    unsigned long highWordCentisecond = word(packetBuffer[44], packetBuffer[45]);
    unsigned long lowWordCentisecond = word(packetBuffer[46], packetBuffer[47]);

    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    originTime.time(highWordSecond << 16 | lowWordSecond);
    originTime.centisecond(highWordCentisecond << 16 | lowWordCentisecond);

    sendNTPpacket(remoteIP, remotePort);
    digitalWrite(LED_PIN, LOW);
    updateLCDtime();
    Serial.println("NTP packet sent.\r\n\r\n*******************\r\n");
    
  }
}


