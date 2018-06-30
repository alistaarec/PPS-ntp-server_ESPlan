#include <Wire.h>
#include <ETH.h>
#include <WifiUDP.h>
#include <SSD1306.h>
#include "DateTime.h"
#include "GPS.h"

// GPS debug
#define DEBUG false

// built-in LED
#define LED_PIN 33

// NTP port and packet buffer
#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
byte packetBuffer[NTP_PACKET_SIZE];

//UDP server (ETH.h uses WifiUDP class)
WiFiUDP Udp;

// global timestamp storage
DateTime referenceTime;
DateTime originTime;
DateTime receiveTime;
DateTime transmitTime;

// refresh timestamp storage
unsigned long nowtime;
#define referenceTimeRefresh 3600000 //3600000 ms == 1h; interval in ms for getting reference time from GPS

// GPS init
#define RXPin 32
#define TXPin  10
#define GPSBaud 115200
#define gpsTimeOffset 4 //centisecond raw offset, compared to known-good stratum 1 server
HardwareSerial Serial2(2); // for initial config at 9600 baud
HardwareSerial Serial1(1); // for GPS comm at 115200 baud

GPS gps(RXPin, TXPin, DEBUG);

//initialize i2C OLED on esp32 I2C0 (pins 16 and 17)
#define SDApin 16
#define SCLpin 17
SSD1306 display(0x3C, SDApin, SCLpin); // Addr, SDA, SCL
#define lcdOffsetUpper  8 // width in number of pixels for upper free space on lcd
#define lcdOffsetLeft  4 // width in number of pixels for left free space on lcd

// rocking switch on PIN5 to toggle between serialMirror/NTPserver modes
#define SWITCH_PIN 5
static bool SerialMirror = 0;


void writeLCD(uint8_t row, String txt)
 {
  display.setColor(BLACK);
  display.fillRect(lcdOffsetLeft, lcdOffsetUpper+(row*15), 128, 16);
  display.setColor(WHITE);
  display.drawString(lcdOffsetLeft, lcdOffsetUpper+(row*15), txt);
  display.display();
 }


static bool eth_connected = false;
// ethernet PHY event handler
void EthEvent(WiFiEvent_t event)
{
  String ip = "";
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      writeLCD(1,"ETH started");
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
      writeLCD(2,"ETH stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}


void setup() {

  pinMode(SWITCH_PIN, INPUT);
  digitalWrite(SWITCH_PIN, LOW);

  //turn builtin LED off
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  //start i2c
  Wire.begin(SDApin,SCLpin);

  // initialise the OLED
  display.init(); 
  display.clear();
  display.flipScreenVertically(); 
  //display.setFont(ArialMT_Plain_10); // does what it says
  // Set the origin of text to top left
  //display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
  
  //start UART0 (for time & debug output)
  Serial.begin(115200);
  
  //initializes UART2 (for GPS comm)
  gps.setup();


  WiFi.mode(WIFI_OFF); // turn off wifi radio
  btStop(); // turn off ble radio
  WiFi.onEvent(EthEvent); //attach ETH PHY event handler
  ETH.begin(); //start ETH
  Udp.begin(NTP_PORT); // start udp server

  Serial.println("NTP Server is running.");
  writeLCD(0,"ESP32 GPS NTP Server");
  
  //get initial reference time
  referenceTime = gps.getZDA();
  int diff = referenceTime.centisecond() + gpsTimeOffset; 
  if (diff <= 99)
    {
      referenceTime.centisecond(diff);
    }
    else
    {
      referenceTime.time(referenceTime.ntptime() + 1);
      referenceTime.centisecond(diff-100);
    }  

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
 * Double to byte array (helper function)
 */
void d2ba(double cent, byte *output) 
{
  // Serial.println(cent);
  uint8_t j = 0;

  for (uint8_t i = 0; i < 8; i++){
    byte tmp = int(cent * 16);
    cent *= 16;
    cent = cent - floor(cent);
    // Serial.print(cent, 10);
    // Serial.print(", ");
    if (i % 2 == 1) 
    {
      output[j] += tmp;
      j++;
    } 
    else 
    {
      output[j] = tmp * 16;
    }
  }

}


// send NTP reply to the given address
void sendNTPpacket(IPAddress remoteIP, int remotePort) 
{
  // set all bytes in the packet buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);

  // Initialize values needed to form NTP request
  // (see URL in readme.md for details on packet fields)
  
  // LI: 0, Version: 4, Mode: 4 (server)
  packetBuffer[0] = 0b00100100;
  
  // Stratum, or type of clock
  packetBuffer[1] = 0b00000001;
  
  // Polling Interval
  packetBuffer[2] = 0x6;

  // Peer Clock Precision
  // log2(sec)
  // 0xF9 <--> -7 <--> 0.0078125 s
  // 0xFA <--> -6 <--> 0.0156250 s
  // 0xFB <--> -5 <--> 0.0312500 s 
  // in fact we have 0.01 s accuracy, so we announce a clock precision of -6
  packetBuffer[3] = 0xFA;
  
  // 8 bytes for Root Delay & Root Dispersion
  // root delay
  packetBuffer[4] = 0; 
  packetBuffer[5] = 0;
  packetBuffer[6] = 0; 
  packetBuffer[7] = 0;
  
  // root dispersion
  packetBuffer[8] = 0;
  packetBuffer[9] = 0;
  packetBuffer[10] = 0;
  packetBuffer[11] = 0; // 0x78  <--> 0x0000.0078 <--> +-0,3052 ms root dispersion
  
  
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
  d2ba((1.0 * receiveTime.centisecond() / 100), recCent);
  packetBuffer[36] = recCent[0];
  packetBuffer[37] = recCent[1];
  packetBuffer[38] = recCent[2];
  packetBuffer[39] = recCent[3];

  // Transmit Time
  //Serial.println("polling transmitTime");
  transmitTime = gps.getZDA();
  
  int diff = transmitTime.centisecond() + gpsTimeOffset;
  if (diff <= 99)
  {
    transmitTime.centisecond(diff);
  }
  else
  {
    transmitTime.time(transmitTime.ntptime() + 1);
    transmitTime.centisecond(diff-100);
  }
    
  packetBuffer[40] = (transmitTime.ntptime() & 0xFF000000) >> 24;
  packetBuffer[41] = (transmitTime.ntptime() & 0x00FF0000) >> 16;
  packetBuffer[42] = (transmitTime.ntptime() & 0x0000FF00) >> 8;
  packetBuffer[43] = (transmitTime.ntptime() & 0x000000FF);
  byte traCent[4];
  d2ba((1.0 * transmitTime.centisecond() / 100), traCent);
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


void loop() 
{

  SerialMirror = digitalRead(SWITCH_PIN);

  if (SerialMirror)
  {
    writeLCD(1,"Serial Monitor mode");

    if (Serial1.available())
    {
        Serial.write(Serial1.read());
    }

    if (Serial.available())
    {
        //Serial2.write("$EIGLQ,ZDA*25\r\n");
        Serial1.write(Serial.read());
    }

  }
  else
  {

    writeLCD(1, "NTP server mode");

    // process NTP requests
    IPAddress remoteIP;
    int remotePort;

    int packetSize = Udp.parsePacket();

    if (packetSize) 
    {
      
      digitalWrite(LED_PIN, HIGH);

      //store sender ip and port for later use
      remoteIP = Udp.remoteIP();
      remotePort = Udp.remotePort();

      Serial.print("Received UDP packet with ");
      Serial.print(packetSize);
      Serial.print(" bytes size - ");
      Serial.print("SourceIP ");
      
      for (uint8_t i =0; i < 4; i++)
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

    //Serial.println("polling receiveTime from GPS");
      receiveTime = gps.getZDA();
      int diff = receiveTime.centisecond() + gpsTimeOffset; 
      if (diff <= 99)
      {
        receiveTime.centisecond(diff);
      }
      else
      {
        receiveTime.time(receiveTime.ntptime() + 1);
        receiveTime.centisecond(diff-100);
      }

      //Serial.println(receiveTime.centisecond());
      
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

      //finally: send reply
      sendNTPpacket(remoteIP, remotePort);
    
      digitalWrite(LED_PIN, LOW);
    
      //output "done"
      updateLCDtime();
      Serial.println("NTP packet sent.\r\n\r\n*******************\r\n");
      
    }
    else // no packet received, we can do other stuff now
    {
      if (millis() >= (nowtime + referenceTimeRefresh))
      {
          nowtime = millis();
          referenceTime = gps.getZDA();
          int diff = referenceTime.centisecond() + gpsTimeOffset; 
          if (diff <= 99)
            {
              referenceTime.centisecond(diff);
            }
            else
            {
              referenceTime.time(referenceTime.ntptime() + 1);
              referenceTime.centisecond(diff-100);
            }  

          
      }
    }
  }
}


