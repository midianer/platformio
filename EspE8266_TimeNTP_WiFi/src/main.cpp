#include <ESP8266WiFi.h>
#include "time.h" // Built-in library
#include <ESPAsyncWebServer.h>
#include "index.h" //Our HTML webpage contents with javascripts

//#define myTZ "PKT-5"
//#define dst 0

const char* ssid       = "UPC39253B3";
const char* password   = "TT6cukds4mfj";

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

IPAddress local_IP(192, 168, 0, 228);   
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 0, 1);
IPAddress dns1(8,8,8,8);
IPAddress dns2(8,8,4,4);

AsyncWebServer server(80);

//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void handleRoot(AsyncWebServerRequest *request) {
  String s = MAIN_page; //Read HTML contents
  request->send(200, "text/html", s); //Send web page
}

void handleDate(AsyncWebServerRequest *request) {
  int sec;
  int min;
  int hour;
  int mday;
  int mon;
  int year;
  String _time_val;
  char  _time[36];

  struct tm timeinfo;
  if(getLocalTime(&timeinfo)){
    sec = timeinfo.tm_sec;
    min = timeinfo.tm_min;
    hour = timeinfo.tm_hour;
    mday = timeinfo.tm_mday;   /* Day of the month [1, 31] */
    mon = timeinfo.tm_mon;    /* Month            [0, 11]  (January = 0) */
    year = timeinfo.tm_year;   /* Year minus 1900 */
  } else
    return;
  sprintf(_time, "%04d.%02d.%02d %02d:%02d:%02d", year+1900, mon+1, mday, hour, min, sec);
  _time_val = String(_time);
//  request->send(200, "text/plane", _sec_val);
  request->send(200, "text/plane", _time_val);
}


void handleRSSI(AsyncWebServerRequest *request) {
  String _rssi_val;
  long rssi = WiFi.RSSI();
  _rssi_val = String(rssi);
  request->send(200, "text/plane", _rssi_val);
}

void connectToWiFi()
{
  WiFi.setHostname("EspE8266_TimeNTP_WiFi");
  WiFi.config(local_IP, gateway, subnet, dns1, dns2);
  WiFi.begin(ssid, password); 
  while(WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500); 
  }

  Serial.println ("Connected ..."); 
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); 
}

//uint32_t sntp_update_delay_MS_rfc_not_less_than_15000 ()
//{
//    //info_sntp_update_delay_MS_rfc_not_less_than_15000_has_been_called = true;
//    return 60000 * 60; // 1 hour
//}

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
    return;
  }
  //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t)
{
  //Serial.println("Got time adjustment from NTP!");
  printLocalTime();
}

void setup()
{
  delay(1000);
  Serial.begin(115200);
  Serial.println ("EspE8266_TimeNTP_WiFi");
  connectToWiFi();
  //configTime(5*3600,0,"pool.ntp.org");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  //configTime(myTZ,"pool.ntp.org"); some how this doesnt work for me, i have no clue why

  //sntp_set_time_sync_notification_cb( timeavailable ); 

  while (time(nullptr) < 1617460172) // minimum valid epoch
  {
    Serial.println("I Am Time");
    Serial.println(time(nullptr));
    delay(100);
  }

  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP
  long rssi = WiFi.RSSI();
  Serial.print("RSSI:");
  Serial.println(rssi);
  
  server.on("/", HTTP_GET, handleRoot);      //Which routine to handle at root location. This is display page
  server.on("/GetDate", handleDate);
  server.on("/GetRSSI", handleRSSI);

  server.begin();                  //Start server
  Serial.println("HTTP server started");

}


time_t rawtime; 

void loop()
{
  int sec, hour;
 
  struct tm* p_timeinfo;  
  struct tm  timeinfo;  
  time(&rawtime);

  p_timeinfo = localtime(&rawtime); 
  char buffer[80];

  strftime(buffer, 80, "%Y%m%d %r",p_timeinfo); 
 
  getLocalTime(&timeinfo);
  hour = timeinfo.tm_hour;
  sec = timeinfo.tm_sec;
  Serial.println(hour);
  Serial.println(sec);


  Serial.println(buffer);
  delay (1000); 
  
}



//11//
//11///*
//11// * TimeNTP_ESP8266WiFi.ino
//11// * Example showing time sync to NTP time source
//11// *
//11// * This sketch uses the ESP8266WiFi library
//11// */
//11//#include "Arduino.h"
//11//#include <WiFiNINA.h> // for UNO Wifi Rev 2 or Nano RP2040 connect
//11////#include "WiFiUdp.h" // not needed for WiFiNINA
//11////#include "NTP.h"
//11////#include <TimeLib.h>
//11//
//11//#include <ESP8266WiFi.h>
//11//#include <WiFiUdp.h>
//11////#include <TimeLib.h>
//11//
//11////WiFiUDP wifiUdp;
//11////NTP ntp(wifiUdp);
//11//
//11//const char ssid[] = "UPC39253B3";
//11//const char pass[] = "TT6cukds4mfj";
//11//
//11//// NTP Servers:
//11//static const char ntpServerName[] = "us.pool.ntp.org";
//11////static const char ntpServerName[] = "time.nist.gov";
//11////static const char ntpServerName[] = "time-a.timefreq.bldrdoc.gov";
//11////static const char ntpServerName[] = "time-b.timefreq.bldrdoc.gov";
//11////static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";
//11//
//11//const int timeZone = 1;     // Central European Time
//11////const int timeZone = -5;  // Eastern Standard Time (USA)
//11////const int timeZone = -4;  // Eastern Daylight Time (USA)
//11////const int timeZone = -8;  // Pacific Standard Time (USA)
//11////const int timeZone = -7;  // Pacific Daylight Time (USA)
//11//
//11//byte mac[6];
//11//
//11//WiFiUDP Udp;
//11//unsigned int localPort = 8888;  // local port to listen for UDP packets
//11//
//11//time_t getNtpTime();
//11//void digitalClockDisplay();
//11//void printDigits(int digits);
//11//void sendNTPpacket(IPAddress &address);
//11//
//11//void setup()
//11//{
//11//  Serial.begin(115200);
//11//  Serial.println("\n\nprg: TimeNTP_ESP8266WiFi");
//11//  while (!Serial) ; // Needed for Leonardo only
//11//  delay(500);
//11//  Serial.println("\n\n\n");
//11//  Serial.println("TimeNTP Example");
//11//  Serial.print("Connecting to ");
//11//  Serial.println(ssid);
//11//  WiFi.begin(ssid, pass);
//11//
//11//  WiFi.macAddress(mac);
//11//  Serial.print("MAC: ");
//11//  Serial.print(mac[0],HEX);
//11//  Serial.print(":");
//11//  Serial.print(mac[1],HEX);
//11//  Serial.print(":");
//11//  Serial.print(mac[2],HEX);
//11//  Serial.print(":");
//11//  Serial.print(mac[3],HEX);
//11//  Serial.print(":");
//11//  Serial.print(mac[4],HEX);
//11//  Serial.print(":");
//11//  Serial.println(mac[5],HEX);
//11//
//11//  while (WiFi.status() != WL_CONNECTED) {
//11//    delay(500);
//11//    Serial.print(".");
//11//  }
//11//  Serial.print("IP number assigned by DHCP is ");
//11//  Serial.println(WiFi.localIP());
//11//  Serial.println("Starting UDP");
//11//  Udp.begin(localPort);
//11//  Serial.print("Local port: ");
//11//  Serial.println(Udp.localPort());
//11//  Serial.println("waiting for sync");
//11//  setSyncProvider(getNtpTime);
//11//  setSyncInterval(1);
//11//  pinMode(12, OUTPUT);
//11//  pinMode(14, OUTPUT);
//11//}
//11//
//11//time_t prevDisplay = 0; // when the digital clock was displayed
//11//time_t day_time;
//11//
//11//void loop()
//11//{
//11//  if (timeStatus() != timeNotSet) {
//11//    setSyncInterval(20);
//11//    if (now() != prevDisplay) { //update the display only if time has changed
//11//      prevDisplay = now();
//11//      day_time = prevDisplay % 86400L;
//11//      Serial.println(prevDisplay);
//11//      Serial.println(day_time);
//11//      digitalClockDisplay();
//11//      if ((second() % 10) == 6) digitalWrite(12, HIGH);
//11//      if ((second() % 10) == 1) digitalWrite(12, LOW);
//11//      if ((second() % 10) == 5) digitalWrite(14, HIGH);
//11//      if ((second() % 10) == 2) digitalWrite(14, LOW);
//11//    }
//11//  }
//11//}
//11//
//11//void digitalClockDisplay()
//11//{
//11//  // digital clock display of the time
//11//  Serial.print(hour());
//11//  printDigits(minute());
//11//  printDigits(second());
//11//  Serial.print(" ");
//11//  Serial.print(day());
//11//  Serial.print(".");
//11//  Serial.print(month());
//11//  Serial.print(".");
//11//  Serial.print(year());
//11//  Serial.print("-");
//11//  Serial.print(weekday());
//11//  Serial.println();
//11//}
//11//
//11//void printDigits(int digits)
//11//{
//11//  // utility for digital clock display: prints preceding colon and leading 0
//11//  Serial.print(":");
//11//  if (digits < 10)
//11//    Serial.print('0');
//11//  Serial.print(digits);
//11//}
//11//
//11///*-------- NTP code ----------*/
//11//
//11//const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
//11//byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
//11//
//11//time_t getNtpTime()
//11//{
//11//  IPAddress ntpServerIP; // NTP server's ip address
//11//
//11//  while (Udp.parsePacket() > 0) ; // discard any previously received packets
//11//  Serial.println("Transmit NTP Request");
//11//  // get a random server from the pool
//11//  WiFi.hostByName(ntpServerName, ntpServerIP);
//11//  Serial.print(ntpServerName);
//11//  Serial.print(": ");
//11//  Serial.println(ntpServerIP);
//11//  sendNTPpacket(ntpServerIP);
//11//  uint32_t beginWait = millis();
//11//  while (millis() - beginWait < 1500) {
//11//    int size = Udp.parsePacket();
//11//    if (size >= NTP_PACKET_SIZE) {
//11//      Serial.println("Receive NTP Response");
//11//      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
//11//      unsigned long secsSince1900;
//11//      // convert four bytes starting at location 40 to a long integer
//11//      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
//11//      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
//11//      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
//11//      secsSince1900 |= (unsigned long)packetBuffer[43];
//11//      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
//11//    }
//11//  }
//11//  Serial.println("No NTP Response :-(");
//11//  return 0; // return 0 if unable to get the time
//11//}
//11//
//11//// send an NTP request to the time server at the given address
//11//void sendNTPpacket(IPAddress &address)
//11//{
//11//  // set all bytes in the buffer to 0
//11//  memset(packetBuffer, 0, NTP_PACKET_SIZE);
//11//  // Initialize values needed to form NTP request
//11//  // (see URL above for details on the packets)
//11//  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
//11//  packetBuffer[1] = 0;     // Stratum, or type of clock
//11//  packetBuffer[2] = 6;     // Polling Interval
//11//  packetBuffer[3] = 0xEC;  // Peer Clock Precision
//11//  // 8 bytes of zero for Root Delay & Root Dispersion
//11//  packetBuffer[12] = 49;
//11//  packetBuffer[13] = 0x4E;
//11//  packetBuffer[14] = 49;
//11//  packetBuffer[15] = 52;
//11//  // all NTP fields have been given values, now
//11//  // you can send a packet requesting a timestamp:
//11//  Udp.beginPacket(address, 123); //NTP requests are to port 123
//11//  Udp.write(packetBuffer, NTP_PACKET_SIZE);
//11//  Udp.endPacket();
//11//}
//11//
