/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-esp8266-web-server-outputs-momentary-switch/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <WiFi.h>
//#include "ESPAsyncTCP.h"
//#include "ESPAsyncWebServer.h"
//#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
//#include <ESPAsyncWebSrv.h>

#include "index.h"
#include "credentials.h"

// REPLACE WITH YOUR NETWORK CREDENTIALS
//const char* ssid = "REPLACE_WITH_YOUR_SSID";
//const char* password = "REPLACE_WITH_YOUR_PASSWORD";
const char* ssid      = "UPC39253B3";
const char* password  = "TT6cukds4mfj";
IPAddress local_IP(192, 168, 0, 228);   
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 0, 1);

const int output1 = 22;
const int output2 = 22;


void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

AsyncWebServer server(80);

const char* PARAM_INPUT_1 = "dauersend";
String dauerstate = "abD";

void handle_dauer(AsyncWebServerRequest *request) {
  String inputMessage;
  String inputParam;
  // GET input1 value on <ESP_IP>/dauer?dauersend=<inputMessage>
  if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      dauerstate = inputMessage;
      inputParam = PARAM_INPUT_1;
      Serial.println(inputParam);
      Serial.println(inputMessage);
  }  
 request->send(200, "text/plane", inputMessage); //Send web page
}


// if(t_state == "1")
// {
//  digitalWrite(LED,LOW); //LED ON
//  ledState = "ON"; //Feedback parameter
// }
// else
// {
//  digitalWrite(LED,HIGH); //LED OFF
//  ledState = "OFF"; //Feedback parameter  
// }
// 

void setup() {
  Serial.begin(115200);
  Serial.println("button_xhttp1");
  WiFi.setHostname(HOSTNAME);
  WiFi.config(local_IP, subnet, gateway);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("ESP IP Address: http://");
  Serial.println(WiFi.localIP());
  
  pinMode(output1, OUTPUT);
  pinMode(output2, OUTPUT);
  digitalWrite(output1, LOW);
  digitalWrite(output2, LOW);
  
  // Send web page to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", index_html);
  });

  // Receive an HTTP GET request
  server.on("/auf", HTTP_GET, [] (AsyncWebServerRequest *request) {
    digitalWrite(output1, LOW);
    delay(100);
    digitalWrite(output2, HIGH);
    Serial.println("Relais: 0-1");
    request->send(200, "text/plain", "ok");
  });

  server.on("/dauer", HTTP_GET, handle_dauer);
//    digitalWrite(output1, LOW);
//    delay(100);
//    digitalWrite(output2, HIGH);
//    Serial.println("Relais: 0-1");
//    request->send(200, "text/plain", "ok");
//  });

  server.on("/ab", HTTP_GET, [] (AsyncWebServerRequest *request) {
    digitalWrite(output1, HIGH);
    delay(100);
    digitalWrite(output2, HIGH);
    Serial.println("Relais: 1-1");
    request->send(200, "text/plain", "ok");
  });

//  server.on("/dauer/abD", HTTP_GET, [] (AsyncWebServerRequest *request) {
//    digitalWrite(output1, HIGH);
//    delay(100);
//    digitalWrite(output2, HIGH);
//    Serial.println("Relais: 1-1");
//    request->send(200, "text/plain", "ok");
//  });

  server.on("/stop", HTTP_GET, [] (AsyncWebServerRequest *request) {
    digitalWrite(output1, LOW);
    delay(100);
    digitalWrite(output2, LOW);
    Serial.println("Relais: 0-0");
    request->send(200, "text/plain", "ok");
  });

//  server.on("/dauer/stopD", HTTP_GET, [] (AsyncWebServerRequest *request) {
//    digitalWrite(output1, LOW);
//    delay(100);
//    digitalWrite(output2, LOW);
//    Serial.println("Relais: 0-0");
//    request->send(200, "text/plain", "ok");
//  });

  server.onNotFound(notFound);
  server.begin();
}

void loop() {
 
}
