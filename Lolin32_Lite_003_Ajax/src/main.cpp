/*
 * ESP32 NodeMCU AJAX Demo
 * Updates and Gets data from webpage without page refresh
 * https://circuits4you.com
 */
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

#include "index.h" //Our HTML webpage contents with javascripts

#define LED 2  //On board LED

//SSID and Password of your WiFi router
//const char* ssid = "circuits4you.com";
//const char* password = "123456789";
const char* ssid      = "UPC39253B3";
const char* password  = "TT6cukds4mfj";
IPAddress local_IP(192, 168, 0, 228);   
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 0, 1);

volatile int sig_down = 0;

AsyncWebServer server(80);

//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void handleRoot(AsyncWebServerRequest *request) {
 String s = MAIN_page; //Read HTML contents
 request->send(200, "text/html", s); //Send web page
}

void handleADC(AsyncWebServerRequest *request) {
 int a = analogRead(A0);
 String adcValue = String(a);
 
 request->send(200, "text/plane", adcValue); //Send ADC value only to client ajax request
}

void handleLED(AsyncWebServerRequest *request) {
 String ledState = "OFF";
 String t_state = request->arg("LEDstate"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 Serial.print("LEDstate :");
 Serial.println(t_state);
 if(t_state == "1")
 {
  digitalWrite(LED,LOW); //LED ON
  ledState = "ON"; //Feedback parameter
 }
 else
 {
  digitalWrite(LED,HIGH); //LED OFF
  ledState = "OFF"; //Feedback parameter  
 }
 
 request->send(200, "text/plane", ledState); //Send web page
}

void handleDown(AsyncWebServerRequest *request) {
  Serial.println("Down Pressed");
  sig_down = 10;
  String sig_down_s = String(sig_down);
  request->send(200, "text/plane", "Down Pressed");
}

void handleDownState(AsyncWebServerRequest *request) {
  String sig_down_s = String(sig_down);
  Serial.println(sig_down_s);
  request->send(200, "text/plane", sig_down_s);
}



//==============================================================
//                  SETUP
//==============================================================
void setup(void){
  Serial.begin(115200);
  Serial.println("button_ajax");
  
  // Connect to Wi-Fi
  WiFi.config(local_IP, subnet, gateway);
  WiFi.begin(ssid, password);     //Connect to your WiFi router
  Serial.println("");

  //Onboard LED port Direction output
  pinMode(LED,OUTPUT); 
  
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP
 
  server.on("/", HTTP_GET, handleRoot);      //Which routine to handle at root location. This is display page
  server.on("/setLED", handleLED);
  server.on("/readADC", handleADC);
  server.on("/Down", handleDown);
  server.on("/DownState", handleDownState);
  

  server.begin();                  //Start server
  Serial.println("HTTP server started");
}
//==============================================================
//                     LOOP
//==============================================================
unsigned long currentTime = millis();
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 1000;    // the debounce time; increase if the output flickers
void loop(void){
  if ((millis() - lastDebounceTime) > debounceDelay) {
	lastDebounceTime = millis();
    currentTime += 1000;
    Serial.println(lastDebounceTime);
    Serial.println(sig_down);
    if (sig_down > 0) {
      sig_down--;
    }
  }
//  server.handleClient();          //Handle client requests
}
