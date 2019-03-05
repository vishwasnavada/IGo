
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "index.h" //Our HTML webpage contents with javascripts

#define LED 2  //On board LED
static const int RXPin = 12, TXPin = 13;
static const uint32_t GPSBaud = 9600;
//SSID and Password of your WiFi router
const char* ssid = "Ironman";
const char* password = "@Roomno52";
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
ESP8266WebServer server(80); //Server on port 80

//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void handleRoot() {
  String s = MAIN_page; //Read HTML contents
  server.send(200, "text/html", s); //Send web page
}

void handleADC() {
  int a = analogRead(A0);
  String adcValue = String(a);
  digitalWrite(LED, !digitalRead(LED)); //Toggle LED on data request ajax
  server.send(200, "text/plane", adcValue); //Send ADC value only to client ajax request
}
//==============================================================
//                  SETUP
//==============================================================
void setup(void) {
  Serial.begin(115200);
    ss.begin(GPSBaud);
  WiFi.begin(ssid, password);     //Connect to your WiFi router
  Serial.println("");

  //Onboard LED port Direction output
  pinMode(LED, OUTPUT);

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

  server.on("/", handleRoot);      //Which routine to handle at root location. This is display page
  server.on("/readADC", handleADC); //This page is called by java Script AJAX

  server.begin();                  //Start server
  Serial.println("HTTP server started");
}
//==============================================================
//                     LOOP
//==============================================================
void loop(void) {
  double latitude = (gps.location.lat());
  double longitude = (gps.location.lng());
  Serial.println("http://www.google.com/maps/place/12.9815323,77.6435257");
  Serial.print(latitude);
  Serial.print(",");
  Serial.println(longitude);
  delay(500);
  server.handleClient();          //Handle client requests
}
