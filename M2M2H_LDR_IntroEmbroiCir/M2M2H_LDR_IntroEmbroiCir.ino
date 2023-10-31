/* This program is to run 2 LDRs and a single led on a firebeetle
  esp32 for an embroidered circuit. Make sure to install FireBeetle -ESP32 drivers
  http://arduino.esp8266.com/stable/package_esp8266com_index.json,http://download.dfrobot.top/FireBeetle/package_esp32_index.json,https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  Do this by putting link into Additonal board managers in url of arduio prefrences.
  Also make sure you are using an upload speed of 115200 and using board
  FireBeetle-ESP32 in board manager. */

#include <WiFi.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>

// WiFi credentials
const char *ssid = "J206";// name of wifi
const char *password = "dontgotowales";// password for wifi

// OSC Setup
WiFiUDP Udp;
const IPAddress outIp(192, 168, 0, 103); // Put Ip address of computer trying to send data to
const unsigned int outPort = 8005; //
const unsigned int localPort = 8888; //

// Pins
const int ldrPin = A3; // LDR connected to pin A3
const int ldrPin2 = A2; //LDR connected to pin A2
const int ledPin = D7; // LED connected to pin D7

// Variables for smoothing
float alpha = 0.1;
float smoothedLdrValue = 0;
float smoothedLdrValue2 = 0;
void setup() {

  pinMode(ldrPin, INPUT); //pin function as input
  pinMode(ldrPin2, INPUT);
  pinMode(ledPin, OUTPUT);//pin function as output

  Serial.begin(115200);//speed at which microcontroller is speaking

  // Connect to WiFi
  Serial.print("Connecting to WiFi...");//serial message for connecting to wifi
  WiFi.begin(ssid, password);// finds wifi and plugs in password

  while (WiFi.status() != WL_CONNECTED) {// establishing connection
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected.");

  Udp.begin(localPort);// begin connection to computer
}

void loop() {

  int ldrValue = analogRead(ldrPin);
  smoothedLdrValue = alpha * ldrValue + (1 - alpha) * smoothedLdrValue;//smoothing data from LDR1

  Serial.print("Smoothed LDR Value 1: ");
  Serial.println((int)smoothedLdrValue); //printing smoothed value

  int ldrValue2 = analogRead(ldrPin2);
  smoothedLdrValue2 = alpha * ldrValue2 + (1 - alpha) * smoothedLdrValue2; // smoothing data from LDR2

  Serial.print("Smoothed LDR Value 2:");
  Serial.println((int)smoothedLdrValue2);//printing smoothed value

  // Turn LED on or off based on LDR value these values need to be latered as ldrs may differ in sensitivity
  if (smoothedLdrValue < 800) {
    digitalWrite(ledPin, HIGH); // turn on led if ldr 1 is below 800
  }
  else if (smoothedLdrValue2 < 100) {
    digitalWrite(ledPin, HIGH); // turn on led if ldr 2 is below 100
  }
  else {
    digitalWrite(ledPin, LOW); //led is off if other functions are not true
  }

  // Create OSC message
  OSCMessage msg("/ldrValues"); //writing messages over osc to computer
  msg.add((int)smoothedLdrValue);
  msg.add((int)smoothedLdrValue2);
  // Send OSC message
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty(); // Clear the message

  delay(100);
}
