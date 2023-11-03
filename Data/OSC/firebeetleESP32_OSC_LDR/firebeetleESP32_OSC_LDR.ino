#include <WiFi.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>

// WiFi credentials
const char *ssid = "J206";
const char *password = "dontgotowales";

// OSC Setup
WiFiUDP Udp;
const IPAddress outIp(192, 168, 0, 103);  // Replace with the IP of your computer running TouchDesigner
const unsigned int outPort = 8005;        // Port on which TouchDesigner is listening
const unsigned int localPort = 8888;      // local port to listen for UDP packets

// Pins
const int ldrPin = A1;  // LDR connected to pin 34
const int ledPin = D2;  // LED connected to pin 35

// Variables for smoothing
float alpha = 0.1;
float smoothedLdrValue = 0;

void setup() {
  pinMode(ldrPin, INPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);

  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected.");

  Udp.begin(localPort);
}

void loop() {
  int ldrValue = analogRead(ldrPin);
  smoothedLdrValue = alpha * ldrValue + (1 - alpha) * smoothedLdrValue;

  Serial.print("Smoothed LDR Value: ");
  Serial.println((int)smoothedLdrValue);

  // Turn LED on or off based on LDR value
  if (smoothedLdrValue < 800) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  // Create OSC message
  OSCMessage msg("/ldrValue");
  msg.add((int)smoothedLdrValue);

  // Send OSC message
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();  // Clear the message

  delay(100);
}