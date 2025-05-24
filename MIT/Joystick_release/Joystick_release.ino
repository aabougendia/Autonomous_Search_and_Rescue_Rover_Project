#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);

// LED Pins
const int ledF = 23;
const int ledB = 22;
const int ledL = 21;
const int ledR = 19;
const int ledS = 18;

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial
  
  // Initialize LEDs
  pinMode(ledF, OUTPUT);
  pinMode(ledB, OUTPUT);
  pinMode(ledL, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledS, OUTPUT);
  
  // Test LEDs
  digitalWrite(ledS, HIGH);
  digitalWrite(ledF, HIGH); delay(200); digitalWrite(ledF, LOW);
  digitalWrite(ledB, HIGH); delay(200); digitalWrite(ledB, LOW);
  digitalWrite(ledL, HIGH); delay(200); digitalWrite(ledL, LOW);
  digitalWrite(ledR, HIGH); delay(200); digitalWrite(ledR, LOW);

  // Start WiFi AP
  Serial.println("Starting AP...");
  bool apStarted = WiFi.softAP("ESP32_Control", "12345678", 6);
  
  if (!apStarted) {
    Serial.println("AP FAILED! Check hardware.");
    while(1) {
      digitalWrite(ledS, HIGH); delay(100);
      digitalWrite(ledS, LOW); delay(100);
    }
  }
  
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  digitalWrite(ledS, HIGH); // Steady on = AP ready

  // Server endpoints
  server.on("/press", HTTP_GET, handlePress);
  server.on("/release", HTTP_GET, handleRelease);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

void handlePress() {
  String dir = server.arg("dir");
  if (dir == "F") digitalWrite(ledF, HIGH);
  else if (dir == "B") digitalWrite(ledB, HIGH);
  else if (dir == "L") digitalWrite(ledL, HIGH);
  else if (dir == "R") digitalWrite(ledR, HIGH);
  server.send(200, "text/plain", "ON");
}

void handleRelease() {
  String dir = server.arg("dir");
  if (dir == "F") digitalWrite(ledF, LOW);
  else if (dir == "B") digitalWrite(ledB, LOW);
  else if (dir == "L") digitalWrite(ledL, LOW);
  else if (dir == "R") digitalWrite(ledR, LOW);
  server.send(200, "text/plain", "OFF");
}