#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);

// LED Pins
const int ledLeft = 23;    // D23 - Left indicator
const int ledRight = 21;   // D21 - Right indicator
const int ledCenter = 4;   // D4 - Center indicator

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  // Initialize LEDs
  pinMode(ledLeft, OUTPUT);
  pinMode(ledRight, OUTPUT);
  pinMode(ledCenter, OUTPUT);
  
  // Start with all LEDs off except center
  digitalWrite(ledLeft, LOW);
  digitalWrite(ledRight, LOW);
  digitalWrite(ledCenter, HIGH); // Center LED on at startup

  // Start WiFi AP with more reliable settings
  Serial.println("Starting WiFi AP...");
  
  // Configure WiFi AP settings
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  
  // Start AP with explicit parameters:
  // SSID: "ESP32_LED_Test"
  // Password: "12345678"
  // Channel: 6 (less crowded)
  // Hidden: false
  // Max connections: 4
  bool apStarted = WiFi.softAP("ESP32_LED_Test", "12345678", 6, false, 4);
  
  if (!apStarted) {
    Serial.println("Failed to start AP!");
    // Blink center LED rapidly to indicate failure
    while(1) {
      digitalWrite(ledCenter, !digitalRead(ledCenter));
      delay(200);
    }
  }

  Serial.println("WiFi AP Started");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("AP MAC address: ");
  Serial.println(WiFi.softAPmacAddress());

  // Server endpoints
  server.on("/servo", handleServo);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

