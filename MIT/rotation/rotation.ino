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

void handleServo() {
  String dir = server.arg("dir");      // "left", "right", or "center"
  String state = server.arg("state");  // "on" or "off"

  // Turn off all direction LEDs first
  digitalWrite(ledLeft, LOW);
  digitalWrite(ledRight, LOW);
  digitalWrite(ledCenter, LOW);

  if (state == "on") {
    if (dir == "left") {
      digitalWrite(ledLeft, HIGH);
      Serial.println("LEFT movement activated");
    } 
    else if (dir == "right") {
      digitalWrite(ledRight, HIGH);
      Serial.println("RIGHT movement activated");
    }
    else if (dir == "center") {
      digitalWrite(ledCenter, HIGH);
      Serial.println("CENTER position activated");
    }
  }
  
  server.send(200, "text/plain", "LEDs: " + dir + "=" + state);
}


//void handleServoo() {
 // String dir = server.arg("dir");      // "left", "right", or "center"
  //String state = server.arg("state");  // "on" or "off"

  //if (state == "on") {
  //  if (dir == "left") {
   //   panServo.write(0);    // Full left
   // } 
    //else if (dir == "right") {
    //  panServo.write(180);  // Full right
    //}
    //else if (dir == "center") {
    //  panServo.write(90);   // Center position
   // }
 // } 
  // state == "off" -> No action (servo stays at current position)
  
 // server.send(200, "text/plain", "Direction:" + dir + " State:" + state);
//}