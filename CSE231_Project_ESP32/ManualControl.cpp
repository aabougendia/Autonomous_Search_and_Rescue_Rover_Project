#include "ManualControl.h"

WebServer server(80);
ManualState man_state = DRV_STOP;
ControlState control_state = STATE_MANUAL;

void handlePress();
void handleRelease();
void handleServo();
void handleMode();

void Wifi_Init(){
  // Serial.begin(115200);
  // delay(1000); // Give serial time to initialize
  
  // Initialize LEDs
  // pinMode(ledLeft, OUTPUT);
  // pinMode(ledRight, OUTPUT);
  // pinMode(ledCenter, OUTPUT);
  
  // Start with all LEDs off except center
  // digitalWrite(ledLeft, LOW);
  // digitalWrite(ledRight, LOW);
  // digitalWrite(ledCenter, HIGH); // Center LED on at startup

  // Start WiFi AP with more reliable settings
  // Serial.println("Starting WiFi AP...");
  
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
    // while(1) {
      // digitalWrite(ledCenter, !digitalRead(ledCenter));
      // delay(200);
    // }
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

void Drive_Init(){
  Serial.begin(115200);
  delay(1000); // Wait for serial
  
  // Start WiFi AP
  Serial.println("Starting AP...");
  bool apStarted = WiFi.softAP("ESP32_Control", "12345678", 6);
  
  if (!apStarted) {
    while(1) {
      Serial.println("AP FAILED! Check hardware.");
    }
  }
  
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  // digitalWrite(ledS, HIGH); // Steady on = AP ready

  // Server endpoints
  server.on("/press", HTTP_GET, handlePress);
  server.on("/release", HTTP_GET, handleRelease);
  server.on("/state", HTTP_GET, handleMode);
  server.begin();
  Serial.println("HTTP server started");
}
void Cam_Init();

// void handlePress() {
//   if (control_state == STATE_MANUAL) {
//     String dir = server.arg("dir");

//     if (dir == "F") man_state = DRV_FWD;
//     else if (dir == "B") man_state = DRV_BWD;
//     else if (dir == "L") man_state = DRV_LEFT;
//     else if (dir == "R") man_state = DRV_RIGHT;

//     server.send(200, "text/plain", "ON");
//   } else {
//     control_state = control_state ^ 1;
//     server.send(403, "text/plain", "Manual control disabled in AUTO mode");
//   }
// }
void handleMode() {
  String state = server.arg("state"); 

  if (state == "on") {
    control_state = STATE_MANUAL;
    Serial.println("Switched to MANUAL mode");
    server.send(200, "text/plain", "Mode: MANUAL");
  } 
  else if (state == "off") {
    control_state = STATE_AUTO;
    Serial.println("Switched to AUTO mode");
    server.send(200, "text/plain", "Mode: AUTO");
  } 
  else {
    server.send(400, "text/plain", "Invalid state");
 }
}

void handlePress() {
  if (control_state == STATE_MANUAL) {
    String dir = server.arg("dir");

    if (dir == "F") man_state = DRV_FWD;
    else if (dir == "B") man_state = DRV_BWD;
    else if (dir == "L") man_state = DRV_LEFT;
    else if (dir == "R") man_state = DRV_RIGHT;
    else if (dir == "M");
    else if (dir == "A");
    server.send(200, "text/plain", "Moving: " + dir);
  } else {
    server.send(403, "text/plain", "Manual control disabled in AUTO mode");
  }
}

// void handleRelease() {
//   if (control_state == STATE_MANUAL) {
//     String dir = server.arg("dir");

//     if (dir == "F" || dir == "B" || dir == "L" || dir == "R") {
//       man_state = DRV_STOP;
//       server.send(200, "text/plain", "OFF");
//     } 
//     else if (dir == "M") {

//       control_state = STATE_MANUAL;
//       // digitalWrite(5, HIGH);
//       Serial.println("MANUAL IN HANDLE -------------------------\n-------------------------\n-------------------------\n-------------------------\n");
//     }
//     else if (dir == "A"){
//       control_state = STATE_AUTO;
//       // digitalWrite(5, LOW);
//       }
//     else {
//       server.send(400, "text/plain", "Invalid direction");
//     }
//   } else {
//     server.send(403, "text/plain", "Manual control disabled in AUTO mode");
//   }
// }

void handleRelease() {
  String dir = server.arg("dir");

  if (dir == "M") {
    control_state = STATE_MANUAL;
    digitalWrite(5, HIGH);
    server.send(200, "text/plain", "Switched to MANUAL mode");
  } 
  else if (dir == "A") {
    control_state = STATE_AUTO;
    digitalWrite(5, LOW);
    server.send(200, "text/plain", "Switched to AUTO mode");
  } 
  else if (control_state == STATE_MANUAL) {
    if (dir == "F" || dir == "B" || dir == "L" || dir == "R") {
      man_state = DRV_STOP;
      server.send(200, "text/plain", "OFF");
    } else {
      server.send(400, "text/plain", "Invalid direction");
    }
  } 
  else {
    server.send(403, "text/plain", "Manual control disabled in AUTO mode");
  }
}

void handleServo() {
  String dir = server.arg("dir");      // "left", "right", or "center"
  String state = server.arg("state");  // "on" or "off"

  // Turn off all direction LEDs first
  // digitalWrite(ledLeft, LOW);
  // digitalWrite(ledRight, LOW);
  // digitalWrite(ledCenter, LOW);

  if (state == "on") {
    if (dir == "left") {
      // digitalWrite(ledLeft, HIGH);
      Serial.println("LEFT movement activated");
    } 
    else if (dir == "right") {
      // digitalWrite(ledRight, HIGH);
      Serial.println("RIGHT movement activated");
    }
    else if (dir == "center") {
      // digitalWrite(ledCenter, HIGH);
      Serial.println("CENTER position activated");
    }
  }
  else {

  }
  
  server.send(200, "text/plain", "LEDs: " + dir + "=" + state);
}