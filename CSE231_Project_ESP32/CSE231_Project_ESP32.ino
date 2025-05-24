// #include "CommBus.h"
#include <Wire.h>
#include "AMG8833.h"

#include "SystemFlow_ESP32.h"

// CommBus comm(Serial2);






void setup() {
  // // put your setup code here, to run once:
  // Serial2.begin(115200, SERIAL_8N1, 16, 17);
  // Serial.begin(115200);
  // comm.begin(115200);
  
  // pinMode(2, OUTPUT);
  // digitalWrite(2, LOW);

  // Serial.begin(115200);
  // if (!thermalSensor.begin()) {
  //   Serial.println("Sensor not found!");
  //   while (true);
  // }
  // Serial.println("in main setup\n");
  SystemFlow_Init();


}

void loop() {
  // // // put your main code here, to run repeatedly:

  // int it = INT_MAX;
  // while(it--){
  //   comm.sendMessage(THM, "1");
  //   delay(500);
  //   comm.sendMessage(THM, "0");
  //   delay(500);
  // }
  

  // // Example: print received sensor data
  // it = INT_MAX;
  // while(it--){
    // Serial.println("____________________________________");
  // comm.enable();
  // comm.handleIncoming();   // Continuously process incoming messages
    // Serial.println("ULT = " + String(comm.ULT_Distance));
    // Serial.println("GPS = " + comm.GPS_GoogleMapsLink);
    // Serial.println("PIR = " + String(comm.PIR_Decision));
    // delay(500);
  //   if(comm.PIR_Decision == PIR_MOTION_DETECTED)
  //     digitalWrite(2, HIGH);
  //   else
  //     digitalWrite(2, LOW);
  // }

  // Serial.println("____________________________________");

  /****************   thermal test  ****************/
    // thermalSensor.readPixels(temperatureData);
    // thermalSensor.printPixels();  // Optional
    // delay(1000);

  // if (thermalSensor.detectHumanRelative()) {
  //   Serial.println("Human detected (ambient-aware)");
  // } else {
  //   Serial.println("No human detected");
  // }
  // delay(500);






  // Serial.println("in system loop\n");
  SystemFlow_Run();



}


// #include <Adafruit_AMG88xx.h>
// Adafruit_AMG88xx amg;

// float pixels[64];

// void setup() {
//   Serial.begin(115200);
//   if (!amg.begin()) {
//     Serial.println("AMG8833 not detected!");
//     while (1);
//   }
// }

// void loop() {
//   amg.readPixels(pixels);
//   for (int i = 0; i < 64; i++) {
//     Serial.print(pixels[i]);
//     Serial.print(",");
//     if ((i + 1) % 8 == 0) Serial.println();
//   }
//   delay(500);
// }

