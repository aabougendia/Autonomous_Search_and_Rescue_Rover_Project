// #include CommBus.h




// CommBusCommBus(HardwareSerial& serialPort)  serial(serialPort) {}

// void CommBusbegin(unsigned long baud, int rx, int tx) {
//     lastBaud = baud;
//     rxPin = rx;
//     txPin = tx;
//     serial.begin(baud, SERIAL_8N1, rx, tx);
// }


// void CommBushandleIncoming() {
//     while(serial.available()){
//         char byte = serial.read();

//         Serial.print(byte);   Echo what was received to debug monitor

//         processByte(byte);
//     }
// }

// void CommBushandleIncoming() {
//     if (!enabled) return;

//     while (serial.available()) {
//         char byte = serial.read();
//         processByte(byte);
//     }
// }


// void CommBusprocessByte(char byte){
//     if(messageReady) return;

//     if (byte == 'n') {
//         Trim trailing r if present
//             if (rxIndex  0 && rxBuffer[rxIndex - 1] == 'r') {
//             rxBuffer[rxIndex - 1] = '0';
//         } else {
//             rxBuffer[rxIndex] = '0';
//         }

//         String message = String(rxBuffer);

//         dispatchMessage(message);
//         rxIndex = 0;
//         messageReady = true;
//     } else {
//         if (rxIndex  RX_BUF_SIZE - 1) {
//             rxBuffer[rxIndex++] = byte;
//         } else {
//             rxIndex = 0;   overflow safety
//         }
//     }
// }

// CommBus_MessageType CommBusgetMessageType(const String& msg) {
//     if (msg.startsWith(GPS)) return GPS;
//     if (msg.startsWith(ULT)) return ULT;
//     if (msg.startsWith(PIR)) return PIR;
//     if (msg.startsWith(THM)) return THM;
//     if (msg.startsWith(DRV)) return DRV;
//     if (msg.startsWith(SRV)) return SRV;

//     return UNKNOWN;
// }

// String CommBusgetPayload(const String& msg) {
//     int index = msg.indexOf('');
//     return (index != -1)  msg.substring(index + 1)  ;
// }

// void CommBusdispatchMessage(const String& msg) {

//     Serial.print([RAW MSG] );
//     Serial.println(msg);


//     CommBus_MessageType type = getMessageType(msg);
//     String payload = getPayload(msg);

//     switch (type) {
//         case GPS
//             Serial.println([GPS]  + payload);
//             this-GPS_GoogleMapsLink = payload;
//             break;
//         case ULT
//             Serial.println([ULT]  + payload +  cm);
//             this-ULT_Distance = payload.toInt();
//             break;
//         case PIR
//             Serial.println([PIR] Motion  + payload);
//             this-PIR_Decision = payload == 1  PIR_MOTION_DETECTED  PIR_MOTION_NOT_DETECTED;
//             break;
//         default
//             Serial.println([WARN] Unknown message type);
//             break;
//     }

//     messageReady = false;
// }

// bool CommBusisMessageReady() const {
//     return messageReady;
// }

// void CommBusresetGPS(){
//     GPS_GoogleMapsLink = ;
// }
// void CommBusresetULT(){
//     ULT_Distance = 0;
// }
// void CommBusresetPIR(){
//     PIR_Decision = PIR_MOTION_NOT_DETECTED;
// }
// void CommBusresetTHM(){

// }


// void CommBussendMessage(CommBus_MessageType type, const String& payload) {
//     String msgType;
//     switch (type) {
//         case THM msgType = THM; break;
//         case DRV msgType = DRV; break;
//         case SRV msgType = SRV; break;
//         default  msgType = UNKNOWN; break;
//     }
//     serial.print(msgType +  + payload + n);
// }


// void CommBusenable() {
//     if (enabled) return;
//     enabled = true;
//     serial.begin(lastBaud, SERIAL_8N1, rxPin, txPin);
// }

// void CommBusdisable() {
//     enabled = false;
//     serial.end();

//     Optional flush lingering data
//     while (serial.available()) {
//         serial.read();   drain any leftover bytes
//     }
// }