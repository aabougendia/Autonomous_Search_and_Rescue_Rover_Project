# Autonomous Search and Rescue Rover Project

## Overview

This project is a fully autonomous search and rescue system built on a two-microcontroller architecture. It is designed to navigate disaster-struck areas, detect humans using thermal imaging, determine motion using PIR sensors, and report precise GPS coordinates via GSM-based SMS alerts. The system supports both autonomous and manual control via a Wi-Fi interface and includes onboard sensing, communication, and navigation capabilities. The project integrates an STM32 microcontroller for control and motion and an ESP32 for sensor fusion, communication, and high-level decision-making.

## Features

- **Autonomous Human Detection**:
  - Uses the AMG8833 thermal camera on the ESP32 to detect heat signatures.
  - The STM32 initiates detection cycles and changes rover behavior based on detection feedback.

- **Motion Confirmation**:
  - A PIR sensor confirms motion to assess the victim’s consciousness level.

- **Obstacle Avoidance**:
  - The STM32 uses an ultrasonic sensor and MPU6050 gyroscope to detect and avoid nearby objects during patrol.

- **GPS Location Sharing**:
  - The Neo-6M GPS module get the Rover location and sends to the STM32 in an NMEA format.
  - The STM32 reads and decodes the NMEA data to get the GPS coordinates and sends them to the ESP32.

- **GSM Communication**:
  - ESP32 communicates with the SIM800L GSM module to send SMS alerts containing human state and GPS location.

- **Manual Control Mode**:
  - The ESP32 hosts a Wi-Fi access point and HTTP server for remote rover control via a mobile browser.

- **LED and Buzzer Feedback**:
  - An RGB LED and buzzer on the STM32 provide visual and audible feedback for different states (Reconning, Sending Info, Idle).

- **Dual-Microcontroller Communication**:
  - UART-based protocol between STM32 and ESP32, with GPIO-based flagging for safe and synchronized data exchange.

## Components

- **Microcontrollers**:
  - STM32F103C8T6 (“Blue Pill”) – low-level motion and sensing
  - ESP32 DevKit – high-level control and communication

- **Sensors and Modules**:
  - AMG8833 thermal camera (human detection)
  - PIR sensor (motion confirmation)
  - GPS Module (e.g., NEO-6M for location)
  - SIM800L GSM module (SMS transmission)
  - HC-SR04 ultrasonic sensor (obstacle detection)
  - MPU6050 gyroscope (orientation and stability)

- **Motors**:
  - Stepper motors (smooth, precise movement)
  - A4988 drivers (stepper control)

- **Power Supply**:
  - 3x 18650 lithium-ion cells (12.5V total)
  - Buck converters (for 5V and 4V regulated outputs)
  - AMS1117 LDO (3.3V for thermal and IMU)

- **User Interface and Feedback**:
  - RGB LED and buzzer (state indication)
  - Wi-Fi AP (manual mode control from phone browser)

- **Interfacing**:
  - UART protocol for GPS coordinates
  - GPIO pins for mode/state synchronization and Thermal detection flagging

## How It Works
1. The Rover starts in **manual mode** for the operator to drive it to the search location, the control is wireless and Wi-Fi based.
2. The operator enables the **autonomous mode** via the wireless controller.
3. The STM32 scans for obstacles and polls the ESP32 for thermal detection.
4. If a human is detected, PIR confirms motion status.
5. GPS data is sent from the STM32 to the ESP32.
6. ESP32 compiles human status and location into an SMS and sends it via GSM.
7. The rover enters idle mode until the operator turns on the manual mode

## How to Use

1. Power the system using the power switch on board.
2. Download the .apk app named Rescue_Rover_Controller
3. Connect to the ESP32's Wi-Fi (SSID: `ESP32_Control`, password: `12345678`) for manual mode.
4. Use the web interface to control the Rover and switch between manual and autonomous control.
5. Monitor status via LED and buzzer.

## License

This project is licensed under a proprietary license. The code and design may be viewed but not used, reproduced, or distributed without explicit permission.

## Contributors

- Abdulrahman Abougendia
- Fouad Hashesh
- Salah Elsayed
- Mohamed Farouk
- Amir Ashraf
- Nour Hazem
- Habiba Ahmed
- Sara Aldahshan
- Jana Sherif
- Rokaya Radwan
