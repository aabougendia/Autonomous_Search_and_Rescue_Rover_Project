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
  - The STM32 reads GPS coordinates and sends them to the ESP32.
  - ESP32 formats the data as a Google Maps link.

- **GSM Communication**:
  - ESP32 communicates with the SIM800L GSM module to send SMS alerts containing human state and GPS location.

- **Manual Control Mode**:
  - The ESP32 hosts a Wi-Fi access point and HTTP server for remote rover control via a mobile browser.

- **LED and Buzzer Feedback**:
  - An RGB LED and buzzer on the STM32 provide visual and audible feedback for different states (scanning, sending, idle).

- **Dual-Microcontroller Communication**:
  - UART-based protocol between STM32 and ESP32, with GPIO-based acknowledgment for safe and synchronized data exchange.

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
  - 4x 18650 lithium-ion cells (14.8V total)
  - Buck converters (for 5V and 3.3V regulated outputs)
  - AMS1117 LDO (3.3V for thermal and IMU)

- **User Interface and Feedback**:
  - RGB LED and buzzer (state indication)
  - Wi-Fi AP (manual mode control from phone browser)

- **Interfacing**:
  - UART protocol for GPS/thermal/PIR data
  - GPIO pins for mode/state synchronization

## How It Works

1. In **autonomous mode**, the STM32 scans for obstacles and polls the ESP32 for thermal detection.
2. If a human is detected, PIR confirms motion status.
3. GPS data is sent from the STM32 to the ESP32.
4. ESP32 compiles human status and location into an SMS and sends it via GSM.
5. The rover resumes patrol or enters idle mode.
6. In **manual mode**, the user connects via Wi-Fi to control the rover through directional commands.

## How to Use

1. Power the system using a 14.8V battery pack.
2. The rover begins scanning autonomously.
3. Connect to the ESP32's Wi-Fi (SSID: `ESP32_Control`, password: `12345678`) for manual mode.
4. Use the web interface to switch between manual and autonomous control.
5. Monitor status via LEDs and console outputs.

## License

This project is licensed under a proprietary license. The code and design may be viewed but not used, reproduced, or distributed without explicit permission.

## Contributors

- Abdulrahman Abougendia
- Jana Sherif
- Rokaya Radwan