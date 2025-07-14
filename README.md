# HelmetGuard: Smart Helmet Safety System

HelmetGuard is an IoT-based smart helmet system designed to enhance rider safety using real-time monitoring, accident detection, and emergency alert features. Built on the ESP32 microcontroller, the system integrates multiple sensors to provide a compact and intelligent solution for two-wheeler riders.

## Features

- Alcohol detection using MQ3 sensor
- Fall detection using MPU6050 (accelerometer + gyroscope)
- Heart rate and SpO₂ monitoring via MAX30105 sensor
- Helmet wear detection using IR sensor
- Emergency alert system with SMS and live GPS location via SIM800L and TinyGPS++
- Wi-Fi connectivity for data transmission

## Hardware Components

| Component     | Purpose                        |
|---------------|--------------------------------|
| ESP32         | Main microcontroller            |
| MAX30105      | Heart rate and SpO₂ sensor     |
| MPU6050       | Fall detection                  |
| MQ3           | Alcohol detection               |
| IR Sensor     | Helmet wear detection           |
| SIM800L       | GSM module for SMS alerts       |
| TinyGPS++     | GPS parsing for live location   |
| Buzzer        | Audio alert system              |

## How It Works

1. Rider wears the helmet; IR sensor confirms usage.
2. Heart rate and oxygen saturation are monitored continuously.
3. Alcohol levels are checked through the MQ3 sensor.
4. MPU6050 detects abnormal movements (falls or collisions).
5. If a dangerous condition is detected (e.g., fall, low vitals, alcohol presence), the system:
   - Triggers a buzzer
   - Sends an SMS alert to emergency contacts with the rider’s vitals and live GPS location

## Getting Started

1. Flash the `HelmetGuard.ino` file to your ESP32 board using the Arduino IDE.
2. Connect the required hardware components as per the circuit diagram.
3. Power the system and ensure sensors are initialized properly.
4. Monitor output via serial monitor or connected interface.



