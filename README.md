# linefollower-25

**ESP32 SMART PID LINE FOLLOWER + WIFI TUNING**

The **Line Follower Robot** is a PID-controlled robot that tracks a line on the floor using 5 IR sensors, with Wi-Fi-based tuning for dynamic adjustments.

## Features

1. **Persistent Storage**: Saves PID parameters, speed, delay, and error memory to flash.
2. **Wi-Fi Tuning**: Adjust PID values and other parameters via a browser-based interface.
3. **Live Feedback**: Real-time sensor readings, motor outputs, and error values.
4. **Search Logic**: Smart recovery when the line is lost:
   - Coasts straight on gaps.
   - Spins on corners.
5. **Peak Hold**: Maintains strong error values briefly to avoid noise on sharp turns.

## Web Interface

- **Sliders**: Tune `Kp`, `Kd`, speed, loop delay, and error memory.
- **Buttons**: Start/stop, toggle black/white line modes, save settings.
- **Live Data**: Displays sensor states, errors, and motor speeds.

## How It Works

1. **PID Control**: Adjusts motor speeds based on sensor errors for precise line following.
2. **Persistent Configurations**: Loads and saves parameters for consistent performance across sessions.
3. **Error Handling**: Implements robust strategies for line loss, reducing oscillations and improving stability.

## Component List

Microcontroller: ESP32 Development Board (NodeMCU-32S / DOIT DevKit V1).
Motors: 2x N20 Micro Metal Gear Motors (~600RPM).
Motor Driver: TB6612FNG Dual Motor Driver.
Sensors: 5-Channel IR Line Tracking Module (TCRT5000).
Power Supply: 3x 18650 Li-ion Batteries (Series connection ~11.1V - 12.6V).
Voltage Regulator: MP1584EN Buck Converter (Stepped down to 5V for ESP32 & IR Sensor).
Chassis: Custom 3D printed or acrylic base with caster wheel.

## Installation and Usage

1. Flash the ESP32 with the provided code.
2. Connect to the Wi-Fi AP (`LineFollower_Setup`, password: `12345678`).
3. Use the web interface to tune parameters.
4. Place the robot on the track and press "START".

## Connections

Motor Driver (TB6612FNG) Connections:

   PWMA to ESP32 Pin 25  
   AIN1 to ESP32 Pin 26  
   AIN2 to ESP32 Pin 27  
   PWMB to ESP32 Pin 14  
   BIN1 to ESP32 Pin 12  
   BIN2 to ESP32 Pin 13  
   STBY to ESP32 Pin 4  

IR Sensor Array Connections:

   (Far Left) to ESP32 Pin 36  
   (Left) to ESP32 Pin 39  
   (Middle) to ESP32 Pin 34  
   (Right) to ESP32 Pin 35  
   (Far Right) to ESP32 Pin 32


---

Developed for the **City St. George's 2025 End of the Line Competition**.
