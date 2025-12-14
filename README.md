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

## Installation and Usage

1. Flash the ESP32 with the provided code.
2. Connect to the Wi-Fi AP (`LineFollower_Setup`, password: `12345678`).
3. Use the web interface to tune parameters.
4. Place the robot on the track and press "START".

Made for dynamic performance and easy parameter tuning, this robot is optimized for adaptability and competition success.

---

Developed for the **City St. George's 2025 End of the Line Competition**.
