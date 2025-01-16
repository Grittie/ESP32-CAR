# Project Overview

## ESP32 Car Project

The ESP32 (toy) car project is a small project written in esp-idf to use conmonly found components to creatively simulate parts and functions of a car.

### Goals

The primary goals of the ESP32 car project are:
- To design and build a car that can be controlled remotely or autonomously.
- To integrate multiple sensors (e.g., light-dependent resistor, Hall effect sensor) to monitor and control the car’s status and operation.
- To develop firmware for the ESP32 that interfaces with various components, such as motors, sensors, and displays, using FreeRTOS for real-time task management.
- To provide a user interface through an I2C OLED display to show the current status of the car (e.g., speed, direction, mode).

### Hardware Components

The car is built using the following components:

1. **ESP32-S3**: The main microcontroller that controls the car's operation.
2. **DC Motor**: Powers the car’s wheels, allowing it to move forward, backward, or stop.
3. **L293D Motor Driver**: Controls the direction and speed of the DC motor.
4. **Hall Effect Sensor**: Used for counting wheel rotations to calculate the car's speed (RPM).
5. **Light-Dependent Resistor (LDR)**: Detects light changes for key detection (e.g., when the car is powered on).
6. **DIP Switches**: Used for selecting modes or settings, such as drive or park.
7. **I2C SSD1306 OLED Display**: Displays car status, mode, speed, and other relevant information.
8. **Potentiometer**: Adjusts motor speed based on user input.
9. **Auxiliary LEDs**: Used for visual indicators (e.g., lights for "Drive", "Park").
10. **Push Buttons**: Control the car’s brake and horn functions.
11. **Buzzer**: Provides audio feedback (horn sound).

### System Architecture

The system consists of several key parts:

1. **Microcontroller (ESP32)**: The heart of the system, it manages all the inputs and outputs, including motor control, button presses, sensor readings, and display updates.
   - Runs FreeRTOS for task management, ensuring that different components of the system operate concurrently (e.g., motor control, sensor reading, button presses).
   - Communicates with peripherals like the OLED display via I2C and handles PWM control for motor speed.
   
2. **Motor Control**: The L293D motor driver is used to control the DC motor. The ESP32 generates PWM signals to adjust the motor’s speed, and GPIO pins control the motor’s direction (forward, reverse, or stop).

3. **Sensors and Inputs**:
   - **Hall Effect Sensor** detects wheel rotations, providing the necessary feedback for calculating the car's speed (RPM).
   - **Light-Dependent Resistor (LDR)** monitors ambient light and detects key press or presence.
   - **Push Buttons** (Brake and Horn) are used to control the car’s behavior, with interrupts triggering specific actions.

4. **User Interface**:
   - The **SSD1306 OLED Display** provides feedback to the user, displaying the current mode, speed, and other car statuses.
   - **Auxiliary LEDs** provide visual indicators for different car modes, such as “Drive” or “Park”.

5. **Power Management**: The ESP32 and all components are powered through a common power supply, with the motor and other components powered accordingly to their requirements.

---
