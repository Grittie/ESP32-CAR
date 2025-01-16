# ESP32-CAR

This repository contains all the files and documentation for the ESP32 car assignment. The project involves designing, building, and programming an autonomous or remote-controlled car powered by an ESP32 microcontroller.

## Project Structure

```plaintext
.
├── LaboratoryProject/       # PlatformIO project containing the ESP32 firmware
│   ├── src/                 # Source code for the project
│   ├── include/             # Header files
│   ├── lib/                 # Libraries used in the project
│   ├── test/                # Unit tests for the project
│   └── platformio.ini       # PlatformIO configuration file
├── Latex Documentation/     # LaTeX files for the project report
│   ├── main.tex             # Main LaTeX file
│   └── figures/             # Figures and diagrams used in the documentation
├── docs/                    # Project documentation in Markdown format
│   ├── index.md             # Main documentation file
│   ├── assets/              # Media assets for documentation
│   │   ├── schematics/      # Circuit diagrams and schematics
│   │   ├── design/          # CAD files and design-related visuals
│   │   └── images/          # General images and screenshots
├── .gitignore               # Git ignore file
└── README.md                # This file
```

## Documentation

- **[Project Overview](docs/index.md)**: An overview of the ESP32 car project, including its goals, hardware components, and system architecture.
- **[LaTeX Report](Latex%20Documentation/main.tex)**: The detailed LaTeX report for the project. This file includes a technical write-up, design considerations, and any experimental results.
- **[Schematics and Circuit Design](docs/assets/schematics/)**: Links to circuit diagrams and schematics used in the project.
- **[Design Files](docs/assets/design/)**: CAD files and design-related visuals, including the mechanical design for the car.
- **[Images and Screenshots](docs/assets/images/)**: General images, including screenshots of the car in action and setup.

## Getting Started

### Requirements
- iteems on BOM
- PlatformIO Extension of VSC

## Bill of Materials (BOM)

The following components are required for building the ESP32-based car:

### 1. **Light-Dependent Resistor (LDR)**
   - **Description**: Used for detecting light levels, typically for key detection or sensing ambient light.
   - **Example Part**: **LDR-100**, Photoresistor

### 2. **DIP Switch (DIP)**
   - **Description**: A set of manual switches used for selecting one of several pre-defined options (e.g., driving mode, settings).
   - **Example Part**: **Mini 4-Pin DIP Switch**

### 3. **DC Motor (MOT)**
   - **Description**: A motor used for driving the wheels or other moving parts of the car.
   - **Example Part**: **DC 12V Motor**

### 4. **L293D Motor Driver (DRV)**
   - **Description**: A motor driver IC used to control the DC motor, providing direction and speed control.
   - **Example Part**: **L293D Motor Driver IC**

### 5. **Potentiometer (POT)**
   - **Description**: A variable resistor used for adjusting motor speed or other parameters.
   - **Example Part**: **10kΩ Potentiometer**

### 6. **I2C SSD1306 OLED Display (DSP)**
   - **Description**: A small I2C OLED display used to show the status of the car (e.g., car mode, speed).
   - **Example Part**: **SSD1306 128x64 OLED Display**

### 7. **Hall Effect Sensor (HAL)**
   - **Description**: A sensor used to detect the presence of a magnetic field, often used for counting wheel rotations (RPM).
   - **Example Part**: **A3144 Hall Effect Sensor**

### 8. **Cube Magnet (MGN)**
   - **Description**: A small magnet used with the Hall Effect sensor to detect wheel rotations.
   - **Example Part**: **N52 Cube Magnet**

### 9. **Buzzer (BZR)**
   - **Description**: A small electronic buzzer used for alerting the user (e.g., horn press).
   - **Example Part**: **5V Piezo Buzzer**

### 10. **Auxiliary LEDs (LED)**
   - **Description**: LEDs used for indicating various statuses of the car (e.g., drive, park).
   - **Example Part**: **5mm RGB LED**

### 11. **Auxiliary Push Buttons (BTN)**
   - **Description**: Push buttons for user input, such as activating the brake or horn.
   - **Example Part**: **Momentary Push Button**

### 12. **ESP32-S3**
   - **Description**: The main microcontroller used to control the car, handle input, and drive outputs.
   - **Example Part**: **ESP32-S3-WROOM-1**

### 13. **Breadboards**
   - **Description**: A tool for prototyping circuits without soldering.
   - **Example Part**: **Half-size Breadboard**

### 14. **Wires**
   - **Description**: Connecting wires for making connections between components on the breadboard.
   - **Example Part**: **Jumper Wires (Male-to-Male)**

### Installation

1. Clone this repository:
    ```bash
    git clone https://github.com/yourusername/esp32-car.git
    ```
2. Navigate to the `LaboratoryProject/` directory and open it in PlatformIO.
3. Build and upload the firmware to the ESP32.
4. For the LaTeX report, compile the `main.tex` file using your preferred LaTeX editor.

## License

This project is licensed under the MIT License.