# Solar Tracker Project

An active solar tracking system designed to optimize photovoltaic energy collection by continuously aligning solar panels with the most intense light source. This repository contains the complete project files, including custom PCB schematics, C++ firmware, and 3D-printable mechanical assemblies. 

The system utilizes closed-loop control, reading environmental light gradients through an array of sensors and actuating motors to maintain optimal panel orientation.

## Project Architecture

The project is divided into three main engineering domains: electrical hardware, mechanical design, and embedded firmware. 

### 1. Hardware and Electronics
The circuit design and PCB layouts were developed using KiCad. The hardware system is responsible for signal acquisition and motor power distribution.

*   **Microcontroller:** Arduino-compatible MCU for processing ADC readings and generating PWM control signals.
*   **Sensors:** Light Dependent Resistors (LDRs) arranged in a directional array with voltage divider networks to measure horizontal and vertical light differentials.
*   **Actuators:** Servos (or stepper motors) to control the pan (azimuth) and tilt (elevation) axes.
*   **Power Management:** Isolated power routing to prevent motor current spikes from inducing noise or brownouts on the logic circuit.
*   **Design Files:** Schematics, PCB layouts, and Gerber files are located in the `hardware/` directory.

### 2. Embedded Firmware
The control logic is written entirely in C++ and focuses on efficiency and preventing mechanical jitter. 

*   **Platform:** C++ (compiled via Arduino IDE or PlatformIO).
*   **Control Loop:** The firmware continuously polls the analog pins connected to the LDRs, calculates the light differential across the axes, and updates motor positions accordingly.
*   **Hysteresis Implementation:** A software-defined deadband is included to filter out minor lighting fluctuations (like passing clouds or sensor noise). This prevents continuous, unnecessary motor twitching, which saves power and reduces mechanical wear.
*   **Source Files:** All code, including headers and main operational logic, can be found in the `firmware/` directory.

### 3. Mechanical Design
The physical chassis is custom-designed for structural rigidity and minimal weight, ensuring the motors do not experience excessive torque loads. 

*   **Fabrication:** Designed for Fused Deposition Modeling (FDM) 3D printing. PLA or PETG is recommended for durability.
*   **Components:** Includes the base mount, azimuth rotation brackets, elevation tilt brackets, and the solar panel mounting plate. A cross-shaped LDR shadow shield is also included to cast distinct shadows over the sensors when the panel is not perfectly aligned.
*   **CAD Files:** STL files and step files are provided in the `cad/` directory.

## Repository Structure

├── cad/                  # 3D printable models (.STL files for chassis and mounts)
├── hardware/             # KiCad schematic files, PCB layouts, and gerbers
├── firmware/             # C++ source code for the microcontroller
├── docs/                 # Wiring diagrams and supplemental documentation
└── README.md             # Project overview

## Setup and Assembly

### Mechanical Assembly
1. Print all components located in the `cad/` directory with a minimum of 20% infill for structural integrity.
2. Assemble the base and mount the azimuth motor.
3. Attach the elevation bracket and corresponding motor.
4. Mount the LDR array and attach the cross-shield to ensure accurate differential shading.

### Electronics Wiring
1. Open the KiCad schematics in the `hardware/` folder to review the pinout.
2. Wire the LDR voltage dividers to the analog input pins of your MCU.
3. Connect the motor control lines to the designated PWM output pins.
4. Ensure the motors are powered by an adequate external power supply with a common ground shared with the MCU.

### Firmware Flashing
1. Open the `firmware/` directory in your preferred IDE.
2. Verify that the assigned pin definitions in the header files match your physical wiring.
3. Adjust the threshold and delay variables if your specific ambient lighting environment requires more or less sensitivity.
4. Compile and upload the code to the microcontroller.

## Future Improvements

*   Implementation of real-time clock (RTC) modules for predictive solar tracking based on time of day and geographical coordinates.
*   Integration of current and voltage sensors to log actual power generation data.
*   Migrating from servo motors to precision stepper motors for larger panel handling.

## License

This project is open-source. Feel free to fork the repository, submit pull requests, or adapt the designs for your own applications.
