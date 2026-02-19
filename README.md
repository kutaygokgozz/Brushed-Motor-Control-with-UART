# 4-Channel STM32 Brushed DC Motor Driver

## Description
This project is a high-performance, custom-designed motor driver board tailored for autonomous systems, mobile robots, and UAV ground control units. Built around the **STM32G431RBT6** mixed-signal microcontroller, this board provides independent control for up to 4 brushed DC motors using dual **TB6612FNG** drivers.

Unlike standard drivers, this system features a **"Smart Voltage Limiting"** algorithm, allowing 5V motors to be safely driven from a 12V power source by dynamically clamping the PWM duty cycle via software. The design also integrates a high-efficiency **TPS562201** synchronous buck converter to ensure stable logic power under heavy loads.

## Key Features
* **Microcontroller:** STM32G431RBT6 (ARM Cortex-M4, 170 MHz) optimized for advanced motor control applications.
* **Motor Driving:** 4x Independent Channels using TB6612FNG (1.2A continuous / 3.2A peak per channel).
* **Power Management:** On-board TPS562201 Synchronous Buck Converter (12V input -> 3.3V logic) with >90% efficiency.
* **Smart Control Algorithms:**
    * *Soft-Start/Ramp Control:* Prevents current spikes and mechanical stress during acceleration.
    * *Voltage Adaptation:* Configurable setup mode to limit voltage for 5V or 12V motors without changing hardware.
* **Connectivity:** UART and I2C interfaces with dedicated headers for ESP-01 Wi-Fi or HM-10 Bluetooth integration.
* **Robust PCB Design:** 2-layer layout with optimized return paths, via stitching, and thermal relief for signal integrity and heat dissipation.

## Tech Stack
* **Hardware Design:** Altium Designer (Schematic & PCB)
* **Firmware:** C / STM32CubeIDE (HAL Library)
* **Simulation/Analysis:** MATLAB, LTspice
