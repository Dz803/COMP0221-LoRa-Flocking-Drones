# ESP32 Secure LoRa Flocking Drones (COMP0221)

This directory contains the ESP32 firmware for **COMP0221 Coursework 1**, implementing a **secure real-time neighbour exchange and flocking control system** using **FreeRTOS** and **LoRa (SX1276)**.

Each ESP32 node emulates an autonomous drone operating in a bounded 3D space. Nodes periodically broadcast authenticated state information over LoRa and update their motion using a distributed flocking controller derived exclusively from validated neighbour states.

---

## Features

- **FreeRTOS real-time architecture**
  - Physics integration at 50 Hz
  - Flocking control loop at 10 Hz
  - Periodic LoRa transmission with bounded jitter
  - Continuous LoRa reception with strict validation

- **LoRa communication (SX1276)**
  - Register-level SPI control
  - Configured for low airtime (BW = 500 kHz, SF7)
  - Hardware CRC, RSSI, and SNR measurement

- **Security**
  - AES-128 CMAC authentication (4-byte truncated tag)
  - Timestamp-based replay protection
  - Self-packet and team filtering

- **Distributed flocking**
  - Cohesion, alignment, and separation rules
  - Explicit speed limits and collision avoidance
  - Boundary reflection in a bounded 3D space

- **Telemetry**
  - MQTT output for external monitoring and evaluation
  - Telemetry isolated from control and LoRa timing

---

## Project Structure

