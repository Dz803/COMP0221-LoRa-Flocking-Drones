COMP0221 – Secure Real-Time LoRa-Based Flocking Drones on ESP32

This repository contains the implementation and evaluation code for COMP0221 Coursework 1 at University College London.
The project demonstrates a secure, real-time neighbour exchange and flocking control system implemented on the ESP32 platform, using LoRa, FreeRTOS, and lightweight cryptographic authentication.

Each ESP32 node emulates an autonomous drone operating in a bounded 3D space and exchanges authenticated state information with nearby peers. A distributed flocking controller computes motion updates based on validated neighbour states only.

.
├── CW1_Drones_Flock/
│   ├── main/
│   │   └── main.c                # ESP32 application (FreeRTOS + LoRa + flocking)
│   ├── CMakeLists.txt
│   ├── sdkconfig
│   └── README.md                 # ESP32 build & flash instructions
│
├── MQTT_Visualizer/
│   └── COMP0221-MQTT-Visualiser/
│       ├── visualiser.py         # Real-time MQTT visualiser
│       ├── plot.py               # Performance plotting (RX interval, RSSI, jitter)
│       ├── mac_dict.py           # Node ID / MAC mapping
│       ├── packet_structure.json # Packet format definition
│       ├── requirements.txt
│       └── README.md             # Visualiser usage instructions
│
└── README.md                     # (This file)
