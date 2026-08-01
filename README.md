# Wi-Fi Controlled Quadcopter Using ESP32

A low-cost Wi-Fi-controlled quadcopter designed and developed using the **ESP32 microcontroller**. The project combines embedded systems, wireless communication, control systems, and real-time programming to create a compact drone that can be operated through a smartphone over Wi-Fi.

This project was completed as part of the requirements for the **Bachelor of Engineering (B.Eng.) in Telecommunication Engineering** at the **Federal University of Technology, Minna, Nigeria**.

---

## Project Overview

Traditional drone systems often rely on dedicated radio transmitters and expensive flight controllers, making them less accessible for students and hobbyists.

This project demonstrates that a reliable quadcopter can be developed using affordable, off-the-shelf components and open-source technologies. The ESP32 serves as the main flight controller, handling sensor acquisition, flight stabilization, motor control, and Wi-Fi communication with a smartphone application.

The system integrates real-time sensor processing, PID-based attitude control, and wireless command transmission to achieve stable flight while maintaining a low overall cost.

---

## Features

* ESP32-based flight controller
* Smartphone control via Wi-Fi
* Real-time PID flight stabilization
* MPU6050 IMU integration
* LiDAR-based altitude measurement
* PWM motor control for four ESCs
* Custom PCB for compact hardware integration
* Safety features including emergency stop and automatic motor disarming
* MATLAB/Simulink simulation for flight dynamics validation

---

## Hardware Components

* ESP32 Dual-Core Microcontroller
* MPU6050 Inertial Measurement Unit (IMU)
* VL53L1X LiDAR Sensor
* Four RS2205 2300KV Brushless Motors
* Four FVT LittleBee 30A Electronic Speed Controllers (ESCs)
* QAV210 Carbon Fiber Frame
* 3S 11.1V Li-Po Battery
* Buck Converter / BEC
* 5045 CW and CCW Propellers
* Custom Designed PCB

---

## Software Stack

* C Programming Language
* ESP-IDF / Arduino Framework
* Wi-Fi (SoftAP Mode)
* UDP Communication
* PWM Motor Control
* I²C Communication
* PID Control Algorithm
* Complementary Filter
* MATLAB/Simulink

---

## System Architecture

The system consists of four major modules:

1. **Flight Controller**

   * ESP32 firmware
   * Sensor acquisition
   * PID controller
   * Motor mixing

2. **Sensor System**

   * MPU6050 for orientation
   * VL53L1X for altitude measurement

3. **Communication Layer**

   * Wi-Fi SoftAP
   * UDP command packets
   * Low-latency communication

4. **Web Application**

   * Dual joystick interface
   * Throttle
   * Roll
   * Pitch
   * Yaw

---

## Flight Control Logic

The drone continuously performs the following sequence:

1. Read IMU sensor data.
2. Apply complementary filtering to estimate orientation.
3. Read altitude data from the LiDAR sensor.
4. Receive control commands from the smartphone.
5. Compute pitch, roll, yaw, and throttle errors.
6. Execute PID control loops.
7. Mix motor outputs.
8. Generate PWM signals for each ESC.
9. Repeat continuously in real time.

---

## Safety Features

* Emergency stop
* Motor arming/disarming sequence
* Sensor validation
* Dead-zone filtering
* Communication timeout detection
* Automatic motor shutdown on Wi-Fi loss

---

## Simulation

MATLAB/Simulink was used to validate the drone's mathematical model before hardware implementation.

The simulation includes:

* Motor thrust distribution
* Roll dynamics
* Pitch dynamics
* Yaw dynamics
* System response analysis
* Stability verification

---

## Repository Structure

```text
firmware/
    Main ESP32 source code

Web App/
    Smartphone control interface
    App Flowchart

simulation/
    MATLAB/Simulink models

Docs/
    Circuit diagrams
    Block diagrams

images/
    Drone photographs
    Drone parts
```

---

## Future Improvements

* GPS navigation
* Autonomous waypoint flight
* Obstacle avoidance
* ESP32-CAM live video streaming
* LoRa communication for extended range
* Battery management system
* Adaptive PID tuning
* Sensor fusion using Kalman filtering

---

## Author
Paulinus Ogbak & Pam Jonn

Bachelor of Engineering (B.Eng.)
Telecommunication Engineering
Federal University of Technology, Minna

---

## License

This repository is provided for educational and research purposes. Feel free to study, reference, and build upon the work with appropriate attribution.
