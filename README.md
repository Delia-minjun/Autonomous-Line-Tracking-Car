<h1 align="center"> Intelligent Line-Tracking & Distance-Measuring Vehicle</h1>

<p align="center">
  <img src="https://img.shields.io/badge/Hardware-Arduino_MEGA_2560-00979D?style=flat-square&logo=arduino&logoColor=white" alt="MCU" />
  <img src="https://img.shields.io/badge/Algorithm-Incremental_PI-blue?style=flat-square" alt="Algorithm" />
  <img src="https://img.shields.io/badge/Sensor-GMR_Encoder_%7C_HC--SR04-brightgreen?style=flat-square" alt="Sensors" />
  <img src="https://img.shields.io/badge/Filter-Sliding_Window_Average-orange?style=flat-square" alt="Filter" />
</p>

> **Project for "Intelligent Sensing and Signal Processing"**
> 
> Designed an autonomous intelligent patrol vehicle capable of promptly correcting path deviations, handling complex intersections, and measuring obstacle distances with high accuracy. 

---

## Overview

This project implements a closed-loop control smart car based on the **Arduino MEGA 2560**. By integrating a massive 13-channel infrared sensor array and GMR (Giant Magnetoresistance) motor encoders, the vehicle achieves smooth and rapid line-tracking on complex courses (including identifying branching intersections). Additionally, an ultrasonic sensor combined with a customized sliding-window filtering algorithm ensures stable distance measurement.

The entire process, from hardware selection and circuit construction to bottom-level driver development and algorithm design, was completed independently.

## 🛠️ Hardware Architecture

- **Core MCU:** Arduino MEGA 2560 (Selected for its abundant I/O and interrupts).
- **Line Tracking Array:** 1x 5-channel IR sensor (front) + 2x 4-channel IR sensors (sides). Total 13 channels for wide-range and complex intersection detection.
- **Actuators & Feedback:** DC Motors equipped with **GMR Encoders** (yielding high-precision A/B quadrature pulses).
- **Distance Sensor:** HC-SR04 Ultrasonic Sensor.
- **Display:** 0.96" OLED (SSD1306) for real-time data monitoring via I2C.

## Core Algorithms & Software Design

### 1. Incremental PI Motor Control (Closed-Loop)
To solve the path deviation problem on complex tracks, a closed-loop speed control system was built:
- **Encoder Feedback:** External interrupts capture the rising/falling edges of the GMR encoder's A/B phases to accurately calculate motor speed and direction.
- **Timer Interrupts:** A 10ms timer (`MsTimer2`) is used to guarantee a strict calculation cycle.
- **PI Controller:** Designed an Incremental PI algorithm to dynamically adjust the PWM duty cycle for both left and right motors, minimizing speed errors and ensuring smooth turns.

### 2. Complex Tracking Logic
Unlike simple 3-way sensors, the 13-channel IR array allows the car to dynamically switch between normal linear tracking and **intersection mode**. 
Different speed coefficients (e.g., `0.5*Target`, `0.3*Target`) are dynamically assigned to the left/right wheels based on the severity of the deviation, achieving smooth correction.

### 3. Distance Measurement & Signal Filtering
Raw ultrasonic data often contains spikes and noise. A robust filtering pipeline was implemented:
- **Outlier Rejection:** Discards sudden spikes exceeding a predefined threshold (`spikeThreshold`).
- **Sliding Window Average Filter:** Utilizes a fixed window (size = 20) to compute the moving average of consecutive readings, significantly smoothing the output distance curve.
- **Hardware Compensation:** Added a +1.5cm physical offset calibration for ultimate precision.


---
