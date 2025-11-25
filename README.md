# Embedded Control Firmware: Two-Wheeled Self-Balancing Robot

[![Platform](https://img.shields.io/badge/Platform-STM32F103-blue)](https://github.com/)
[![Sensor Fusion](https://img.shields.io/badge/Algorithm-Complementary%20Filter-green)](#sensor-fusion-implementation)
[![Control](https://img.shields.io/badge/Control-Cascaded%20PID-orange)](#control-system-topology)

**Developer:** Rongzhe Zhao (RongzheZhao2R2-lab)  
**Focus:** Real-time Embedded Systems, Sensor Fusion, Control Theory

---

## 1. Project Abstract
This repository houses the embedded firmware for a two-wheeled self-balancing robot (inverted pendulum model). The system is built on the **STM32** microcontroller ecosystem and implements a robust **Complementary Filter** to fuse data from a 6-axis MEMS IMU (MPU6050). The control logic utilizes a cascaded PID architecture to ensure dynamic stability, velocity tracking, and steering control.

Key Engineering Challenges Solved:
*   **Sensor Noise:** Mitigating accelerometer high-frequency noise and gyroscope drift.
*   **Real-time constraints:** Ensuring strict timing for the discrete control loop (5ms period).
*   **Non-linearity:** Stabilizing an inherently unstable non-linear system.

---

## 2. Control System Topology
The robot employs a **Cascaded PID** strategy, decoupling the control tasks into three parallel/nested loops.

```mermaid  
graph TD  
    subgraph "Outer Loop (Low Frequency)"  
    A[Target Velocity] -->|Error| B(Velocity PID)  
    B -->|Output: Target Pitch Angle| C  
    end  

    subgraph "Inner Loop (High Frequency 200Hz)"  
    C[Target Angle Ref] -->|Error| D(Vertical/Balance PD)  
    M[Sensor Fusion] -->|Current Pitch| D  
    M -->|Gyro Rate| D  
    end  

    subgraph "Steering"  
    E[Steering Command] --> F(Turn PD)  
    end  

    D --> G{Motor Mixer}  
    F --> G  
    G -->|PWM Signal| H[Motor Driver]

