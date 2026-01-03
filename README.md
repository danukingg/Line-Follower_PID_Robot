# PID Line Follower Robot (Arduino Nano)

![Arduino](https://img.shields.io/badge/Arduino-Nano-00979D?style=for-the-badge&logo=Arduino&logoColor=white)
![C++](https://img.shields.io/badge/Code-C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![EasyEDA](https://img.shields.io/badge/PCB-EasyEDA-blue?style=for-the-badge)
![Fusion 360](https://img.shields.io/badge/Mechanical-Fusion%20360-F57F24?style=for-the-badge&logo=autodesk&logoColor=white)

This repository contains the firmware, hardware design (PCB), and mechanical files (3D) for an autonomous **Line Follower Robot**. The robot utilizes a **PID (Proportional-Integral-Derivative)** control algorithm to accurately follow tracks, including complex turns and dashed lines.

The project demonstrates a **full-stack robotics implementation**, encompassing custom PCB design in EasyEDA, 3D modeling in Fusion 360, and embedded C++ programming.

## Project Gallery

| **3D Design (Fusion 360)** | **Real Implementation** |
|:--------------------------:|:-----------------------:|
| <img src="Mechanical/3D_Render.jpeg" height="250" alt="3D Render"> | <img src="Docs/Robot_Actual_View.jpeg" height="250" alt="Real Robot"> |
| *Designed in Fusion 360* | *Final Assembly with Custom PCB* |

## Hardware Specifications

Based on the final implementation design:

| Component | Specification | Function |
| :--- | :--- | :--- |
| **Microcontroller** | Arduino Nano (ATmega328P) | Central processing unit. |
| **Motor Driver** | L293D | Controls motor direction and speed via PWM. |
| **Sensors** | 3x Photodiodes + LED Superbright | Detects line contrast (Black/White). |
| **Actuators** | 2x DC Motors N20 (**60 RPM**) | Low RPM chosen for higher torque and stability. |
| **Power System** | Li-Po Battery + Step-Down Module | Regulated to **8V** for safety and stability. |
| **Interface** | OLED Display + Buzzer + 5 Buttons | Used for debugging, PID tuning, and mode selection. |
| **PCB Design** | Custom Shield (EasyEDA) | Integrates driver, sensor ports, and UI buttons. |

## Software & Control Logic

### 1. PID Control Algorithm
The robot uses a closed-loop control system to correct path errors in real-time. The PID parameters were tuned specifically for the 60 RPM motors:

* **Kp (Proportional):** `17.0` - Provides immediate reaction to the current error.
* **Ki (Integral):** `0.4` - Corrects accumulated past errors (steady-state error).
* **Kd (Derivative):** `0.3` - Dampens the oscillation to prevent overshooting.

```cpp
// PID Configuration Snippet
lf.setPID(17.0, 0.4, 0.3);
lf.polaritasMotor(1, 0); // Adjust based on motor wiring direction
