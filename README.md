# PID Line Follower Robot (Arduino Nano)

This repository contains the firmware, hardware design (PCB), and mechanical files (3D) for an autonomous **Line Follower Robot**. The robot utilizes a PID (Proportional-Integral-Derivative) control algorithm to accurately follow tracks, including complex turns and dashed lines.

The project demonstrates a full-stack robotics implementation, from custom PCB design in **EasyEDA** and 3D modeling in **Fusion 360** to embedded C++ programming.

## 📸 Project Gallery
| **3D Design (Fusion 360)** | **Real Implementation** |
|:--------------------------:|:-----------------------:|
| ![3D Design](mechanical/render_preview.png) | ![Real Robot](docs/robot_top_view.jpg) |
| *Designed in Fusion 360* | *Final Assembly with Custom PCB* |

## ⚙️ Hardware Specifications
[cite_start]Based on the final implementation[cite: 31, 82, 314]:

| Component | Specification | Function |
| :--- | :--- | :--- |
| **Microcontroller** | Arduino Nano (ATmega328P) | Central processing unit. |
| **Motor Driver** | L293D | Controls motor direction and speed (PWM). |
| **Sensors** | 3x Photodiodes + LED Superbright | Detects line contrast (Black/White). |
| **Actuators** | 2x DC Motors N20 (60 RPM) | Low RPM chosen for higher torque and stability. |
| **Power System** | Li-Po Battery + Step-Down Module | Regulated to 8V for safety and stability. |
| **Interface** | OLED Display + Buzzer + 5 Buttons | For debugging, mode selection, and status. |
| **PCB Design** | Custom Shield (EasyEDA) | Integrates driver, sensor ports, and UI. |

## 🧠 Software & Control Logic

### PID Control
The robot uses a closed-loop control system to correct errors in real-time.
* [cite_start]**Kp (Proportional):** `17` - Reacts to the current error[cite: 239].
* **Ki (Integral):** `0.4` - Corrects accumulated past errors.
* **Kd (Derivative):** `0.3` - Predicts future errors to dampen oscillation.

```cpp
// Snippet from main code
lf.setPID(17, 0.4, 0.3);
lf.polaritasMotor(1, 0); // Motor polarity configuration
