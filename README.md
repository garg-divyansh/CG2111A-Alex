# 🤖 Alex to the Rescue  
### CG2111A Engineering Principles and Practice II  
**Team B02-G7 | Semester 2 AY2025/2026**

![NUS](https://img.shields.io/badge/NUS-Computer%20Engineering-orange?style=for-the-badge)  
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros)  
![RaspberryPi](https://img.shields.io/badge/Raspberry%20Pi-4-red?style=for-the-badge&logo=raspberrypi)  
![Arduino](https://img.shields.io/badge/Arduino-Mega-00979D?style=for-the-badge&logo=arduino)  
![C++](https://img.shields.io/badge/C++-Firmware-blue?style=for-the-badge&logo=c%2B%2B)  
![Python](https://img.shields.io/badge/Python-Backend-yellow?style=for-the-badge&logo=python)  
![Docker](https://img.shields.io/badge/Docker-ROS2%20Env-2496ED?style=for-the-badge&logo=docker)  
![Status](https://img.shields.io/badge/Status-Completed-success?style=for-the-badge)

---

## 📑 Table of Contents
1. [Introduction](#-section-1-introduction)
2. [Review of State of the Art](#-section-2-review-of-state-of-the-art)
3. [System Architecture](#-section-3-system-architecture)
4. [Hardware Design](#-section-4-hardware-design)
5. [Firmware Design](#-section-5-firmware-design)
6. [Software Design](#-section-6-software-design)
7. [Lessons Learnt - Conclusion](#-section-7-lessons-learnt---conclusion)
8. [Appendix](#-appendix)
9. [References](#-references)

---

# 📌 Section 1 Introduction

The Moonbase SG Rescue Mission is a search and rescue quest in the lunar environment. After an unexpected sudden oxygen tank explosion, astronauts are severely injured and are unable to move around to retrieve medical supplies. This emergency requires urgent supply of medical supplies (medpaks) to sustain them.

To supply the medpak, the project focuses on designing and building a remotely operated rescue robot **“Alex”** with two operators. Alex navigates through an unknown environment with obstacles that represent the damaged moonbase. Using sensors such as LiDAR, a color sensor, and a camera, the robot must detect medpak locations, avoid obstacles, and understand its surroundings. It uses a robotic arm to pick up the correct medpak and deliver it safely to the astronauts. Additionally, the robot is also expected to map the environment, for future rescue operations.

The system operates under strict constraints:
- Limited camera usage (maximum 15 photographs)
- Remote control without direct visual access
- Precise multi-operator coordination

This project integrates mechanical, electronic, and software components to solve a real-world problem, providing hands-on experience in robotics, navigation, sensing, and system coordination.

---

# 🧠 Section 2 Review of State of the Art

## 2.1 Clearpath Jackal UGV

- Compact 4-wheeled unmanned ground vehicle  
- ROS-compatible platform  
- Supports LiDAR, cameras, robotic arm  
- Multi-operator control  
- Battery life: ~4–6 hours  

## 2.2 Endeavor Robotics 310 SUGV

- Compact, tracked robot for military/SAR use  
- Operates without line-of-sight  
- Controlled via Android-based GUI  
- Supports modular payloads (thermal, chemical sensors)  

### ⚖️ Comparison

| Platform | Strengths | Weaknesses |
|----------|----------|------------|
| Jackal | Modular, extensible, multi-operator | Wheeled drivetrain |
| 310 SUGV | All-terrain mobility, rugged | Single operator load |

---

# 🏗️ Section 3 System Architecture

![System Architecture](assets/architecture/system_architecture.png)

---

# 🔩 Section 4 Hardware Design

The Alex Robot comprises a **4WD 2-layer chassis**.

### 🔽 Bottom Layer
- Arduino Mega  
- L293D Motor Driver Shield  
- LiDAR Sensor  
- 4 DC Motors  
- TCS3200 Color Sensor  

### 🔼 Top Layer
- Raspberry Pi 4  
- RPi Camera  
- 4-DoF Robot Arm (MeArm v0.4)  
- Battery packs  
- Emergency Stop button  

![Hardware Design](assets/hardware/robot_chassis.png)

---

# ⚙️ Section 5 Firmware Design

## 5.1 High-Level Algorithm - Arduino Mega

### Initialization
- UART initialized at 9600 bps (8N1)  
- GPIO configured for motors, sensors, servos  
- Timer5 configured for PWM (20 ms period)  
- Interrupts enabled (INT2: color sensor, INT3: E-stop)  

### Main Loop
- Check E-stop state  
- Receive and validate packets  
- Execute commands  
- Send response packets  

---

## 5.2 Communication Protocol

### UART Configuration
