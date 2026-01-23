
# Isaac Drone Project

## Project Overview

This project explores the latest advancements in autonomous navigation leveraging state-of-the-art computer vision. Reliable, real-time state estimation and mapping are achieved using NVIDIA's Isaac platform, with the goal of delivering robust, efficient onboard processing.

**Key Objectives:**
- Achieve precise real-time state estimation.
- Implement advanced pathfinding and autonomous decision making.
- Optimize for small footprint embedded platforms.

## Core Components

1. **Jetson Orin Nano 8GB:** Onboard computation.
2. **ArkV6 with Pab Jetson Carrier (PX4):** Open-source flight controller.
3. **Intel RealSense D435:** Depth camera for spatial awareness.

## System Dataflow

```
Depth Camera → State Estimation (VIO) + Point Cloud → Pathfinding
```

## Project Milestones
1. ESC is done soldering
2. Companion computer is ready
3. FC is ready
4. Material research for filament : PAHT-CF
5. 3D Modeling the Drone Frame is done
6. Nvidia Isaac is ready
7. Isaac publishes into ekf2 and seems to be precise  
8. Assembly complete
9. Maiden Flight ready 
10. Maiden Flight succesfull xy stability +/- 5cm
11. Offboard Maiden in demo Mode succesfull 
12. Pathplanning succes with A*

## Hardware Gallery

### 3D Printed Frame

<video src="https://github.com/user-attachments/assets/a0ec3edb-5fa1-4d56-b7c1-96a39f53448b" controls width="400">
  Your browser does not support the video tag.
</video>

### Completed Prototype

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/64b54573-65f1-479c-a795-9ef405951ebf" alt="Prototype 4" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/f5631850-194a-46cc-8916-1bb056ddde30" alt="Prototype 2" width="600"/></td>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/f324a5ea-7b7e-4e55-9cfe-fc3f16929abf" alt="Prototype 3" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/ab912019-94fd-4f83-a513-46b2cb706767" alt="Prototype 1" width="600"/></td>
    
  </tr>
</table>
