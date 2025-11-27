


# Description

This is a Project to explore the possibilities of state of the art Autonomous Navigation Implementations utilizing Computer Vision.
To establish realtime reliable state estimation and mapping Nvidias Isaac Project is used.
Main Challenges will be a small footprint and implementation of pathfinding decision making.

***Core Components***
 1. Jetson Orin Nano 8gb: Onboard Computation Tasks
 2. ArkV6 with Pab Jetson Carrier: Open Source Flightcontroller (PX4)
 3. Intel Realsense D435: DepthCamera

## Dataflow

DepthCamera --> State Estimation (Vio) + Pointcloud --> Pathfinding

## Updates
- Assembly complete

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/ab912019-94fd-4f83-a513-46b2cb706767" alt="1000007455" width="200"/></td>
    <td><img src="https://github.com/user-attachments/assets/f5631850-194a-46cc-8916-1bb056ddde30" alt="1000007454" width="200"/></td>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/f324a5ea-7b7e-4e55-9cfe-fc3f16929abf" alt="1000007452" width="200"/></td>
    <td><img src="https://github.com/user-attachments/assets/64b54573-65f1-479c-a795-9ef405951ebf" alt="1000007450" width="200"/></td>
  </tr>
</table>

- Isaac publishes into ekf2 and seems to be precise  
- Nvidia Isaac is ready
- 3D Modeling the Drone Frame is done 
- Material reasearch for fillament : PAHT-CF
- FC is ready
- Companion computer is ready
- ESC is done soldering

