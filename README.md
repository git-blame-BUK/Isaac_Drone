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
- Isaac publishes into ekf2 and seems to be precise  
- Nvidia Isaac is ready
- 3D Modeling the Drone Frame is done 
- Material reasearch for fillament : PAHT-CF
- FC is ready
- Companion computer is ready
- ESC is done soldering