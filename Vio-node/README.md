# How to Vio ?
- utilize Nvidia Isaac gpu accelerated Vslam to feed into PX4 EKF 
- VIO: Visual Inertial Odometry --> Pose Estimation
- ROS: Robot Operating System --> Internal Networking of Flight Controller Camera and Onboard Computer (ROS2 Humble)

## Tools
1. debug.py: checks for all important Datapoints from realsense 
2. poc.py: checks for realsense devices on host
3. imu_publisher.py: Systemtime stamped imu publisher for ROS
4. realsense_publisher.py is another proof of concept to visualize correct output in Rviz2

## Info
- [about Isaac](https://developer.nvidia.com/isaac)
- [Guide](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html)
    - this tutorial provides a starting point for Isaacs vio and slam capabilities


