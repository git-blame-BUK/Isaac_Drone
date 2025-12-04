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

## Quickstart

1. The Internal the Z Axis of both systems (Px4 and Isaac) oppose each other 
    - quick fix is a transform of the Z Axis with the Camera_link topic 
        ```bash 
        ros2 run tf2_ros static_transform_publisher 0 0 0 1 0 0 0 camera0_link camera0_link_frd
        ```
2. Subscribe Px4 via Mavros   
    - This launch file Subscribes to the MavRouter prebuilt from ArkOS (useable for GCS)
        ```bash 
        ros2 launch mavros px4.launch fcu_url:=udp://:14600@127.0.0.1:14550
        ```
    
3. Set External Odometry relay (Camera-Mavros-Px4)
    - A topic that publishes Isaacs Odometry (VIo) to be used by Px4
        ```bash
        bash ros2 run topic_tools relay /visual_slam/tracking/odometry /mavros/odometry/out
        ```
4. Start the Nvblox or cuVSLAM session
