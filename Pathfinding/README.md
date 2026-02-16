## Dataflow 
Startup sequence
PX4 + MAVROS
- PX4 is running on the FCU.
- MAVROS bridges it to ROS and publishes key topics like:
```
/mavros/local_position/pose (PoseStamped)
/mavros/state (flight mode, armed, etc.)
```
 -  nvblox container (with Realsense / depth camera)
 -  The container runs realsense_example.launch.py.
 -  nvblox subscribes to depth + pose and builds:
 -  TSDF / ESDF internally,
 -  publishes various map-related topics (static_occupancy_grid, mesh, etc.),
 -  exposes the ESDF service, e.g. /nvblox_node/get_esdf_and_gradient.

Mission_manager
 - publishes a goal point

 - Planner_node (inside the same nvblox container - with Planner_core)
This node:
```
 /mavros/local_position/pose
 /uav/goal_pose
 ```
 - calls the ESDF service to get a local 3D ESDF block
 - runs A* on that ESDF-derived cost grid
 - publishes a nav_msgs/Path on /uav/trajectory

offboard_controller.py (same ROS domain)
Subscribes to:
```
/mavros/state
/mavros/local_position/pose
/uav/trajectory
```
Publishes:
```
/mavros/setpoint_position/local 
```
 - Contains state machine (phases like: wait_offboard, takeoff, holding, follow_trajectory, land).
### Ideas For Leightweight Trjectory Planning
 1. Building a dense 3D Map in Numpy with Python and replanning at 2Hz is to heavy
    --> Slicing into 2D at Height and only considering e.g. 0,5 m above the refrence and below keeps 3D avoidance but decreases weight of the sliding singed distance fields.
    --> Density can be tuned down but leaves a random margin of error in cluttered envoironments 

 2. Isaac_Drone has a short break distance caused by the low overall weight
    --> Planning radius can be quite small 

 3. High values in ESDF from Nvblox mean save trajectory.
    --> Balancing pathlength and those values from ESDF constitute obstacle avoiding navigation.


