## Dataflow 
Nvblox publishes Map 
    (Mission Manager sets Heading)
        Trajectory Planner calculates Setpoints
            List of Setpoints passes to offboard_controller


### Ideas For Leightweight Trjectory Planning
 1. Building a dense 3D Map in Numpy with Python and replanning at 2Hz is to heavy
    --> Slicing into 2D at Height and only considering e.g. 0,5 m above the refrence and below keeps 3D avoidance but decreases weight of the occupancy Grid.
    --> Density can be tuned down but leaves a random margin of error in cluttered envoironments 
 2. Isaac_Drone has a short break distance caused by the low overall weight
    --> Planning radius can be quite small 



