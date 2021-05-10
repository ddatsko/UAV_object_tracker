# UAV_object_tracker
ROS package that makes [CTU MRS](https://github.com/ctu-mrs) drone follow an object
# Dependencies:
This ROS package depends on:
- mrs_lib
- mrs_msgs
- roscpp
- rospy
- sensor_msgs
- std_msgs

Before running the simulation, follow [MRS UAV System installation](https://github.com/ctu-mrs/mrs_uav_system#installation)

# Launch
To launch everything just use the ```launch/tracker.launch``` file provided with needed parameters

# Simulation
To run the simulation, you can use ``` simulation_scripts/start.sh``` script. It will run gazebo with a walking person world, spawn the drone and make all the setup.
Then, use ```status``` fo tmuxinator to fly closer to the person and direct the drone to it manually. As soon as the drone can spot a person, it will start following it and no actions from you are needed anymore
### You can find the sample video [here](https://www.youtube.com/watch?v=tZ_qHlezM2g)
