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
# Sample video
[![Sample Video](https://res.cloudinary.com/marcomontalbano/image/upload/v1620653995/video_to_markdown/images/youtube--tZ_qHlezM2g-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=tZ_qHlezM2g "Sample Video")
