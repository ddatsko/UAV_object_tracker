#ifndef _UAV_CONTROLLER_HPP
#define _UAV_CONTROLLER_HPP

#include <string>
#include <tuple>
#include <ros/ros.h>
#include <mrs_msgs/Reference.h>

class UAVController {
private:
    ros::ServiceClient reference_client;
    std::string uav_name;
    int request_counter;
    std::string frame_id;

public:
    UAVController(const std::string &uav_name, ros::NodeHandle &n);

    void move(mrs_msgs::Reference &position);
};

#endif