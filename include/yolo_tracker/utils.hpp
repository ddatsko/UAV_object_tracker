#ifndef _YOLO_TRACKER_UTILS_HPP
#define _YOLO_TRACKER_UTILS_HPP

#include <ros/ros.h>
#include <string>

void read_image_size(ros::NodeHandle &n, const std::string &uav_name, double &image_width, double &image_height);

double read_max_speed(ros::NodeHandle &n, const std::string &uav_name);


#endif
