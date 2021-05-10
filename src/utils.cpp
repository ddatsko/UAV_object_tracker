#include "yolo_tracker/utils.hpp"
#include <mrs_lib/subscribe_handler.h>
#include <sensor_msgs/CameraInfo.h>
#include <mrs_msgs/DynamicsConstraints.h>
#include <ros/time.h>

template<class M>
bool _wait_for_msg(ros::NodeHandle &n, std::string &&topic, M &res) {
    mrs_lib::SubscribeHandler <M> subscribe_handler(n, topic);
    ros::Time start = ros::Time::now();
    ros::Rate rate(5);
    while (!subscribe_handler.hasMsg()) {
        rate.sleep();
        ros::spinOnce();
        if (ros::Time::now() - start > ros::Duration(5, 0)) {
            break;
        }
    }
    if (subscribe_handler.hasMsg()) {
        res = *subscribe_handler.getMsg();
        subscribe_handler.stop();
        return true;
    }
    subscribe_handler.stop();
    return false;

}


void read_image_size(ros::NodeHandle &n, const std::string &uav_name, double &image_width, double &image_height) {
    sensor_msgs::CameraInfo msg;
    if (_wait_for_msg<sensor_msgs::CameraInfo>(n, std::string() + "/" + uav_name + "/mobius_front/camera_info", msg)) {
        image_height = msg.height;
        image_width = msg.width;
    } else {
        ROS_WARN("Could not read image size. Using default 1280x720");
        image_height = 720;
        image_width = 1280;
    }
}


double read_max_speed(ros::NodeHandle &n, const std::string &uav_name) {
    mrs_msgs::DynamicsConstraints msg;
    if (_wait_for_msg<mrs_msgs::DynamicsConstraints>(n, std::string() + "/" + uav_name +
                                                        "/control_manager/current_constraints", msg)) {
        return msg.horizontal_speed;
    } else {
        ROS_WARN("Could not read max sped constaint. Using default 1.0");
        return 1.0;
    }
}

