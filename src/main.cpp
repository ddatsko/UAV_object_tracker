#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <mrs_lib/subscribe_handler.h>
#include <sensor_msgs/Image.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
#include <yolo_tracker/ObjectDetector.hpp>
#include <yolo_tracker/UAVController.hpp>
#include <mrs_lib/param_loader.h>
#include "yolo_tracker/utils.hpp"

using namespace cv;


std::vector<std::string> classes;

int main(int argc, char **argv) {
    // Init ros
    const std::string node_name = "tracker";

    ros::init(argc, argv, node_name);
    ros::NodeHandle n;


    // Load all the parameters
    mrs_lib::ParamLoader param_loader(n, "tracker");

    int in_w, in_h;

    double max_speed;
    double person_height = 1.80;
    double camera_view_angle;
    double desired_distance;
    double image_width;
    double image_height;

    std::string object_to_track;
    std::string uav_name;
    std::string names_file;
    std::string model_def;
    std::string weights;


    param_loader.loadParam(node_name + "/uav_name", uav_name);
    param_loader.loadParam(node_name + "/view_angle", camera_view_angle);
    param_loader.loadParam(node_name + "/object_to_track", object_to_track);
    param_loader.loadParam(node_name + "/names_file", names_file);
    param_loader.loadParam(node_name + "/model_config_file", model_def);
    param_loader.loadParam(node_name + "/model_weights_file", weights);
    param_loader.loadParam(node_name + "/desired_distance_to_object", desired_distance);
    param_loader.loadParam(node_name + "/model_in_w", in_w);
    param_loader.loadParam(node_name + "/model_in_h", in_h);

    read_image_size(n, uav_name, image_width, image_height);
    max_speed = read_max_speed(n, uav_name);

    const double view_angle = camera_view_angle * ((double) in_w / image_width) / 180 * 3.14; // in radians
    const double vertical_view_angle = camera_view_angle * ((double) in_h / image_height) / 180 * 3.14;


    // Constructong detector and controller
    double thresh = 0.4;
    double nms_thresh = 0.25;
    ObjectDetector detector(object_to_track, model_def, weights, names_file, thresh, nms_thresh, 320, 320);
    UAVController uav_controller(uav_name, n);

    // Subscribe to topic for gettign images
    std::string camera_images_topic = "/" + uav_name + "/mobius_front/image_raw";
    mrs_lib::SubscribeHandler <sensor_msgs::Image> camera_images_handler(n, camera_images_topic);

    // Variables for detection and position estimation
    ProcessingResult object;
    ros::Time time_prev;
    ros::Duration time_elapsed;
    time_prev = ros::Time::now();


    while (ros::ok()) {
        // Wait for a new image to come
        while (!camera_images_handler.newMsg() && ros::ok()) {
            ros::Duration(0, 10e8).sleep();
            ros::spinOnce();
        }
        if (!ros::ok()) {
            break;
        }

        ROS_INFO("Got an image");

        // Getting a pointer to the Image and voncerting it to cv::Mat for detector
        auto raw_image = camera_images_handler.getMsg();
        cv::Mat image = cv_bridge::toCvCopy(raw_image)->image;


        time_elapsed = ros::Time::now() - time_prev;
        time_prev = ros::Time::now();


        if (detector.detect_object(image, object)) {
            ROS_INFO_STREAM(
                    "Detected object with center in: {x: " << object.rectangle.x + object.rectangle.width / 2 << ", y: "
                                                           <<
                                                           object.rectangle.y + object.rectangle.height / 2
                                                           << ". Width: " << object.rectangle.width << ". Height: "
                                                           << object.rectangle.height);
            ROS_INFO_STREAM(image.rows << " " << image.cols);

            // Center of the rectangle onthe image
            int x_center = object.rectangle.x + object.rectangle.width / 2;

            // Angle in radians between lower and top edge of detection rectangle
            double angle = ((double) object.rectangle.height / (double) image.rows) * vertical_view_angle;

            // Calculating the distance to the object, knowing its height
            double distance = (1 / std::tan(angle / 2)) * person_height / 2;
            ROS_INFO_STREAM("Distance to it: " << distance);

            // Calculate the distance of object from the center of image line in meters
            double movement_on_camera = x_center - image.cols / 2;
            double object_movement = distance * std::tan(movement_on_camera / image.cols * view_angle);
            ROS_INFO_STREAM("Person moved " << object_movement << " units away from the image center line");

            // Calculate where to move the drone to follow the target
            mrs_msgs::Reference movement;
            movement.position.y = -object_movement;

            // If the object is moving pretty fast, also rotate the drone to continue following the object with less movement needed
            if (std::abs(object_movement) / (time_elapsed.sec + time_elapsed.nsec / 10.0e9) > (max_speed / 2)) {
                movement.heading = (-movement_on_camera / image.cols) * view_angle;
            }

            // Keep the dron on the desired distance far from the object
            movement.position.x = distance - desired_distance;

            uav_controller.move(movement);
        }
        ros::spinOnce();
    }
    return 0;
}