#include <yolo_tracker/UAVController.hpp>

#include <ros/ros.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <tuple>

UAVController::UAVController(const std::string &uav_name, ros::NodeHandle &n) : uav_name(uav_name) {
    frame_id = uav_name + "/fcu_untilted";
    reference_client = n.serviceClient<mrs_msgs::ReferenceStampedSrv>("/" + uav_name + "/control_manager/reference");
    request_counter = 0;
}


void UAVController::move(mrs_msgs::Reference &position) {
    mrs_msgs::ReferenceStampedSrv srv;

    srv.request.header.frame_id = frame_id;
    srv.request.header.seq = request_counter++;
    srv.request.header.stamp = ros::Time::now();

    srv.request.reference = position;

    if (reference_client.call(srv)) {
        if (!srv.response.success) {
            ROS_INFO_STREAM("Move successfull. Success: " << srv.response.success + '0' << ". Message: "
                                                          << srv.response.message;);
        }
    } else {
        ROS_ERROR("Service call error...");
    }

}
