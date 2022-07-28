#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <iostream>
#include <jsk_gui_msgs/YesNo.h>

class CameraPoseSub
{
public:
    void odomCallback(const fiducial_msgs::FiducialTransform::ConstPtr& msg);
    double pos_x;
    double pos_y;
    double pos_z;
    double or_x;
    double or_y;
    double or_z;
    double or_w;
    double time_last_seen;
};

void CameraPoseSub::odomCallback(const fiducial_msgs::FiducialTransform::ConstPtr& msg){

    int fid_id = msg->fiducial_id;
    if (fid_id == 1)
    {
        time_last_seen = ros::Time::now().toSec();
        pos_x = msg->transform.translation.x;
        pos_y = msg->transform.translation.y;
        pos_z = msg->transform.translation.z;
        or_x = msg->transform.rotation.x;
        or_y = msg->transform.rotation.y;
        or_z = msg->transform.rotation.z;
        or_w = msg->transform.rotation.w;
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_pose_pub");

    ros::NodeHandle nh;

    CameraPoseSub cps;
    ros::Subscriber cameraPoseSub = nh.subscribe("/fiducial_transforms", 10, &CameraPoseSub::odomCallback, &cps);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0);
    ros::ServiceClient client = nh.serviceClient<jsk_gui_msgs::YesNo::Response>("/rviz/yes_no_button");

    int marker_id = 0;
    double threshold = 0.2;

    while(ros::ok())
    {
        double time_stamp = ros::Time::now().toSec();
        jsk_gui_msgs::YesNo::Request req;
        jsk_gui_msgs::YesNo::Response res;

        client.call(req, res);
        ROS_INFO_STREAM("TEST");
        if(res.yes){

            ROS_INFO_STREAM("Button yes");

            if (time_stamp - cps.time_last_seen < threshold)
            {
                ROS_INFO_STREAM("Marker seen within threshold time!");
                visualization_msgs::Marker marker;
                marker.header.frame_id = "camera_color_optical_frame";
                marker.header.stamp = ros::Time();
                marker.ns = "my_namespace";
                marker.id = marker_id;
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = cps.pos_x;
                marker.pose.position.y = cps.pos_y;
                marker.pose.position.z = cps.pos_z;
                marker.pose.orientation.x = cps.or_x;
                marker.pose.orientation.y = cps.or_y;
                marker.pose.orientation.z = cps.or_z;
                marker.pose.orientation.w = cps.or_w;
                marker.scale.x = 0.2;
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                vis_pub.publish( marker );
                marker_id += 1;
            }
            else{
                ROS_INFO_STREAM("Marker not seen!");
            }
        }

        ros::spinOnce();
    }
    return 0;
}