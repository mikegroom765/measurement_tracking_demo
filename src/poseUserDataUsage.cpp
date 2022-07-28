#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <jsk_gui_msgs/YesNo.h>

class CameraPoseSub
{
public:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    double pos_x;
    double pos_y;
    double pos_z;
    double or_x;
    double or_y;
    double or_z;
    double or_w;
};

void CameraPoseSub::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

    pos_x = msg->pose.pose.position.x;
    pos_y = msg->pose.pose.position.y;
    pos_z = msg->pose.pose.position.z;
    or_x = msg->pose.pose.orientation.x;
    or_y = msg->pose.pose.orientation.y;
    or_z = msg->pose.pose.orientation.z;
    or_w = msg->pose.pose.orientation.w;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_pose_pub");

    ros::NodeHandle nh;

    CameraPoseSub cps;
    ros::Subscriber cameraPoseSub = nh.subscribe("rtabmap/odom", 10, &CameraPoseSub::odomCallback, &cps);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0);
    ros::ServiceClient client = nh.serviceClient<jsk_gui_msgs::YesNo::Response>("/rviz/yes_no_button");

    int marker_id = 0;

    while(ros::ok())
    {
        ros::Time stamp = ros::Time::now();
        jsk_gui_msgs::YesNo::Request req;
        jsk_gui_msgs::YesNo::Response res;

        if(client.call(req, res)){
            ROS_INFO_STREAM("service request was successful!");
        }
        if(res.yes){
            ROS_INFO_STREAM("Button yes");
        }
        else{
            ROS_INFO_STREAM("Button no");
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
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
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        vis_pub.publish( marker );
        marker_id += 1;
        
        

        ros::spinOnce();
    }
    return 0;
}