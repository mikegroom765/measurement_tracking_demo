#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <iostream>
#include <jsk_gui_msgs/YesNo.h>
#include <std_msgs/Empty.h>

class CameraPoseSub
{
public:
    void fiducialCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg); //::ConstPtr&
    void yesCallback(const std_msgs::Empty::ConstPtr& msg);
    double pos_x;
    double pos_y;
    double pos_z;
    double or_x;
    double or_y;
    double or_z;
    double or_w;
    bool record_pose;
    int id = 0;
};

void CameraPoseSub::fiducialCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){

    int fid_id = 0;
    if (msg->transforms.size() > 0)
    {
        fid_id = msg->transforms[0].fiducial_id;
        if (fid_id == 1)
        {
            pos_x = msg->transforms[0].transform.translation.x;
            pos_y = msg->transforms[0].transform.translation.y;
            pos_z = msg->transforms[0].transform.translation.z;
            or_x = msg->transforms[0].transform.rotation.x;
            or_y = msg->transforms[0].transform.rotation.y;
            or_z = msg->transforms[0].transform.rotation.z;
            or_w = msg->transforms[0].transform.rotation.w;
            id = fid_id;
        }
    }

}

void CameraPoseSub::yesCallback(const std_msgs::Empty::ConstPtr& msg){

    record_pose = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_pose_pub");

    ros::NodeHandle nh;

    CameraPoseSub cps;
    ros::Subscriber cameraPoseSub = nh.subscribe("/fiducial_transforms", 1, &CameraPoseSub::fiducialCallback, &cps);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0);
    ros::Subscriber record_pose = nh.subscribe("/yes", 10, &CameraPoseSub::yesCallback, &cps);

    int marker_id = 0;

    while(ros::ok())
    {
        double time_stamp = ros::Time::now().toSec();

        if(cps.record_pose){

            ROS_INFO_STREAM("Yes topic published!");
            ROS_INFO_STREAM(cps.id);

            if (cps.id == 1) //cps.fiducial_id == 1 when marker 1 is visible!
            {
                ROS_INFO_STREAM("Marker seen, vis marker published!");
                visualization_msgs::Marker marker;
                marker.header.frame_id = "camera_color_frame";
                marker.header.stamp = ros::Time();
                marker.ns = "my_namespace";
                marker.id = marker_id;
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = cps.pos_x;
                marker.pose.position.y = cps.pos_y;
                marker.pose.position.z = cps.pos_z;
                marker.pose.orientation.x = cps.or_x;
                marker.pose.orientation.y = cps.or_y - 0.7071068;
                marker.pose.orientation.z = cps.or_z;
                marker.pose.orientation.w = cps.or_w + 0.7071068;
                marker.scale.x = 0.2;
                marker.scale.y = 0.025;
                marker.scale.z = 0.025;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                vis_pub.publish( marker );
                marker_id += 1;
                cps.id = 0;
            }
            else{
                ROS_INFO_STREAM("Marker not seen!");
            }
            cps.record_pose = false;
        }

        ros::spinOnce();
    }
    return 0;
}