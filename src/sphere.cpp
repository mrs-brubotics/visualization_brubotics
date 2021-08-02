#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <mrs_msgs/FutureTrajectory.h>
#include <iostream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <string>

void pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg, int uav_number);
void publishSphereMarker(const geometry_msgs::PoseArray& obj_pose, int uav_number);


double obst_sphere1_radius_ = 1.5;

ros::Publisher marker_sphere1_pub;
ros::Publisher marker_sphere2_pub;
ros::Subscriber pose_subscriber1;
ros::Subscriber pose_subscriber2;

geometry_msgs::PoseArray uav_pose;

void pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg, int uav_number){
    uav_pose.poses = msg -> poses;
    ROS_INFO_STREAM("camera pos:\nx: " << uav_number);
    publishSphereMarker(uav_pose, uav_number);
}


void publishSphereMarker(const geometry_msgs::PoseArray& obj_pose, int uav_number){
    ROS_INFO_STREAM("ok4");
    visualization_msgs::Marker marker_sphere1;
    marker_sphere1.header.frame_id = "/common_origin";
    marker_sphere1.header.stamp = ros::Time::now();
    marker_sphere1.ns = "marker";
    marker_sphere1.id = 0;
    marker_sphere1.type = visualization_msgs::Marker::SPHERE; 
    marker_sphere1.action = visualization_msgs::Marker::ADD;

    //for fixed sphere
    // marker_sphere1.pose.position.x = 5.0; 
    // marker_sphere1.pose.position.y = 5.0; 
    // marker_sphere1.pose.position.z = 5.0;  
    // marker_sphere1.pose.orientation.x = 0.0;
    // marker_sphere1.pose.orientation.y = 0.0;
    // marker_sphere1.pose.orientation.z = 0.0;
    // marker_sphere1.pose.orientation.w = 1.0;

    marker_sphere1.scale.x = 2*obst_sphere1_radius_; 
    marker_sphere1.scale.y = 2*obst_sphere1_radius_; 
    marker_sphere1.scale.z = 2*obst_sphere1_radius_; 
    marker_sphere1.color.r = 1.0f;
    marker_sphere1.color.g = 0.0f;
    marker_sphere1.color.b = 0.0f;
    marker_sphere1.color.a = 0.25;
    marker_sphere1.lifetime = ros::Duration();
    if(uav_number == 1){
        marker_sphere1.pose = obj_pose.poses[0];
        marker_sphere1_pub.publish(marker_sphere1);
    }
    if(uav_number == 2){
        marker_sphere1.pose = obj_pose.poses[0];
        marker_sphere2_pub.publish(marker_sphere1);
    }
}

int main(int argc, char **argv){
    
    // Initialization

    // init node
    ros::init(argc, argv,"marker");
    ros::NodeHandle n;

    // create subscriber for uav_predicted pose
    boost::function<void (const geometry_msgs::PoseArray::ConstPtr&)> f1 = boost::bind(pose_callback, _1, 1);
    pose_subscriber1 = n.subscribe("/uav1/control_manager/dergbryan_tracker/custom_predicted_poses", 10, f1);

    boost::function<void (const geometry_msgs::PoseArray::ConstPtr&)> f2 = boost::bind(pose_callback, _1, 2);
    pose_subscriber2 = n.subscribe("/uav2/control_manager/dergbryan_tracker/custom_predicted_poses", 10, f2);
    
    // create publisher for marker
    marker_sphere1_pub = n.advertise<visualization_msgs::Marker>("visualization_marker1", 10);
    marker_sphere2_pub = n.advertise<visualization_msgs::Marker>("visualization_marker2", 10);

    ros::spin();
    return 0;
}