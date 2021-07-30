#include <visualization_msgs/Marker.h>
#include <ros/ros.h>



int main(int argc, char **argv){
    
    // Initialization
    // ^^^^^^^^^^^^^^

    // init node
    ros::init(argc, argv,"marker");
    ros::NodeHandle n;

    ros::Publisher marker_sphere1_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    double obst_sphere1_radius_ = 10;
    double obst_sphere1_center_x = 0.3;
    double obst_sphere1_center_y = 0.3;
    double obst_sphere1_center_z = 0.65;
    
    while (ros::ok()){

    visualization_msgs::Marker marker_sphere1;
    marker_sphere1.header.frame_id = "/common_origin";
    marker_sphere1.header.stamp = ros::Time::now();
    marker_sphere1.ns = "marker";
    marker_sphere1.id = 0;
    marker_sphere1.type = visualization_msgs::Marker::SPHERE; 
    marker_sphere1.action = visualization_msgs::Marker::ADD;
    marker_sphere1.pose.position.x = obst_sphere1_center_x; 
    marker_sphere1.pose.position.y = obst_sphere1_center_y; 
    marker_sphere1.pose.position.z = obst_sphere1_center_z;  
    marker_sphere1.pose.orientation.x = 0.0;
    marker_sphere1.pose.orientation.y = 0.0;
    marker_sphere1.pose.orientation.z = 0.0;
    marker_sphere1.pose.orientation.w = 1.0;
    marker_sphere1.scale.x = 2*obst_sphere1_radius_; 
    marker_sphere1.scale.y = 2*obst_sphere1_radius_; 
    marker_sphere1.scale.z = 2*obst_sphere1_radius_; 
    marker_sphere1.color.r = 1.0f;
    marker_sphere1.color.g = 0.0f;
    marker_sphere1.color.b = 0.0f;
    marker_sphere1.color.a = 0.8;
    marker_sphere1.lifetime = ros::Duration();
    marker_sphere1_pub.publish(marker_sphere1);
    }

    return 0;


}