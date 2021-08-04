#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mrs_msgs/FutureTrajectory.h>
#include <mrs_msgs/SpawnerDiagnostics.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <iostream>
#include <string>

#define MAX_UAV_NUMBER 10


double marker_radius = 1.5;
bool test1 = false;

void DiagnosticsCallback(const mrs_msgs::SpawnerDiagnostics::ConstPtr& diagnostics);
void CurrentPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg, int uav_number);
void PublishMarker(const std::vector<geometry_msgs::Pose>& obj_pose);


// Publishers and subscribers
ros::Publisher marker_pub;
std::vector<ros::Subscriber> uav_current_pose_sub(MAX_UAV_NUMBER);
std::vector<std::string> topic(MAX_UAV_NUMBER);
ros::Subscriber diagnostics_sub;

// Messages
mrs_msgs::SpawnerDiagnostics diagnostics;
geometry_msgs::PoseArray uav_current_pose;
std::vector<geometry_msgs::Pose> uav_current_poses(MAX_UAV_NUMBER);
mrs_msgs::FutureTrajectory uav_applied_ref;

void DiagnosticsCallback(const mrs_msgs::SpawnerDiagnostics::ConstPtr& msg){
    if (test1)
        return;
    diagnostics.active_vehicles = msg -> active_vehicles;
    //ROS_INFO_STREAM("UAV list: " << diagnostics.active_vehicles[0] << ", " << diagnostics.active_vehicles[1]);
    test1 = true;
}

void CurrentPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg, int uav_number){
    uav_current_pose.poses = msg -> poses;
    uav_current_poses[uav_number-1] = uav_current_pose.poses[uav_number-1];
    //ROS_INFO_STREAM("uav1: " << uav_current_poses[0] << "uav2: " << uav_current_poses[1]);
}

void PublishMarker(const std::vector<geometry_msgs::Pose>& obj_pose){
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markers;

    for(int i=0; i<2; i++){
        marker.header.frame_id = "/common_origin";
        marker.header.stamp = ros::Time::now();
        marker.ns = "marker";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE; 
        marker.action = visualization_msgs::Marker::ADD;

    //for fixed sphere
    // marker.pose.position.x = 5.0; 
    // marker.pose.position.y = 5.0; 
    // marker.pose.position.z = 5.0;  
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;

        marker.pose = obj_pose[i];
        marker.scale.x = 2*marker_radius; 
        marker.scale.y = 2*marker_radius; 
        marker.scale.z = 2*marker_radius; 
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.25;
        marker.lifetime = ros::Duration();
        markers.markers.push_back(marker);
    }
    marker_pub.publish(markers);
}


int main(int argc, char **argv){
    
    // Initialization
    ros::init(argc, argv,"marker");
    ros::NodeHandle n;
    ros::Rate r(30);


    // Subscribers and publishers

    // create subscriber for active UAV list
    diagnostics_sub = n.subscribe("mrs_drone_spawner/diagnostics", 10, DiagnosticsCallback);
    while(!test1){
        ros::spinOnce();
        r.sleep();
    }
    
    std::vector<boost::function<void (const geometry_msgs::PoseArray::ConstPtr&)>> f;
    f.resize(MAX_UAV_NUMBER);

    for(int i=0; i<diagnostics.active_vehicles.size(); i++){
        topic[i] = "/" + diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/custom_predicted_poses";
        f[i] = boost::bind(CurrentPoseCallback, _1, i+1);
        uav_current_pose_sub[i] = n.subscribe(topic[i], 10, f[i]);
    }

    // create publisher for spheres
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);

    while(ros::ok){
        PublishMarker(uav_current_poses);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}