#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mrs_msgs/FutureTrajectory.h>
#include <mrs_msgs/SpawnerDiagnostics.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <iostream>
#include <string>

#define UAV_NUMBER_MAX 10


double obst_sphere1_radius_ = 1.5;
bool test1 = false;

void pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg, int uav_number);
void publishSphereMarker(const std::vector<geometry_msgs::Pose>& obj_pose);
void diagnostics_callback(const mrs_msgs::SpawnerDiagnostics::ConstPtr& diagnostics);


// Publishers and subscribers
ros::Publisher marker_spheres_pub;
std::vector<ros::Subscriber> pose_subscriber(UAV_NUMBER_MAX);
std::vector<std::string> topic(UAV_NUMBER_MAX);
ros::Subscriber diagnostics_subscriber;

// Messages
geometry_msgs::PoseArray uav_pose;
std::vector<geometry_msgs::Pose> uav_poses(UAV_NUMBER_MAX);
mrs_msgs::SpawnerDiagnostics diagnostics;


void pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg, int uav_number){
    uav_pose.poses = msg -> poses;
    uav_poses[uav_number-1] = uav_pose.poses[uav_number-1];
    //ROS_INFO_STREAM("uav1: " << uav_poses[0] << "uav2: " << uav_poses[1]);
}

void diagnostics_callback(const mrs_msgs::SpawnerDiagnostics::ConstPtr& msg){
    if (test1)
        return;
    diagnostics.active_vehicles = msg -> active_vehicles;
    //ROS_INFO_STREAM("UAV list: " << diagnostics.active_vehicles[0] << ", " << diagnostics.active_vehicles[1]);
    test1 = true;
}

void publishSphereMarker(const std::vector<geometry_msgs::Pose>& obj_pose){
    visualization_msgs::Marker marker_sphere1;
    visualization_msgs::MarkerArray marker_spheres;

    for(int i=0; i<2; i++){
        marker_sphere1.header.frame_id = "/common_origin";
        marker_sphere1.header.stamp = ros::Time::now();
        marker_sphere1.ns = "marker";
        marker_sphere1.id = i;
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

        marker_sphere1.pose = obj_pose[i];
        marker_sphere1.scale.x = 2*obst_sphere1_radius_; 
        marker_sphere1.scale.y = 2*obst_sphere1_radius_; 
        marker_sphere1.scale.z = 2*obst_sphere1_radius_; 
        marker_sphere1.color.r = 1.0f;
        marker_sphere1.color.g = 0.0f;
        marker_sphere1.color.b = 0.0f;
        marker_sphere1.color.a = 0.25;
        marker_sphere1.lifetime = ros::Duration();
        marker_spheres.markers.push_back(marker_sphere1);
    }
    marker_spheres_pub.publish(marker_spheres);
}


int main(int argc, char **argv){
    
    // Initialization

    // init node
    ros::init(argc, argv,"marker");
    ros::NodeHandle n;
    ros::Rate r(30);


    // Subscribers and publishers

    // create subscriber for active UAV list
    diagnostics_subscriber = n.subscribe("mrs_drone_spawner/diagnostics", 10, diagnostics_callback);
    while(!test1){
        ros::spinOnce();
        r.sleep();
    }
    
    std::vector<boost::function<void (const geometry_msgs::PoseArray::ConstPtr&)>> f;
    f.resize(UAV_NUMBER_MAX);

    for(int i=0; i<diagnostics.active_vehicles.size(); i++){
        topic[i] = "/" + diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/custom_predicted_poses";
        f[i] = boost::bind(pose_callback, _1, i+1);
        pose_subscriber[i] = n.subscribe(topic[i], 10, f[i]);
    }

    // create publisher for spheres
    marker_spheres_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);

    while(ros::ok){
        publishSphereMarker(uav_poses);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}