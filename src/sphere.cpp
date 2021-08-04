#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mrs_msgs/SpawnerDiagnostics.h>
#include <mrs_msgs/FutureTrajectory.h>
#include <mrs_msgs/FuturePoint.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>

#include <iostream>
#include <string>

#define MAX_UAV_NUMBER 10


double marker_radius = 1.5;
bool test1 = false;
std::array<std::array<double, 2>, 3> uav_applied_ref;


void DiagnosticsCallback(const mrs_msgs::SpawnerDiagnostics::ConstPtr& diagnostics);
void CurrentPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg, int uav_number);
void AppliedRefCallback(const mrs_msgs::FutureTrajectory::ConstPtr& msg);
void PublishMarkers(const std::vector<geometry_msgs::Pose>& obj_pose, std::array<std::array<double, 2>, 3> ref);


// Publishers and subscribers
ros::Publisher marker_pub;
std::vector<ros::Subscriber> uav_current_pose_sub(MAX_UAV_NUMBER);
std::vector<ros::Subscriber> uav_applied_ref_sub(MAX_UAV_NUMBER);
ros::Subscriber diagnostics_sub;


// Messages
mrs_msgs::SpawnerDiagnostics diagnostics;
geometry_msgs::PoseArray uav_current_pose;
std::vector<geometry_msgs::Pose> uav_current_poses(MAX_UAV_NUMBER);
mrs_msgs::FutureTrajectory uav_applied_ref_traj;
mrs_msgs::FuturePoint uav_applied_ref_point;

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
    //ROS_INFO_STREAM("UAV " << uav_number << ": " uav_current_poses[uav_number-1]);
}

void AppliedRefCallback(const mrs_msgs::FutureTrajectory::ConstPtr& msg, int uav_number){
    uav_applied_ref_traj.points = msg -> points;

    uav_applied_ref[0][uav_number-1] = uav_applied_ref_traj.points[0].x;
    uav_applied_ref[1][uav_number-1] = uav_applied_ref_traj.points[0].y;
    uav_applied_ref[2][uav_number-1] = uav_applied_ref_traj.points[0].z;
    ROS_INFO_STREAM("UAV " << uav_number << ": x = " << uav_applied_ref[uav_number-1][0] << ", y = " << uav_applied_ref[uav_number-1][1] << ", z = " << uav_applied_ref[uav_number-1][2]);
}

void PublishMarkers(const std::vector<geometry_msgs::Pose>& obj_pose, std::array<std::array<double, 2>, 3> ref){
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker_sphere;

    // loop for spheres
    for(int i=0; i<2; i++){
        //sphere at applied ref pose
        marker_sphere.header.frame_id = "/common_origin";
        marker_sphere.header.stamp = ros::Time::now();
        marker_sphere.ns = "marker_sphere";
        marker_sphere.id = i;
        marker_sphere.type = visualization_msgs::Marker::SPHERE; 
        marker_sphere.action = visualization_msgs::Marker::ADD;
        marker_sphere.pose.position.x = ref[0][i];
        marker_sphere.pose.position.y = ref[1][i];
        marker_sphere.pose.position.z = ref[2][i];
        marker_sphere.pose.orientation.x = 0;
        marker_sphere.pose.orientation.y = 0;
        marker_sphere.pose.orientation.z = 0;
        marker_sphere.pose.orientation.w = 0.0;
        marker_sphere.scale.x = 1.0; 
        marker_sphere.scale.y = 1.0; 
        marker_sphere.scale.z = 1.0; 
        marker_sphere.color.r = 0.0f;
        marker_sphere.color.g = 1.0f;
        marker_sphere.color.b = 1.0f;
        marker_sphere.color.a = 0.50;
        marker_sphere.lifetime = ros::Duration();
        markers.markers.push_back(marker_sphere);
        
        //sphere at current pose
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
    
    // Initialization //
    ros::init(argc, argv,"marker");
    ros::NodeHandle n;
    ros::Rate r(30);

    // Subscribers and publishers //
    // create subscriber for active UAV list
    diagnostics_sub = n.subscribe("mrs_drone_spawner/diagnostics", 10, DiagnosticsCallback);

    while(!test1){
        ros::spinOnce();
        r.sleep();
    }
    
    // create subscribers
    std::vector<boost::function<void (const geometry_msgs::PoseArray::ConstPtr&)>> f;
    f.resize(MAX_UAV_NUMBER);
    std::vector<boost::function<void (const mrs_msgs::FutureTrajectory::ConstPtr&)>> f1;
    f1.resize(MAX_UAV_NUMBER);

    for(int i=0; i<diagnostics.active_vehicles.size(); i++){
            // create subscriber for current uav pose
        f[i] = boost::bind(CurrentPoseCallback, _1, i+1);
        uav_current_pose_sub[i] = n.subscribe("/" + diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/custom_predicted_poses", 10, f[i]);
    }

    for(int i=0; i<diagnostics.active_vehicles.size(); i++){
            // create subscriber for uav applied ref
        f1[i] = boost::bind(AppliedRefCallback, _1, i+1);
        uav_applied_ref_sub[i] = n.subscribe("/" + diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/uav_applied_ref", 10, f1[i]);
    }

    // create publisher for markers
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);

    // Display
    while(ros::ok){
        PublishMarkers(uav_current_poses, uav_applied_ref);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}