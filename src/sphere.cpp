#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mrs_msgs/SpawnerDiagnostics.h>
#include <mrs_msgs/FutureTrajectory.h>
#include <mrs_msgs/FuturePoint.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>

#include <ros/ros.h>

#include <iostream>
#include <string>

#define MAX_UAV_NUMBER 10
#define MAX_POINTS_TRAJECTORY 50


double Ra = 0.35;
int test;
bool test1 = false;
bool test2 = false;
std::array<std::array<double, 2>, 3> uav_applied_ref;
std::array<std::array<std::array<double, 2>, 3>, 1000> predicted_trajectories;


// Publishers and subscribers
ros::Publisher marker_pub;
ros::Publisher trajectory_pub;
ros::Subscriber diagnostics_sub;
ros::Subscriber DERG_strategy_id_sub;
ros::Subscriber Sa_max_sub;
ros::Subscriber Sa_perp_max_sub;
ros::Subscriber tube_min_radius_sub;
std::vector<ros::Subscriber> uav_current_pose_sub(MAX_UAV_NUMBER);
std::vector<ros::Subscriber> uav_applied_ref_sub(MAX_UAV_NUMBER);
std::vector<ros::Subscriber> point_link_star_sub(MAX_UAV_NUMBER);
std::vector<ros::Subscriber> point_link_p0_sub(MAX_UAV_NUMBER);
std::vector<ros::Subscriber> point_link_p1_sub(MAX_UAV_NUMBER);
std::vector<ros::Subscriber> predicted_trajectory_sub(MAX_UAV_NUMBER);


// Messages
mrs_msgs::SpawnerDiagnostics diagnostics;
mrs_msgs::FutureTrajectory uav_applied_ref_traj;
mrs_msgs::FutureTrajectory predicted_traj;
geometry_msgs::PoseArray uav_current_pose;
geometry_msgs::Pose cylinder_pose;
geometry_msgs::Pose point_link_star;
geometry_msgs::Pose point_link_p0;
geometry_msgs::Pose point_link_p1;
std_msgs::Int32 _DERG_strategy_id_;
std_msgs::Int32 _Sa_max_;
std_msgs::Int32 _Sa_perp_max_;
std_msgs::Float32 tube_min_radius;
std::vector<geometry_msgs::Pose> uav_current_poses(MAX_UAV_NUMBER);
std::vector<geometry_msgs::Pose> point_link_stars(MAX_UAV_NUMBER);
std::vector<geometry_msgs::Pose> point_link_p0s(MAX_UAV_NUMBER);
std::vector<geometry_msgs::Pose> point_link_p1s(MAX_UAV_NUMBER);


class Point
{

    public:
        double x;
        double y;
        double z;
        Point(double a, double b, double c){
            x = a;
            y = b;
            z = c;
        }
        Point(){
        };

};


void DiagnosticsCallback(const mrs_msgs::SpawnerDiagnostics::ConstPtr& diagnostics);
void CurrentPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg, int uav_number);
void AppliedRefCallback(const mrs_msgs::FutureTrajectory::ConstPtr& msg, int uav_number);
void PredictedTrajectoryCallback(const mrs_msgs::FutureTrajectory::ConstPtr& msg, int uav_number);
void PointLinkStarCallback(const geometry_msgs::Pose::ConstPtr& msg, int uav_number);
void PointLinkP1Callback(const geometry_msgs::Pose::ConstPtr& msg, int uav_number);
void PointLinkP2Callback(const geometry_msgs::Pose::ConstPtr& msg, int uav_number);
void SaMaxCallback(const std_msgs::Int32::ConstPtr& msg);
void SaPerpMaxCallback(const std_msgs::Int32::ConstPtr& msg);
void TubeMinRadiusCallback(const std_msgs::Float32::ConstPtr& msg);
void DERGStrategyIdCallback(const std_msgs::Int32::ConstPtr& msg);
double getDistance(const Point& p1, const Point& p2);
Point getMiddle(const Point& pt1, const Point& pt2);
void CylinderOrientation(const Point &p1,const Point &p2, geometry_msgs::Pose& cylinder_pose, double& cylinder_height);
void PublishMarkers(const std::vector<geometry_msgs::Pose>& obj_pose, std::array<std::array<double, 2>, 3> ref, 
                    const std::vector<geometry_msgs::Pose>& pstar, 
                    const std::vector<geometry_msgs::Pose>& p0, 
                    const std::vector<geometry_msgs::Pose>& p1, 
                    std::array<std::array<std::array<double, 2>, 3>, 1000> predicted_trajectories);


void DiagnosticsCallback(const mrs_msgs::SpawnerDiagnostics::ConstPtr& msg){
    if (test1){
        return;
    }
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
    //ROS_INFO_STREAM("UAV " << uav_number << ": x = " << uav_applied_ref[0][uav_number-1] << ", y = " << uav_applied_ref[1][uav_number-1] << ", z = " << uav_applied_ref[2][uav_number-1]);
}

void PredictedTrajectoryCallback(const mrs_msgs::FutureTrajectory::ConstPtr& msg, int uav_number){
    predicted_traj.points = msg -> points;
    for(int i=0; i<predicted_traj.points.size(); i++){
        predicted_trajectories[i][0][uav_number-1] = predicted_traj.points[i].x;
        predicted_trajectories[i][1][uav_number-1] = predicted_traj.points[i].y;
        predicted_trajectories[i][2][uav_number-1] = predicted_traj.points[i].z;
        //ROS_INFO_STREAM("x : " << predicted_trajectories[i][0][uav_number-1]);
    }
    
    if(test2){
        return;
    }
    test2 = true;
}

void PointLinkStarCallback(const geometry_msgs::Pose::ConstPtr& msg, int uav_number){
    point_link_star.position = msg -> position;
    point_link_stars[uav_number-1].position = point_link_star.position;
    //ROS_INFO_STREAM("UAV" << uav_number << ": " << point_link_stars[uav_number-1].position);
}

void PointLinkP0Callback(const geometry_msgs::Pose::ConstPtr& msg, int uav_number){
    point_link_p0.position = msg -> position;
    point_link_p0s[uav_number-1].position = point_link_p0.position;
    //ROS_INFO_STREAM("UAV" << uav_number << ": " << point_link_p0s[uav_number-1].position);
}

void PointLinkP1Callback(const geometry_msgs::Pose::ConstPtr& msg, int uav_number){
    point_link_p1.position = msg -> position;
    point_link_p1s[uav_number-1].position = point_link_p1.position;
    //ROS_INFO_STREAM("UAV" << uav_number << ": " << point_link_p1s[uav_number-1].position);
}

void SaMaxCallback(const std_msgs::Int32::ConstPtr& msg){
    _Sa_max_.data = msg -> data;
    //ROS_INFO_STREAM("_Sa_max_ = " << _Sa_max_.data);
}

void SaPerpMaxCallback(const std_msgs::Int32::ConstPtr& msg){
    _Sa_perp_max_.data = msg -> data;
    //ROS_INFO_STREAM("_Sa_perp_max_ = " << _Sa_perp_max_.data);
}

void TubeMinRadiusCallback(const std_msgs::Float32::ConstPtr& msg){
    tube_min_radius.data = msg -> data;
    //ROS_INFO_STREAM("tube_min_radius = " << tube_min_radius.data);
}

void DERGStrategyIdCallback(const std_msgs::Int32::ConstPtr& msg){
    _DERG_strategy_id_.data = msg -> data;
    //ROS_INFO_STREAM("DERG_strategy_id: " << _DERG_strategy_id_.data);
}


double getDistance(const Point& p1, const Point& p2){
    double res;
    res = sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
    return res;
}

Point getMiddle(const Point& pt1, const Point& pt2){
    Point res;
    res.x = (pt1.x + pt2.x)*0.5;
    res.y = (pt1.y + pt2.y)*0.5;
    res.z = (pt1.z + pt2.z)*0.5;
    return res;
}

void CylinderOrientation(const Point &p1,const Point &p2, geometry_msgs::Pose& cylinder_pose, double& cylinder_height){

    Point middle = getMiddle(p1, p2);
    cylinder_height = getDistance(p1, p2);

    cylinder_pose.position.x = middle.x; 
    cylinder_pose.position.y = middle.y; 
    cylinder_pose.position.z = middle.z;

    Eigen::Vector3d cylinder_z_direction(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
    Eigen::Vector3d origin_z_direction(0., 0., 1.);
    cylinder_z_direction.normalize();
    Eigen::Vector3d axis;
    axis = origin_z_direction.cross(cylinder_z_direction);
    axis.normalize();
    double angle = acos(cylinder_z_direction.dot(origin_z_direction));
    cylinder_pose.orientation.x = axis.x() * sin(angle/2);
    cylinder_pose.orientation.y = axis.y() * sin(angle/2);
    cylinder_pose.orientation.z = axis.z() * sin(angle/2);
    cylinder_pose.orientation.w = cos(angle/2);

}

void PublishMarkers(const std::vector<geometry_msgs::Pose>& obj_pose,
                    std::array<std::array<double, 2>, 3> ref, const std::vector<geometry_msgs::Pose>& pstar,
                    const std::vector<geometry_msgs::Pose>& p0, const std::vector<geometry_msgs::Pose>& p1,
                    std::array<std::array<std::array<double, 2>, 3>, 1000> predicted_trajectories){
    
    visualization_msgs::Marker current_pose_sphere;
    visualization_msgs::Marker error_sphere;
    visualization_msgs::Marker applied_ref_sphere;
    visualization_msgs::Marker cylinder1;
    visualization_msgs::Marker cylinder2;
    visualization_msgs::Marker cylinder3;
    visualization_msgs::Marker cylinder_hemisphere11;
    visualization_msgs::Marker cylinder_hemisphere12;
    visualization_msgs::Marker cylinder_hemisphere21;
    visualization_msgs::Marker cylinder_hemisphere22;
    visualization_msgs::Marker cylinder_hemisphere31;
    visualization_msgs::Marker cylinder_hemisphere32;
    visualization_msgs::Marker line;
    visualization_msgs::Marker trajectory_sphere;
    visualization_msgs::MarkerArray trajectory_array;
    visualization_msgs::MarkerArray all_markers;
    geometry_msgs::Pose cylinder_pose;
    geometry_msgs::Point p;
    geometry_msgs::Point p_traj;
    double cylinder_height;

    // loop for each drone
    for(int i=0; i<2; i++){

        // error sphere at applied ref pose
        error_sphere.header.frame_id = "/common_origin";
        error_sphere.header.stamp = ros::Time::now();
        error_sphere.ns = "error_sphere";
        error_sphere.id = i;
        error_sphere.type = visualization_msgs::Marker::SPHERE; 
        error_sphere.action = visualization_msgs::Marker::ADD;
        error_sphere.pose.position.x = ref[0][i];
        error_sphere.pose.position.y = ref[1][i];
        error_sphere.pose.position.z = ref[2][i];
        error_sphere.pose.orientation.x = 0;
        error_sphere.pose.orientation.y = 0;
        error_sphere.pose.orientation.z = 0;
        error_sphere.pose.orientation.w = 0.0;
        error_sphere.scale.x = 2*_Sa_max_.data; 
        error_sphere.scale.y = 2*_Sa_max_.data; 
        error_sphere.scale.z = 2*_Sa_max_.data; 
        error_sphere.color.r = 0.8f;
        error_sphere.color.g = 0.898f;
        error_sphere.color.b = 1.0f;
        error_sphere.color.a = 0.2;
        error_sphere.lifetime = ros::Duration();
        all_markers.markers.push_back(error_sphere);

        // small sphere at current pose
        current_pose_sphere.header.frame_id = "/common_origin";
        current_pose_sphere.header.stamp = ros::Time::now();
        current_pose_sphere.ns = "current_pose_sphere";
        current_pose_sphere.id = i;
        current_pose_sphere.type = visualization_msgs::Marker::SPHERE; 
        current_pose_sphere.action = visualization_msgs::Marker::ADD;
        current_pose_sphere.pose = obj_pose[i];
        current_pose_sphere.scale.x = 2*Ra; 
        current_pose_sphere.scale.y = 2*Ra; 
        current_pose_sphere.scale.z = 2*Ra; 
        current_pose_sphere.color.r = 0.6f;
        current_pose_sphere.color.g = 0.6f;
        current_pose_sphere.color.b = 0.6f;
        current_pose_sphere.color.a = 0.35;
        current_pose_sphere.lifetime = ros::Duration();
        all_markers.markers.push_back(current_pose_sphere);

        // small sphere at applied ref pose
        applied_ref_sphere.header.frame_id = "/common_origin";
        applied_ref_sphere.header.stamp = ros::Time::now();
        applied_ref_sphere.ns = "applied_ref_sphere";
        applied_ref_sphere.id = i;
        applied_ref_sphere.type = visualization_msgs::Marker::SPHERE; 
        applied_ref_sphere.action = visualization_msgs::Marker::ADD;
        applied_ref_sphere.pose.position.x = ref[0][i];
        applied_ref_sphere.pose.position.y = ref[1][i];
        applied_ref_sphere.pose.position.z = ref[2][i];
        applied_ref_sphere.pose.orientation.x = 0;
        applied_ref_sphere.pose.orientation.y = 0;
        applied_ref_sphere.pose.orientation.z = 0;
        applied_ref_sphere.pose.orientation.w = 0.0;
        applied_ref_sphere.scale.x = 2*Ra; 
        applied_ref_sphere.scale.y = 2*Ra; 
        applied_ref_sphere.scale.z = 2*Ra; 
        applied_ref_sphere.color.r = 0.6f;
        applied_ref_sphere.color.g = 0.6f;
        applied_ref_sphere.color.b = 0.6f;
        applied_ref_sphere.color.a = 0.35;
        applied_ref_sphere.lifetime = ros::Duration();
        all_markers.markers.push_back(applied_ref_sphere);

        // ends points of the line
        p.x = obj_pose[i].position.x;
        p.y = obj_pose[i].position.y;
        p.z = obj_pose[i].position.z;
        line.points.push_back(p);

        // Predicted trajectory
        trajectory_sphere.points.clear();

        p_traj.x = predicted_trajectories[0][0][i];
        p_traj.y = predicted_trajectories[0][1][i];
        p_traj.z = predicted_trajectories[0][2][i];
        trajectory_sphere.points.push_back(p_traj);

        test = predicted_traj.points.size() / MAX_POINTS_TRAJECTORY;

        for(int j=test; j<(predicted_traj.points.size() - test); j+=test){
            p_traj.x = predicted_trajectories[j][0][i];
            p_traj.y = predicted_trajectories[j][1][i];
            p_traj.z = predicted_trajectories[j][2][i];
            trajectory_sphere.points.push_back(p_traj);
        }

        p_traj.x = predicted_trajectories[predicted_traj.points.size()-1][0][i];
        p_traj.y = predicted_trajectories[predicted_traj.points.size()-1][1][i];
        p_traj.z = predicted_trajectories[predicted_traj.points.size()-1][2][i];
        trajectory_sphere.points.push_back(p_traj);

        trajectory_sphere.header.frame_id = "/common_origin";
        trajectory_sphere.id = i;
        trajectory_sphere.header.stamp = ros::Time::now();
        trajectory_sphere.ns = "trajectory";
        trajectory_sphere.type = visualization_msgs::Marker::SPHERE_LIST;   // trajectory displaied as succession of small spheres
        // trajectory_sphere.type = visualization_msgs::Marker::LINE_STRIP; // trajectory displaied as a line strip
        trajectory_sphere.action = visualization_msgs::Marker::ADD;
        trajectory_sphere.pose.orientation.w = 0.0;
        trajectory_sphere.scale.x = 0.05; // width
        trajectory_sphere.scale.y = 0.05;
        trajectory_sphere.scale.z = 0.05;
        trajectory_sphere.color.r = 1.0f;
        trajectory_sphere.color.g = 0.0f;
        trajectory_sphere.color.b = 0.0f;
        trajectory_sphere.color.a = 1.0;
        all_markers.markers.push_back(trajectory_sphere);
        ROS_INFO_STREAM("number of points: " << trajectory_sphere.points.size() << ", test: " << test << ", size: " << predicted_traj.points.size());

        if(_DERG_strategy_id_.data == 1){

            // Blue tube
            // cylinder1 between point link star and applied ref
            Point p1, p2;
            p1.x = pstar[i].position.x;
            p1.y = pstar[i].position.y;
            p1.z = pstar[i].position.z;
            p2.x = ref[0][i];
            p2.y = ref[1][i];
            p2.z = ref[2][i];
            cylinder1.header.frame_id = "/common_origin";
            cylinder1.id = i;
            cylinder1.header.stamp = ros::Time::now();
            cylinder1.ns = "cylinder1";
            cylinder1.type = visualization_msgs::Marker::CYLINDER; 
            cylinder1.action = visualization_msgs::Marker::ADD;
            CylinderOrientation(p1, p2, cylinder_pose, cylinder_height);
            cylinder1.pose = cylinder_pose;
            cylinder1.scale.x = 2*_Sa_perp_max_.data;           // x radius
            cylinder1.scale.y = 2*_Sa_perp_max_.data;           // y radius
            cylinder1.scale.z = cylinder_height;     // height
            cylinder1.color.r = 0.4f;
            cylinder1.color.g = 0.698f;
            cylinder1.color.b = 1.0f;
            cylinder1.color.a = 0.6;
            all_markers.markers.push_back(cylinder1);

            // Blue tube
            // sphere12 at cylinder ends
            cylinder_hemisphere12.header.frame_id = "/common_origin";
            cylinder_hemisphere12.header.stamp = ros::Time::now();
            cylinder_hemisphere12.ns = "cylinder_hemisphere12";
            cylinder_hemisphere12.id = i;
            cylinder_hemisphere12.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere12.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere12.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere12.pose.position.x = ref[0][i];
            cylinder_hemisphere12.pose.position.y = ref[1][i];
            cylinder_hemisphere12.pose.position.z = ref[2][i];
            cylinder_hemisphere12.pose.orientation = cylinder_pose.orientation;
            //ROS_INFO_STREAM("sphere 1 " << cylinder_pose.orientation);      
            cylinder_hemisphere12.scale.x = 0.001 * _Sa_perp_max_.data; 
            cylinder_hemisphere12.scale.y = 0.001 * _Sa_perp_max_.data; 
            cylinder_hemisphere12.scale.z = 0.001 * _Sa_perp_max_.data; 
            cylinder_hemisphere12.color.r = 0.4f;
            cylinder_hemisphere12.color.g = 0.698f;
            cylinder_hemisphere12.color.b = 1.0f;
            cylinder_hemisphere12.color.a = 0.5;
            cylinder_hemisphere12.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere12);

            // Blue tube
            // sphere11 at cylinder ends
            cylinder_hemisphere11.header.frame_id = "/common_origin";
            cylinder_hemisphere11.header.stamp = ros::Time::now();
            cylinder_hemisphere11.ns = "cylinder_hemisphere11";
            cylinder_hemisphere11.id = i;
            cylinder_hemisphere11.type = visualization_msgs::Marker::MESH_RESOURCE;
            cylinder_hemisphere11.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere11.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere11.pose.position = pstar[i].position;
            CylinderOrientation(p2, p1, cylinder_pose, cylinder_height);
            cylinder_hemisphere11.pose.orientation = cylinder_pose.orientation;
            //ROS_INFO_STREAM("sphere 2 " << cylinder_pose.orientation);
            cylinder_hemisphere11.scale.x = 0.001 *_Sa_perp_max_.data; 
            cylinder_hemisphere11.scale.y = 0.001 *_Sa_perp_max_.data; 
            cylinder_hemisphere11.scale.z = 0.001 *_Sa_perp_max_.data; 
            cylinder_hemisphere11.color.r = 0.4f;
            cylinder_hemisphere11.color.g = 0.698f;
            cylinder_hemisphere11.color.b = 1.0f;
            cylinder_hemisphere11.color.a = 0.5;
            cylinder_hemisphere11.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere11);

        }

        if(_DERG_strategy_id_.data == 2){

            // Blue tube
            // cylinder1 between current pose and applied ref
            Point p21, p22;
            p21.x = obj_pose[i].position.x;
            p21.y = obj_pose[i].position.y;
            p21.z = obj_pose[i].position.z;
            p22.x = ref[0][i];
            p22.y = ref[1][i];
            p22.z = ref[2][i];
            cylinder1.header.frame_id = "/common_origin";
            cylinder1.id = i;
            cylinder1.header.stamp = ros::Time::now();
            cylinder1.ns = "cylinder1";
            cylinder1.type = visualization_msgs::Marker::CYLINDER; 
            cylinder1.action = visualization_msgs::Marker::ADD;
            CylinderOrientation(p21, p22, cylinder_pose, cylinder_height);
            cylinder1.pose = cylinder_pose;
            cylinder1.scale.x = 2*_Sa_perp_max_.data;           // x radius
            cylinder1.scale.y = 2*_Sa_perp_max_.data;           // y radius
            cylinder1.scale.z = cylinder_height;     // height
            cylinder1.color.r = 0.4f;
            cylinder1.color.g = 0.698f;
            cylinder1.color.b = 1.0f;
            cylinder1.color.a = 0.6;
            all_markers.markers.push_back(cylinder1);

            // Blue tube
            // sphere12 at cylinder ends
            cylinder_hemisphere12.header.frame_id = "/common_origin";
            cylinder_hemisphere12.header.stamp = ros::Time::now();
            cylinder_hemisphere12.ns = "cylinder_hemisphere12";
            cylinder_hemisphere12.id = i;
            cylinder_hemisphere12.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere12.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere12.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere12.pose.position.x = ref[0][i];
            cylinder_hemisphere12.pose.position.y = ref[1][i];
            cylinder_hemisphere12.pose.position.z = ref[2][i];
            cylinder_hemisphere12.pose.orientation = cylinder_pose.orientation;        
            cylinder_hemisphere12.scale.x = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere12.scale.y = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere12.scale.z = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere12.color.r = 0.4f;
            cylinder_hemisphere12.color.g = 0.698f;
            cylinder_hemisphere12.color.b = 1.0f;
            cylinder_hemisphere12.color.a = 0.5;
            cylinder_hemisphere12.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere12);

            // Blue tube
            // sphere11 at cylinder ends
            cylinder_hemisphere11.header.frame_id = "/common_origin";
            cylinder_hemisphere11.header.stamp = ros::Time::now();
            cylinder_hemisphere11.ns = "cylinder_hemisphere11";
            cylinder_hemisphere11.id = i;
            cylinder_hemisphere11.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere11.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere11.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere11.pose.position = obj_pose[i].position;
            CylinderOrientation(p22, p21, cylinder_pose, cylinder_height);
            cylinder_hemisphere11.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere11.scale.x = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere11.scale.y = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere11.scale.z = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere11.color.r = 0.4f;
            cylinder_hemisphere11.color.g = 0.698f;
            cylinder_hemisphere11.color.b = 1.0f;
            cylinder_hemisphere11.color.a = 0.5;
            cylinder_hemisphere11.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere11);

            // Transparent tube
            // cylinder2 between point link star and applied ref
            Point p23, p24;
            p23.x = pstar[i].position.x;
            p23.y = pstar[i].position.y;
            p23.z = pstar[i].position.z;
            p24.x = ref[0][i];
            p24.y = ref[1][i];
            p24.z = ref[2][i];
            cylinder2.header.frame_id = "/common_origin";
            cylinder2.id = i;
            cylinder2.header.stamp = ros::Time::now();
            cylinder2.ns = "cylinder2";
            cylinder2.type = visualization_msgs::Marker::CYLINDER; 
            cylinder2.action = visualization_msgs::Marker::ADD;
            CylinderOrientation(p23, p24, cylinder_pose, cylinder_height);
            cylinder2.pose = cylinder_pose;
            cylinder2.scale.x = 2*_Sa_perp_max_.data;   // x radius
            cylinder2.scale.y = 2*_Sa_perp_max_.data;   // y radius
            cylinder2.scale.z = cylinder_height;        // height
            cylinder2.color.r = 0.251f;
            cylinder2.color.g = 0.251f;
            cylinder2.color.b = 0.251f;
            cylinder2.color.a = 0.25;
            all_markers.markers.push_back(cylinder2);

            // Transparent tube
            // sphere22 at cylinder ends
            cylinder_hemisphere22.header.frame_id = "/common_origin";
            cylinder_hemisphere22.header.stamp = ros::Time::now();
            cylinder_hemisphere22.ns = "cylinder_hemisphere22";
            cylinder_hemisphere22.id = i;
            cylinder_hemisphere22.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere22.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere22.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere22.pose.position.x = ref[0][i];
            cylinder_hemisphere22.pose.position.y = ref[1][i];
            cylinder_hemisphere22.pose.position.z = ref[2][i];
            cylinder_hemisphere22.pose.orientation = cylinder_pose.orientation;      
            cylinder_hemisphere22.scale.x = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere22.scale.y = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere22.scale.z = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere22.color.r = 0.251f;
            cylinder_hemisphere22.color.g = 0.251f;
            cylinder_hemisphere22.color.b = 0.251f;
            cylinder_hemisphere22.color.a = 0.25;
            cylinder_hemisphere22.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere22);

            // Transparent tube
            // sphere21 at cylinder ends
            cylinder_hemisphere21.header.frame_id = "/common_origin";
            cylinder_hemisphere21.header.stamp = ros::Time::now();
            cylinder_hemisphere21.ns = "cylinder_hemisphere21";
            cylinder_hemisphere21.id = i;
            cylinder_hemisphere21.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere21.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere21.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere21.pose.position = pstar[i].position;
            CylinderOrientation(p24, p23, cylinder_pose, cylinder_height);
            cylinder_hemisphere21.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere21.scale.x = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere21.scale.y = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere21.scale.z = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere21.color.r = 0.251f;
            cylinder_hemisphere21.color.g = 0.251f;
            cylinder_hemisphere21.color.b = 0.251f;
            cylinder_hemisphere21.color.a = 0.25;
            cylinder_hemisphere21.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere21);

        }

        if(_DERG_strategy_id_.data == 3){

            // Blue tube
            // cylinder1 between current pose and applied ref
            Point p21, p22;
            p21.x = obj_pose[i].position.x;
            p21.y = obj_pose[i].position.y;
            p21.z = obj_pose[i].position.z;
            p22.x = ref[0][i];
            p22.y = ref[1][i];
            p22.z = ref[2][i];
            cylinder1.header.frame_id = "/common_origin";
            cylinder1.id = i;
            cylinder1.header.stamp = ros::Time::now();
            cylinder1.ns = "cylinder1";
            cylinder1.type = visualization_msgs::Marker::CYLINDER; 
            cylinder1.action = visualization_msgs::Marker::ADD;
            CylinderOrientation(p21, p22, cylinder_pose, cylinder_height);
            cylinder1.pose = cylinder_pose;
            cylinder1.scale.x = 2*_Sa_perp_max_.data;           // x radius
            cylinder1.scale.y = 2*_Sa_perp_max_.data;           // y radius
            cylinder1.scale.z = cylinder_height;     // height
            cylinder1.color.r = 0.4f;
            cylinder1.color.g = 0.698f;
            cylinder1.color.b = 1.0f;
            cylinder1.color.a = 0.6;
            all_markers.markers.push_back(cylinder1);

            // Blue tube
            // sphere12 at cylinder ends
            cylinder_hemisphere12.header.frame_id = "/common_origin";
            cylinder_hemisphere12.header.stamp = ros::Time::now();
            cylinder_hemisphere12.ns = "cylinder_hemisphere12";
            cylinder_hemisphere12.id = i;
            cylinder_hemisphere12.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere12.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere12.pose.position.x = ref[0][i];
            cylinder_hemisphere12.pose.position.y = ref[1][i];
            cylinder_hemisphere12.pose.position.z = ref[2][i];
            cylinder_hemisphere12.pose.orientation = cylinder_pose.orientation;      
            cylinder_hemisphere12.scale.x = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere12.scale.y = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere12.scale.z = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere12.color.r = 0.4f;
            cylinder_hemisphere12.color.g = 0.698f;
            cylinder_hemisphere12.color.b = 1.0f;
            cylinder_hemisphere12.color.a = 0.5;
            cylinder_hemisphere12.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere12);

            // Blue tube
            // sphere11 at cylinder ends
            cylinder_hemisphere11.header.frame_id = "/common_origin";
            cylinder_hemisphere11.header.stamp = ros::Time::now();
            cylinder_hemisphere11.ns = "cylinder_hemisphere11";
            cylinder_hemisphere11.id = i;
            cylinder_hemisphere11.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere11.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere11.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere11.pose.position = obj_pose[i].position;
            CylinderOrientation(p22, p21, cylinder_pose, cylinder_height);
            cylinder_hemisphere11.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere11.scale.x = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere11.scale.y = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere11.scale.z = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere11.color.r = 0.4f;
            cylinder_hemisphere11.color.g = 0.698f;
            cylinder_hemisphere11.color.b = 1.0f;
            cylinder_hemisphere11.color.a = 0.5;
            cylinder_hemisphere11.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere11);

            // Transparent tube
            // cylinder2 between point link star and applied ref
            Point p23, p24;
            p23.x = pstar[i].position.x;
            p23.y = pstar[i].position.y;
            p23.z = pstar[i].position.z;
            p24.x = ref[0][i];
            p24.y = ref[1][i];
            p24.z = ref[2][i];
            cylinder2.header.frame_id = "/common_origin";
            cylinder2.id = i;
            cylinder2.header.stamp = ros::Time::now();
            cylinder2.ns = "cylinder2";
            cylinder2.type = visualization_msgs::Marker::CYLINDER; 
            cylinder2.action = visualization_msgs::Marker::ADD;
            CylinderOrientation(p23, p24, cylinder_pose, cylinder_height);
            cylinder2.pose = cylinder_pose;
            cylinder2.scale.x = 2*_Sa_perp_max_.data;   // x radius
            cylinder2.scale.y = 2*_Sa_perp_max_.data;   // y radius
            cylinder2.scale.z = cylinder_height;        // height
            cylinder2.color.r = 0.251f;
            cylinder2.color.g = 0.251f;
            cylinder2.color.b = 0.251f;
            cylinder2.color.a = 0.25;
            all_markers.markers.push_back(cylinder2);

            // Transparent tube
            // sphere22 at cylinder ends
            cylinder_hemisphere22.header.frame_id = "/common_origin";
            cylinder_hemisphere22.header.stamp = ros::Time::now();
            cylinder_hemisphere22.ns = "cylinder_hemisphere22";
            cylinder_hemisphere22.id = i;
            cylinder_hemisphere22.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere22.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere22.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere22.pose.position.x = ref[0][i];
            cylinder_hemisphere22.pose.position.y = ref[1][i];
            cylinder_hemisphere22.pose.position.z = ref[2][i];
            cylinder_hemisphere22.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere22.scale.x = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere22.scale.y = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere22.scale.z = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere22.color.r = 0.251f;
            cylinder_hemisphere22.color.g = 0.251f;
            cylinder_hemisphere22.color.b = 0.251f;
            cylinder_hemisphere22.color.a = 0.25;
            cylinder_hemisphere22.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere22);

            // Transparent tube
            // sphere21 at cylinder ends
            cylinder_hemisphere21.header.frame_id = "/common_origin";
            cylinder_hemisphere21.header.stamp = ros::Time::now();
            cylinder_hemisphere21.ns = "cylinder_hemisphere21";
            cylinder_hemisphere21.id = i;
            cylinder_hemisphere21.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere21.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere21.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere21.pose.position = pstar[i].position;
            CylinderOrientation(p24, p23, cylinder_pose, cylinder_height);
            cylinder_hemisphere21.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere21.scale.x = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere21.scale.y = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere21.scale.z = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere21.color.r = 0.251f;
            cylinder_hemisphere21.color.g = 0.251f;
            cylinder_hemisphere21.color.b = 0.251f;
            cylinder_hemisphere21.color.a = 0.25;
            cylinder_hemisphere21.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere21);

            // Orange tube
            // cylinder3 between point link star and applied ref
            Point p25, p26;
            p25.x = obj_pose[i].position.x;
            p25.y = obj_pose[i].position.y;
            p25.z = obj_pose[i].position.z;
            p26.x = ref[0][i];
            p26.y = ref[1][i];
            p26.z = ref[2][i];
            cylinder3.header.frame_id = "/common_origin";
            cylinder3.id = i;
            cylinder3.header.stamp = ros::Time::now();
            cylinder3.ns = "cylinder3";
            cylinder3.type = visualization_msgs::Marker::CYLINDER; 
            cylinder3.action = visualization_msgs::Marker::ADD;
            CylinderOrientation(p25, p26, cylinder_pose, cylinder_height);
            cylinder3.pose = cylinder_pose;
            cylinder3.scale.x = 2*tube_min_radius.data;   // x radius
            cylinder3.scale.y = 2*tube_min_radius.data;   // y radius
            cylinder3.scale.z = cylinder_height;        // height
            cylinder3.color.r = 0.749f;
            cylinder3.color.g = 0.647f;
            cylinder3.color.b = 0.412f;
            cylinder3.color.a = 0.2;
            all_markers.markers.push_back(cylinder3);

            // Orange tube
            // sphere32 at cylinder ends
            cylinder_hemisphere32.header.frame_id = "/common_origin";
            cylinder_hemisphere32.header.stamp = ros::Time::now();
            cylinder_hemisphere32.ns = "cylinder_hemisphere32";
            cylinder_hemisphere32.id = i;
            cylinder_hemisphere32.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere32.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere32.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere32.pose.position.x = ref[0][i];
            cylinder_hemisphere32.pose.position.y = ref[1][i];
            cylinder_hemisphere32.pose.position.z = ref[2][i];
            cylinder_hemisphere32.pose.orientation = cylinder_pose.orientation;     
            cylinder_hemisphere32.scale.x = 0.001*tube_min_radius.data; 
            cylinder_hemisphere32.scale.y = 0.001*tube_min_radius.data; 
            cylinder_hemisphere32.scale.z = 0.001*tube_min_radius.data; 
            cylinder_hemisphere32.color.r = 0.749f;
            cylinder_hemisphere32.color.g = 0.647f;
            cylinder_hemisphere32.color.b = 0.412f;
            cylinder_hemisphere32.color.a = 0.2;
            cylinder_hemisphere32.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere32);
            // Orange tube
            
            // sphere31 at cylinder ends
            cylinder_hemisphere31.header.frame_id = "/common_origin";
            cylinder_hemisphere31.header.stamp = ros::Time::now();
            cylinder_hemisphere31.ns = "cylinder_hemisphere31";
            cylinder_hemisphere31.id = i;
            cylinder_hemisphere31.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere31.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere31.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere31.pose.position = obj_pose[i].position;
            CylinderOrientation(p23, p24, cylinder_pose, cylinder_height);
            cylinder_hemisphere31.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere31.scale.x = 0.001*tube_min_radius.data; 
            cylinder_hemisphere31.scale.y = 0.001*tube_min_radius.data; 
            cylinder_hemisphere31.scale.z = 0.001*tube_min_radius.data; 
            cylinder_hemisphere31.color.r = 0.749f;
            cylinder_hemisphere31.color.g = 0.647f;
            cylinder_hemisphere31.color.b = 0.412f;
            cylinder_hemisphere31.color.a = 0.2;
            cylinder_hemisphere31.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere31);

        }

        if(_DERG_strategy_id_.data == 4){

            // Blue tube
            // cylinder1 between current pose and applied ref
            Point p21, p22;
            p21.x = obj_pose[i].position.x;
            p21.y = obj_pose[i].position.y;
            p21.z = obj_pose[i].position.z;
            p22.x = ref[0][i];
            p22.y = ref[1][i];
            p22.z = ref[2][i];
            cylinder1.header.frame_id = "/common_origin";
            cylinder1.id = i;
            cylinder1.header.stamp = ros::Time::now();
            cylinder1.ns = "cylinder1";
            cylinder1.type = visualization_msgs::Marker::CYLINDER; 
            cylinder1.action = visualization_msgs::Marker::ADD;
            CylinderOrientation(p21, p22, cylinder_pose, cylinder_height);
            cylinder1.pose = cylinder_pose;
            cylinder1.scale.x = 2*_Sa_perp_max_.data;           // x radius
            cylinder1.scale.y = 2*_Sa_perp_max_.data;           // y radius
            cylinder1.scale.z = cylinder_height;     // height
            cylinder1.color.r = 0.4f;
            cylinder1.color.g = 0.698f;
            cylinder1.color.b = 1.0f;
            cylinder1.color.a = 0.6;
            all_markers.markers.push_back(cylinder1);

            // Blue tube
            // sphere12 at cylinder ends
            cylinder_hemisphere12.header.frame_id = "/common_origin";
            cylinder_hemisphere12.header.stamp = ros::Time::now();
            cylinder_hemisphere12.ns = "cylinder_hemisphere12";
            cylinder_hemisphere12.id = i;
            cylinder_hemisphere12.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere12.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere12.pose.position.x = ref[0][i];
            cylinder_hemisphere12.pose.position.y = ref[1][i];
            cylinder_hemisphere12.pose.position.z = ref[2][i];
            cylinder_hemisphere12.pose.orientation = cylinder_pose.orientation;      
            cylinder_hemisphere12.scale.x = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere12.scale.y = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere12.scale.z = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere12.color.r = 0.4f;
            cylinder_hemisphere12.color.g = 0.698f;
            cylinder_hemisphere12.color.b = 1.0f;
            cylinder_hemisphere12.color.a = 0.5;
            cylinder_hemisphere12.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere12);

            // Blue tube
            // sphere11 at cylinder ends
            cylinder_hemisphere11.header.frame_id = "/common_origin";
            cylinder_hemisphere11.header.stamp = ros::Time::now();
            cylinder_hemisphere11.ns = "cylinder_hemisphere11";
            cylinder_hemisphere11.id = i;
            cylinder_hemisphere11.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere11.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere11.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere11.pose.position = obj_pose[i].position;
            CylinderOrientation(p22, p21, cylinder_pose, cylinder_height);
            cylinder_hemisphere11.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere11.scale.x = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere11.scale.y = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere11.scale.z = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere11.color.r = 0.4f;
            cylinder_hemisphere11.color.g = 0.698f;
            cylinder_hemisphere11.color.b = 1.0f;
            cylinder_hemisphere11.color.a = 0.5;
            cylinder_hemisphere11.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere11);

            // Transparent tube
            // cylinder2 between point link star and applied ref
            Point p23, p24;
            p23.x = pstar[i].position.x;
            p23.y = pstar[i].position.y;
            p23.z = pstar[i].position.z;
            p24.x = ref[0][i];
            p24.y = ref[1][i];
            p24.z = ref[2][i];
            cylinder2.header.frame_id = "/common_origin";
            cylinder2.id = i;
            cylinder2.header.stamp = ros::Time::now();
            cylinder2.ns = "cylinder2";
            cylinder2.type = visualization_msgs::Marker::CYLINDER; 
            cylinder2.action = visualization_msgs::Marker::ADD;
            CylinderOrientation(p23, p24, cylinder_pose, cylinder_height);
            cylinder2.pose = cylinder_pose;
            cylinder2.scale.x = 2*_Sa_perp_max_.data;   // x radius
            cylinder2.scale.y = 2*_Sa_perp_max_.data;   // y radius
            cylinder2.scale.z = cylinder_height;        // height
            cylinder2.color.r = 0.251f;
            cylinder2.color.g = 0.251f;
            cylinder2.color.b = 0.251f;
            cylinder2.color.a = 0.25;
            all_markers.markers.push_back(cylinder2);

            // Transparent tube
            // sphere22 at cylinder ends
            cylinder_hemisphere22.header.frame_id = "/common_origin";
            cylinder_hemisphere22.header.stamp = ros::Time::now();
            cylinder_hemisphere22.ns = "cylinder_hemisphere22";
            cylinder_hemisphere22.id = i;
            cylinder_hemisphere22.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere22.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere22.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere22.pose.position.x = ref[0][i];
            cylinder_hemisphere22.pose.position.y = ref[1][i];
            cylinder_hemisphere22.pose.position.z = ref[2][i];
            cylinder_hemisphere22.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere22.scale.x = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere22.scale.y = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere22.scale.z = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere22.color.r = 0.251f;
            cylinder_hemisphere22.color.g = 0.251f;
            cylinder_hemisphere22.color.b = 0.251f;
            cylinder_hemisphere22.color.a = 0.25;
            cylinder_hemisphere22.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere22);

            // Transparent tube
            // sphere21 at cylinder ends
            cylinder_hemisphere21.header.frame_id = "/common_origin";
            cylinder_hemisphere21.header.stamp = ros::Time::now();
            cylinder_hemisphere21.ns = "cylinder_hemisphere21";
            cylinder_hemisphere21.id = i;
            cylinder_hemisphere21.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere21.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere21.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere21.pose.position = pstar[i].position;
            CylinderOrientation(p24, p23, cylinder_pose, cylinder_height);
            cylinder_hemisphere21.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere21.scale.x = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere21.scale.y = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere21.scale.z = 0.001*_Sa_perp_max_.data; 
            cylinder_hemisphere21.color.r = 0.251f;
            cylinder_hemisphere21.color.g = 0.251f;
            cylinder_hemisphere21.color.b = 0.251f;
            cylinder_hemisphere21.color.a = 0.25;
            cylinder_hemisphere21.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere21);

            // Orange tube
            // cylinder3 between point link star and applied ref
            Point p25, p26;
            p25.x = p1[i].position.x;
            p25.y = p1[i].position.y;
            p25.z = p1[i].position.z;
            p26.x = p0[i].position.x;
            p26.y = p0[i].position.y;
            p26.z = p0[i].position.z;
            cylinder3.header.frame_id = "/common_origin";
            cylinder3.id = i;
            cylinder3.header.stamp = ros::Time::now();
            cylinder3.ns = "cylinder3";
            cylinder3.type = visualization_msgs::Marker::CYLINDER; 
            cylinder3.action = visualization_msgs::Marker::ADD;
            CylinderOrientation(p25, p26, cylinder_pose, cylinder_height);
            cylinder3.pose = cylinder_pose;
            cylinder3.scale.x = 2*tube_min_radius.data;   // x radius
            cylinder3.scale.y = 2*tube_min_radius.data;   // y radius
            cylinder3.scale.z = cylinder_height;        // height
            cylinder3.color.r = 0.749f;
            cylinder3.color.g = 0.647f;
            cylinder3.color.b = 0.412f;
            cylinder3.color.a = 0.5;
            all_markers.markers.push_back(cylinder3);

            // Orange tube
            // Red hemisphere32 at cylinder ends
            cylinder_hemisphere32.header.frame_id = "/common_origin";
            cylinder_hemisphere32.header.stamp = ros::Time::now();
            cylinder_hemisphere32.ns = "cylinder_hemisphere32";
            cylinder_hemisphere32.id = i;
            cylinder_hemisphere32.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere32.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere32.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere32.pose.position = p0[i].position;
            cylinder_hemisphere32.pose.orientation = cylinder_pose.orientation;     
            cylinder_hemisphere32.scale.x = 0.001*tube_min_radius.data; 
            cylinder_hemisphere32.scale.y = 0.001*tube_min_radius.data; 
            cylinder_hemisphere32.scale.z = 0.001*tube_min_radius.data; 
            cylinder_hemisphere32.color.r = 0.8f;
            cylinder_hemisphere32.color.g = 0.0f;
            cylinder_hemisphere32.color.b = 0.0f;
            cylinder_hemisphere32.color.a = 0.5;
            cylinder_hemisphere32.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere32);

            // Orange tube
            // Red hemisphere31 at cylinder ends
            cylinder_hemisphere31.header.frame_id = "/common_origin";
            cylinder_hemisphere31.header.stamp = ros::Time::now();
            cylinder_hemisphere31.ns = "cylinder_hemisphere31";
            cylinder_hemisphere31.id = i;
            cylinder_hemisphere31.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere31.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere31.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere31.pose.position = p1[i].position;
            CylinderOrientation(p24, p23, cylinder_pose, cylinder_height);
            cylinder_hemisphere31.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere31.scale.x = 0.001*tube_min_radius.data; 
            cylinder_hemisphere31.scale.y = 0.001*tube_min_radius.data; 
            cylinder_hemisphere31.scale.z = 0.001*tube_min_radius.data; 
            cylinder_hemisphere31.color.r = 0.8f;
            cylinder_hemisphere31.color.g = 0.0f;
            cylinder_hemisphere31.color.b = 0.0f;
            cylinder_hemisphere31.color.a = 0.5;
            cylinder_hemisphere31.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere31);

        }

    }

    // red line betweetn both current uav pose
    line.header.frame_id = "/common_origin";
    line.id = 2;
    line.header.stamp = ros::Time::now();
    line.ns = "line";
    line.type = visualization_msgs::Marker::LINE_STRIP; 
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 0.0;
    line.scale.x = 0.05; // width
    line.color.r = 1.0f;
    line.color.g = 0.0f;
    line.color.b = 0.0f;
    line.color.a = 1.0;
    all_markers.markers.push_back(line);

    //trajectory_pub.publish(trajectory_array);
    // all_markers.markers.push_back(trajectory_array);
    marker_pub.publish(all_markers);

}

int main(int argc, char **argv){
    
    // Initialization
    // ^^^^^^^^^^^^^^

    // init node
    ros::init(argc, argv,"current_pose_sphere");
    ros::NodeHandle n;
    ros::Rate r(30);

    // Subscribers and publishers
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^

    // create subscriber for active UAV list
    diagnostics_sub = n.subscribe("mrs_drone_spawner/diagnostics", 1, DiagnosticsCallback);
    DERG_strategy_id_sub = n.subscribe("uav1/control_manager/dergbryan_tracker/derg_strategy_id", 1, DERGStrategyIdCallback);
    Sa_max_sub = n.subscribe("uav1/control_manager/dergbryan_tracker/sa_max", 1, SaMaxCallback);
    Sa_perp_max_sub = n.subscribe("uav1/control_manager/dergbryan_tracker/sa_perp_max", 1, SaPerpMaxCallback);
    tube_min_radius_sub = n.subscribe("uav1/control_manager/dergbryan_tracker/tube_min_radius", 1, TubeMinRadiusCallback);

    while(!test1){
        ros::spinOnce();
        r.sleep();

    }
    
    // create subscribers
    std::vector<boost::function<void (const geometry_msgs::PoseArray::ConstPtr&)>> f1;
    f1.resize(MAX_UAV_NUMBER);
    std::vector<boost::function<void (const mrs_msgs::FutureTrajectory::ConstPtr&)>> f2;
    f2.resize(MAX_UAV_NUMBER);
    std::vector<boost::function<void (const geometry_msgs::Pose::ConstPtr&)>> f3;
    f3.resize(MAX_UAV_NUMBER);
    std::vector<boost::function<void (const mrs_msgs::FutureTrajectory::ConstPtr&)>> f4;
    f4.resize(MAX_UAV_NUMBER);
    std::vector<boost::function<void (const geometry_msgs::Pose::ConstPtr&)>> f5;
    f5.resize(MAX_UAV_NUMBER);
    std::vector<boost::function<void (const geometry_msgs::Pose::ConstPtr&)>> f6;
    f6.resize(MAX_UAV_NUMBER);

    for(int i=0; i<diagnostics.active_vehicles.size(); i++){

        // create subscribers for current uav pose
        f1[i] = boost::bind(CurrentPoseCallback, _1, i+1);
        uav_current_pose_sub[i] = n.subscribe(diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/custom_predicted_poses", 10, f1[i]);

        // create subscribers for uav applied ref
        f2[i] = boost::bind(AppliedRefCallback, _1, i+1);
        uav_applied_ref_sub[i] = n.subscribe(diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/uav_applied_ref", 10, f2[i]);

        // create subscribers for uav point link star
        f3[i] = boost::bind(PointLinkStarCallback, _1, i+1);
        point_link_star_sub[i] = n.subscribe(diagnostics.active_vehicles[i] +"/control_manager/dergbryan_tracker/point_link_star", 10, f3[i]);

        // create subscribers for uav predicted trajectory
        f4[i] = boost::bind(PredictedTrajectoryCallback, _1, i+1);
        predicted_trajectory_sub[i] = n.subscribe(diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/predicted_trajectory", 10, f4[i]);

        // create subscribers for uav point link p0
        f5[i] = boost::bind(PointLinkP0Callback, _1, i+1);
        point_link_p0_sub[i] = n.subscribe(diagnostics.active_vehicles[i] +"/control_manager/dergbryan_tracker/point_link_p0", 10, f5[i]);

        // create subscribers for uav point link p1
        f6[i] = boost::bind(PointLinkP1Callback, _1, i+1);
        point_link_p1_sub[i] = n.subscribe(diagnostics.active_vehicles[i] +"/control_manager/dergbryan_tracker/point_link_p1", 10, f6[i]);
    }   

    while(!test2){
        ros::spinOnce();
        r.sleep();

    }

    // create one publisher for all the all the markers
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
    //trajectory_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker2", 10);

    // Display
    // ^^^^^^^

    while(ros::ok){
        PublishMarkers(uav_current_poses, uav_applied_ref, point_link_stars, point_link_p0s, point_link_p1s, predicted_trajectories);                   
        ros::spinOnce();
        r.sleep();

    }

    return 0;

}