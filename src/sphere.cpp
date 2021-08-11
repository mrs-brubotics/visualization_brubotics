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


double Ra = 0.35;
double Sa = 1.5;
double Sa_perp = 0.2;
double Sa_long = Sa-Sa_perp;
bool test1 = false;
std::array<std::array<double, 2>, 3> uav_applied_ref;


// Publishers and subscribers
ros::Publisher marker_pub;
ros::Subscriber diagnostics_sub;
ros::Subscriber DERG_strategy_id_sub;
ros::Subscriber Sa_max_sub;
ros::Subscriber Sa_perp_max_sub;
ros::Subscriber tube_min_radius_sub;
std::vector<ros::Subscriber> uav_current_pose_sub(MAX_UAV_NUMBER);
std::vector<ros::Subscriber> uav_applied_ref_sub(MAX_UAV_NUMBER);
std::vector<ros::Subscriber> point_link_star_sub(MAX_UAV_NUMBER);


// Messages
mrs_msgs::SpawnerDiagnostics diagnostics;
mrs_msgs::FutureTrajectory uav_applied_ref_traj;
mrs_msgs::FuturePoint uav_applied_ref_point;
geometry_msgs::PoseArray uav_current_pose;
geometry_msgs::Pose cylinder_pose;
geometry_msgs::Pose point_link_star;
std_msgs::Int32 _DERG_strategy_id_;
std_msgs::Int32 _Sa_max_;
std_msgs::Int32 _Sa_perp_max_;
std_msgs::Float32 tube_min_radius;
std::vector<geometry_msgs::Pose> uav_current_poses(MAX_UAV_NUMBER);
std::vector<geometry_msgs::Pose> point_link_stars(MAX_UAV_NUMBER);


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
void AppliedRefCallback(const mrs_msgs::FutureTrajectory::ConstPtr& msg);
void PointLinkStarCallback(const geometry_msgs::Pose::ConstPtr& msg, int uav_number);
void SaMaxCallback(const std_msgs::Int32::ConstPtr& msg);
void SaPerpMaxCallback(const std_msgs::Int32::ConstPtr& msg);
void TubeMinRadiusCallback(const std_msgs::Float32::ConstPtr& msg);
void DERGStrategyIdCallback(const std_msgs::Int32::ConstPtr& msg);
double getDistance(const Point& p1, const Point& p2);
Point getMiddle(const Point& pt1, const Point& pt2);
void CylinderOrientation(const Point &p1,const Point &p2, geometry_msgs::Pose& cylinder_pose, double& cylinder_height);
void PublishMarkers(const std::vector<geometry_msgs::Pose>& obj_pose, std::array<std::array<double, 2>, 3> ref, const std::vector<geometry_msgs::Pose>& pstar);


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
    //ROS_INFO_STREAM("UAV " << uav_number << ": x = " << uav_applied_ref[uav_number-1][0] << ", y = " << uav_applied_ref[uav_number-1][1] << ", z = " << uav_applied_ref[uav_number-1][2]);
}

void PointLinkStarCallback(const geometry_msgs::Pose::ConstPtr& msg, int uav_number){
    point_link_star.position = msg -> position;
    point_link_stars[uav_number-1].position = point_link_star.position;
    //ROS_INFO_STREAM("UAV" << uav_number << ": " << point_link_stars[uav_number-1].position);
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
    ROS_INFO_STREAM("tube_min_radius = " << tube_min_radius.data);
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

void PublishMarkers(const std::vector<geometry_msgs::Pose>& obj_pose, std::array<std::array<double, 2>, 3> ref, const std::vector<geometry_msgs::Pose>& pstar){
    visualization_msgs::Marker current_pose_sphere;
    visualization_msgs::Marker error_sphere;
    visualization_msgs::Marker applied_ref_sphere;
    visualization_msgs::Marker cylinder1;
    visualization_msgs::Marker cylinder2;
    visualization_msgs::Marker cylinder3;
    visualization_msgs::Marker cylinder_sphere11;
    visualization_msgs::Marker cylinder_sphere12;
    visualization_msgs::Marker cylinder_sphere21;
    visualization_msgs::Marker cylinder_sphere22;
    visualization_msgs::Marker cylinder_sphere31;
    visualization_msgs::Marker cylinder_sphere32;
    visualization_msgs::Marker line;
    visualization_msgs::MarkerArray all_markers;
    geometry_msgs::Pose cylinder_pose;
    geometry_msgs::Point p;
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
        error_sphere.color.a = 0.33;
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
            cylinder_sphere12.header.frame_id = "/common_origin";
            cylinder_sphere12.header.stamp = ros::Time::now();
            cylinder_sphere12.ns = "cylinder_sphere12";
            cylinder_sphere12.id = i;
            cylinder_sphere12.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_sphere12.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_sphere12.action = visualization_msgs::Marker::ADD;
            cylinder_sphere12.pose.position.x = ref[0][i];
            cylinder_sphere12.pose.position.y = ref[1][i];
            cylinder_sphere12.pose.position.z = ref[2][i];
            cylinder_sphere12.pose.orientation.x = cylinder_pose.orientation.x;
            cylinder_sphere12.pose.orientation.y = cylinder_pose.orientation.y;
            cylinder_sphere12.pose.orientation.z = cylinder_pose.orientation.z;
            cylinder_sphere12.pose.orientation.w = cylinder_pose.orientation.w; 
            ROS_INFO_STREAM("sphere 1 " << cylinder_pose.orientation);      
            cylinder_sphere12.scale.x = 0.001 * _Sa_perp_max_.data; 
            cylinder_sphere12.scale.y = 0.001 * _Sa_perp_max_.data; 
            cylinder_sphere12.scale.z = 0.001 * _Sa_perp_max_.data; 
            cylinder_sphere12.color.r = 0.4f;
            cylinder_sphere12.color.g = 0.698f;
            cylinder_sphere12.color.b = 1.0f;
            cylinder_sphere12.color.a = 0.5;
            cylinder_sphere12.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_sphere12);

            // Blue tube
            // sphere11 at cylinder ends
            cylinder_sphere11.header.frame_id = "/common_origin";
            cylinder_sphere11.header.stamp = ros::Time::now();
            cylinder_sphere11.ns = "cylinder_sphere11";
            cylinder_sphere11.id = i;
            cylinder_sphere11.type = visualization_msgs::Marker::MESH_RESOURCE;
            cylinder_sphere11.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_sphere11.action = visualization_msgs::Marker::ADD;
            cylinder_sphere11.pose.position.x = pstar[i].position.x;
            cylinder_sphere11.pose.position.y = pstar[i].position.y;
            cylinder_sphere11.pose.position.z = pstar[i].position.z;
            CylinderOrientation(p2, p1, cylinder_pose, cylinder_height);
            cylinder_sphere11.pose.orientation.x = cylinder_pose.orientation.x;
            cylinder_sphere11.pose.orientation.y = cylinder_pose.orientation.y;
            cylinder_sphere11.pose.orientation.z = cylinder_pose.orientation.z;
            cylinder_sphere11.pose.orientation.w = cylinder_pose.orientation.w;
            ROS_INFO_STREAM("sphere 2 " << cylinder_pose.orientation);
            cylinder_sphere11.scale.x = 0.001 *_Sa_perp_max_.data; 
            cylinder_sphere11.scale.y = 0.001 *_Sa_perp_max_.data; 
            cylinder_sphere11.scale.z = 0.001 *_Sa_perp_max_.data; 
            cylinder_sphere11.color.r = 0.4f;
            cylinder_sphere11.color.g = 0.698f;
            cylinder_sphere11.color.b = 1.0f;
            cylinder_sphere11.color.a = 0.5;
            cylinder_sphere11.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_sphere11);

            
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
            // sphere11 at cylinder ends
            cylinder_sphere11.header.frame_id = "/common_origin";
            cylinder_sphere11.header.stamp = ros::Time::now();
            cylinder_sphere11.ns = "cylinder_sphere11";
            cylinder_sphere11.id = i;
            cylinder_sphere11.type = visualization_msgs::Marker::SPHERE; 
            cylinder_sphere11.action = visualization_msgs::Marker::ADD;
            cylinder_sphere11.pose = obj_pose[i];
            cylinder_sphere11.scale.x = 2*_Sa_perp_max_.data; 
            cylinder_sphere11.scale.y = 2*_Sa_perp_max_.data; 
            cylinder_sphere11.scale.z = 2*_Sa_perp_max_.data; 
            cylinder_sphere11.color.r = 0.4f;
            cylinder_sphere11.color.g = 0.698f;
            cylinder_sphere11.color.b = 1.0f;
            cylinder_sphere11.color.a = 0.5;
            cylinder_sphere11.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_sphere11);

            // Blue tube
            // sphere12 at cylinder ends
            cylinder_sphere12.header.frame_id = "/common_origin";
            cylinder_sphere12.header.stamp = ros::Time::now();
            cylinder_sphere12.ns = "cylinder_sphere12";
            cylinder_sphere12.id = i;
            cylinder_sphere12.type = visualization_msgs::Marker::SPHERE; 
            cylinder_sphere12.action = visualization_msgs::Marker::ADD;
            cylinder_sphere12.pose.position.x = ref[0][i];
            cylinder_sphere12.pose.position.y = ref[1][i];
            cylinder_sphere12.pose.position.z = ref[2][i];
            cylinder_sphere12.pose.orientation.x = 0;
            cylinder_sphere12.pose.orientation.y = 0;
            cylinder_sphere12.pose.orientation.z = 0;
            cylinder_sphere12.pose.orientation.w = 0.0;        
            cylinder_sphere12.scale.x = 2*_Sa_perp_max_.data; 
            cylinder_sphere12.scale.y = 2*_Sa_perp_max_.data; 
            cylinder_sphere12.scale.z = 2*_Sa_perp_max_.data; 
            cylinder_sphere12.color.r = 0.4f;
            cylinder_sphere12.color.g = 0.698f;
            cylinder_sphere12.color.b = 1.0f;
            cylinder_sphere12.color.a = 0.5;
            cylinder_sphere12.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_sphere12);

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
            // sphere21 at cylinder ends
            cylinder_sphere21.header.frame_id = "/common_origin";
            cylinder_sphere21.header.stamp = ros::Time::now();
            cylinder_sphere21.ns = "cylinder_sphere21";
            cylinder_sphere21.id = i;
            cylinder_sphere21.type = visualization_msgs::Marker::SPHERE; 
            cylinder_sphere21.action = visualization_msgs::Marker::ADD;
            cylinder_sphere21.pose = pstar[i];
            cylinder_sphere21.scale.x = 2*_Sa_perp_max_.data; 
            cylinder_sphere21.scale.y = 2*_Sa_perp_max_.data; 
            cylinder_sphere21.scale.z = 2*_Sa_perp_max_.data; 
            cylinder_sphere21.color.r = 0.251f;
            cylinder_sphere21.color.g = 0.251f;
            cylinder_sphere21.color.b = 0.251f;
            cylinder_sphere21.color.a = 0.25;
            cylinder_sphere21.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_sphere21);

            // Transparent tube
            // sphere22 at cylinder ends
            cylinder_sphere22.header.frame_id = "/common_origin";
            cylinder_sphere22.header.stamp = ros::Time::now();
            cylinder_sphere22.ns = "cylinder_sphere22";
            cylinder_sphere22.id = i;
            cylinder_sphere22.type = visualization_msgs::Marker::SPHERE; 
            cylinder_sphere22.action = visualization_msgs::Marker::ADD;
            cylinder_sphere22.pose.position.x = ref[0][i];
            cylinder_sphere22.pose.position.y = ref[1][i];
            cylinder_sphere22.pose.position.z = ref[2][i];
            cylinder_sphere22.pose.orientation.x = 0;
            cylinder_sphere22.pose.orientation.y = 0;
            cylinder_sphere22.pose.orientation.z = 0;
            cylinder_sphere22.pose.orientation.w = 0.0;        
            cylinder_sphere22.scale.x = 2*_Sa_perp_max_.data; 
            cylinder_sphere22.scale.y = 2*_Sa_perp_max_.data; 
            cylinder_sphere22.scale.z = 2*_Sa_perp_max_.data; 
            cylinder_sphere22.color.r = 0.251f;
            cylinder_sphere22.color.g = 0.251f;
            cylinder_sphere22.color.b = 0.251f;
            cylinder_sphere22.color.a = 0.25;
            cylinder_sphere22.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_sphere22);
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
            // sphere11 at cylinder ends
            cylinder_sphere11.header.frame_id = "/common_origin";
            cylinder_sphere11.header.stamp = ros::Time::now();
            cylinder_sphere11.ns = "cylinder_sphere11";
            cylinder_sphere11.id = i;
            cylinder_sphere11.type = visualization_msgs::Marker::SPHERE; 
            cylinder_sphere11.action = visualization_msgs::Marker::ADD;
            cylinder_sphere11.pose = obj_pose[i];
            cylinder_sphere11.scale.x = 2*_Sa_perp_max_.data; 
            cylinder_sphere11.scale.y = 2*_Sa_perp_max_.data; 
            cylinder_sphere11.scale.z = 2*_Sa_perp_max_.data; 
            cylinder_sphere11.color.r = 0.4f;
            cylinder_sphere11.color.g = 0.698f;
            cylinder_sphere11.color.b = 1.0f;
            cylinder_sphere11.color.a = 0.5;
            cylinder_sphere11.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_sphere11);

            // Blue tube
            // sphere12 at cylinder ends
            cylinder_sphere12.header.frame_id = "/common_origin";
            cylinder_sphere12.header.stamp = ros::Time::now();
            cylinder_sphere12.ns = "cylinder_sphere12";
            cylinder_sphere12.id = i;
            cylinder_sphere12.type = visualization_msgs::Marker::SPHERE; 
            cylinder_sphere12.action = visualization_msgs::Marker::ADD;
            cylinder_sphere12.pose.position.x = ref[0][i];
            cylinder_sphere12.pose.position.y = ref[1][i];
            cylinder_sphere12.pose.position.z = ref[2][i];
            cylinder_sphere12.pose.orientation.x = 0;
            cylinder_sphere12.pose.orientation.y = 0;
            cylinder_sphere12.pose.orientation.z = 0;
            cylinder_sphere12.pose.orientation.w = 0.0;        
            cylinder_sphere12.scale.x = 2*_Sa_perp_max_.data; 
            cylinder_sphere12.scale.y = 2*_Sa_perp_max_.data; 
            cylinder_sphere12.scale.z = 2*_Sa_perp_max_.data; 
            cylinder_sphere12.color.r = 0.4f;
            cylinder_sphere12.color.g = 0.698f;
            cylinder_sphere12.color.b = 1.0f;
            cylinder_sphere12.color.a = 0.5;
            cylinder_sphere12.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_sphere12);

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
            // sphere21 at cylinder ends
            cylinder_sphere21.header.frame_id = "/common_origin";
            cylinder_sphere21.header.stamp = ros::Time::now();
            cylinder_sphere21.ns = "cylinder_sphere21";
            cylinder_sphere21.id = i;
            cylinder_sphere21.type = visualization_msgs::Marker::SPHERE; 
            cylinder_sphere21.action = visualization_msgs::Marker::ADD;
            cylinder_sphere21.pose = pstar[i];
            cylinder_sphere21.scale.x = 2*_Sa_perp_max_.data; 
            cylinder_sphere21.scale.y = 2*_Sa_perp_max_.data; 
            cylinder_sphere21.scale.z = 2*_Sa_perp_max_.data; 
            cylinder_sphere21.color.r = 0.251f;
            cylinder_sphere21.color.g = 0.251f;
            cylinder_sphere21.color.b = 0.251f;
            cylinder_sphere21.color.a = 0.25;
            cylinder_sphere21.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_sphere21);

            // Transparent tube
            // sphere22 at cylinder ends
            cylinder_sphere22.header.frame_id = "/common_origin";
            cylinder_sphere22.header.stamp = ros::Time::now();
            cylinder_sphere22.ns = "cylinder_sphere22";
            cylinder_sphere22.id = i;
            cylinder_sphere22.type = visualization_msgs::Marker::SPHERE; 
            cylinder_sphere22.action = visualization_msgs::Marker::ADD;
            cylinder_sphere22.pose.position.x = ref[0][i];
            cylinder_sphere22.pose.position.y = ref[1][i];
            cylinder_sphere22.pose.position.z = ref[2][i];
            cylinder_sphere22.pose.orientation.x = 0;
            cylinder_sphere22.pose.orientation.y = 0;
            cylinder_sphere22.pose.orientation.z = 0;
            cylinder_sphere22.pose.orientation.w = 0.0;        
            cylinder_sphere22.scale.x = 2*_Sa_perp_max_.data; 
            cylinder_sphere22.scale.y = 2*_Sa_perp_max_.data; 
            cylinder_sphere22.scale.z = 2*_Sa_perp_max_.data; 
            cylinder_sphere22.color.r = 0.251f;
            cylinder_sphere22.color.g = 0.251f;
            cylinder_sphere22.color.b = 0.251f;
            cylinder_sphere22.color.a = 0.25;
            cylinder_sphere22.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_sphere22);

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
            // sphere31 at cylinder ends
            cylinder_sphere31.header.frame_id = "/common_origin";
            cylinder_sphere31.header.stamp = ros::Time::now();
            cylinder_sphere31.ns = "cylinder_sphere31";
            cylinder_sphere31.id = i;
            cylinder_sphere31.type = visualization_msgs::Marker::SPHERE; 
            cylinder_sphere31.action = visualization_msgs::Marker::ADD;
            cylinder_sphere31.pose = obj_pose[i];
            cylinder_sphere31.scale.x = 2*tube_min_radius.data; 
            cylinder_sphere31.scale.y = 2*tube_min_radius.data; 
            cylinder_sphere31.scale.z = 2*tube_min_radius.data; 
            cylinder_sphere31.color.r = 0.749f;
            cylinder_sphere31.color.g = 0.647f;
            cylinder_sphere31.color.b = 0.412f;
            cylinder_sphere31.color.a = 0.2;
            cylinder_sphere31.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_sphere31);

            // Orange tube
            // sphere32 at cylinder ends
            cylinder_sphere32.header.frame_id = "/common_origin";
            cylinder_sphere32.header.stamp = ros::Time::now();
            cylinder_sphere32.ns = "cylinder_sphere32";
            cylinder_sphere32.id = i;
            cylinder_sphere32.type = visualization_msgs::Marker::SPHERE; 
            cylinder_sphere32.action = visualization_msgs::Marker::ADD;
            cylinder_sphere32.pose.position.x = ref[0][i];
            cylinder_sphere32.pose.position.y = ref[1][i];
            cylinder_sphere32.pose.position.z = ref[2][i];
            cylinder_sphere32.pose.orientation.x = 0;
            cylinder_sphere32.pose.orientation.y = 0;
            cylinder_sphere32.pose.orientation.z = 0;
            cylinder_sphere32.pose.orientation.w = 0.0;        
            cylinder_sphere32.scale.x = 2*tube_min_radius.data; 
            cylinder_sphere32.scale.y = 2*tube_min_radius.data; 
            cylinder_sphere32.scale.z = 2*tube_min_radius.data; 
            cylinder_sphere32.color.r = 0.749f;
            cylinder_sphere32.color.g = 0.647f;
            cylinder_sphere32.color.b = 0.412f;
            cylinder_sphere32.color.a = 0.2;
            cylinder_sphere32.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_sphere32);
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

    marker_pub.publish(all_markers);
}

int main(int argc, char **argv){
    
    // Initialization
    // ^^^^^^^^^^^^^^

    // init node
    ros::init(argc, argv,"current_pose_sphere");
    ros::NodeHandle n;
    ros::Rate r(30);

    // Subscribers and publishers //
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

    for(int i=0; i<diagnostics.active_vehicles.size(); i++){
        // create subscriber for current uav pose
        f1[i] = boost::bind(CurrentPoseCallback, _1, i+1);
        uav_current_pose_sub[i] = n.subscribe(diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/custom_predicted_poses", 10, f1[i]);

        // create subscriber for uav applied ref
        f2[i] = boost::bind(AppliedRefCallback, _1, i+1);
        uav_applied_ref_sub[i] = n.subscribe(diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/uav_applied_ref", 10, f2[i]);
        //ROS_INFO_STREAM("yes2");
        // create subscriber for uav point link star
        f3[i] = boost::bind(PointLinkStarCallback, _1, i+1);
        point_link_star_sub[i] = n.subscribe(diagnostics.active_vehicles[i] +"/control_manager/dergbryan_tracker/point_link_star", 10, f3[i]);
    }

    // create one publisher for all the all the markers
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);

    // Display
    // ^^^^^^^
    while(ros::ok){
        PublishMarkers(uav_current_poses, uav_applied_ref, point_link_stars);                   
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}