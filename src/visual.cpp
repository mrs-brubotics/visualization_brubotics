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
#include <trackers_brubotics/FutureTrajectoryTube.h> // custom ROS message
#include <Eigen/Dense>

#include <ros/ros.h>

#include <iostream>
#include <string>

#define MAX_UAV_NUMBER 10           // Maximum number of UAVs for the visualization, used to initialized vectors and arrays
#define MAX_POINTS_TRAJECTORY 50    // Maximum number of points used to display the trajectory


// | -------------------------------- Parameters -------------------------------- |

const std::string empty = std::string();
bool test1 = false;
bool test2 = false;
int step;           // displayed trajectory points step
double Ra = 0.35;   // drone's radius
int number_of_uav;
std::array<std::array<double, 2>, 3> uav_applied_ref;
std::array<std::array<std::array<double, 2>, 3>, 1000> predicted_trajectories;


// | ------------------------ Publishers and subscribers ----------------------- |

// Publishers
ros::Publisher marker_publisher_;
ros::Publisher trajectory_publisher_;
// Subscribers
ros::Subscriber diagnostics_subscriber_;
ros::Subscriber DERG_strategy_id_subscriber_;
ros::Subscriber Sa_subscriber_;             
ros::Subscriber Sa_perp_subscriber_;
ros::Subscriber tube_min_radius_subscriber_;
std::vector<ros::Subscriber> uav_current_pose_subscribers_(MAX_UAV_NUMBER);
std::vector<ros::Subscriber> uav_applied_ref_subscribers_(MAX_UAV_NUMBER);
std::vector<ros::Subscriber> point_link_star_subscribers_(MAX_UAV_NUMBER);
std::vector<ros::Subscriber> future_tube_subscribers_(MAX_UAV_NUMBER);
std::vector<ros::Subscriber> predicted_trajectory_subscribers_(MAX_UAV_NUMBER);


// | -------------------------------- Messages --------------------------------- |

mrs_msgs::SpawnerDiagnostics diagnostics;               // we use the "active_vehicles" member of this message to set the number_of_uav
mrs_msgs::FutureTrajectory uav_applied_ref_traj;
mrs_msgs::FutureTrajectory predicted_traj;
trackers_brubotics::FutureTrajectoryTube future_tube;
geometry_msgs::PoseArray uav_current_pose;
geometry_msgs::Pose cylinder_pose;
geometry_msgs::Pose point_link_star;
std_msgs::Int32 _DERG_strategy_id_;
std_msgs::Int32 Sa_;                                    // error sphere radius (strategy 0)
std_msgs::Int32 Sa_min_perp;
std_msgs::Float32 tube_min_radius;
std::vector<geometry_msgs::Pose> uav_current_poses(MAX_UAV_NUMBER);
std::vector<geometry_msgs::Pose> point_link_stars(MAX_UAV_NUMBER);
std::vector<trackers_brubotics::FutureTrajectoryTube> future_tubes(MAX_UAV_NUMBER);


// | ---------------------------------- Class --------------------------------- |

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


// | --------------------------- Function prototypes -------------------------- |

// Callbacks
void DiagnosticsCallback(const mrs_msgs::SpawnerDiagnostics::ConstPtr& diagnostics);
void DERGStrategyIdCallback(const std_msgs::Int32::ConstPtr& msg);
void PredictedTrajectoryCallback(const mrs_msgs::FutureTrajectory::ConstPtr& msg, int uav_number);

void CurrentPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg, int uav_number);
void AppliedRefCallback(const mrs_msgs::FutureTrajectory::ConstPtr& msg, int uav_number);
void PointLinkStarCallback(const geometry_msgs::Pose::ConstPtr& msg, int uav_number);
void FutureTubeCallback(const trackers_brubotics::FutureTrajectoryTube::ConstPtr& msg, int uav_number);

void SaCallback(const std_msgs::Int32::ConstPtr& msg);
void SaPerpCallback(const std_msgs::Int32::ConstPtr& msg);
void TubeMinRadiusCallback(const std_msgs::Float32::ConstPtr& msg);

// Others functions
double getDistance(const Point& p1, const Point& p2);
Point getMiddle(const Point& pt1, const Point& pt2);
void CylinderOrientation(const Point &p1,const Point &p2, geometry_msgs::Pose& cylinder_pose, double& cylinder_height);
void CalculNorm(const std::array<std::array<std::array<double, 2>, 3>, 1000> point, const int& uav1, const int& uav2, double& norm, const int i);
void CalculNormMin(const std::array<std::array<std::array<double, 2>, 3>, 1000> point, const int& uav1, const int& uav2, double& norm_min, int& ind);
void GiveTranslatedPoint(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, geometry_msgs::Point& new_p, const double& distance, const double& norm);
void InitMarker(visualization_msgs::Marker& marker, const std::string name, const int id, const int type, const float r, const float g, const float b, const float a, const std::string &mesh);
void PublishMarkers(const std::vector<geometry_msgs::Pose>& obj_pose, std::array<std::array<double, 2>, 3> ref, 
                    const std::vector<geometry_msgs::Pose>& pstar, 
                    const std::vector<trackers_brubotics::FutureTrajectoryTube>& future_tubes, 
                    std::array<std::array<std::array<double, 2>, 3>, 1000> predicted_trajectories, int number_uav);


// | -------------------------------- Callbacks ------------------------------- |

void DiagnosticsCallback(const mrs_msgs::SpawnerDiagnostics::ConstPtr& msg){
    if (test1){
        return;
    }
    diagnostics.active_vehicles = msg -> active_vehicles;
    //ROS_INFO_STREAM("UAV list: " << diagnostics.active_vehicles[0] << ", " << diagnostics.active_vehicles[1]);
    test1 = true;
}

void DERGStrategyIdCallback(const std_msgs::Int32::ConstPtr& msg){
    _DERG_strategy_id_.data = msg -> data;
    //ROS_INFO_STREAM("DERG_strategy_id = " << _DERG_strategy_id_.data);
}

void PredictedTrajectoryCallback(const mrs_msgs::FutureTrajectory::ConstPtr& msg, int uav_number){
    predicted_traj.points = msg -> points;
    for(int i=0; i<predicted_traj.points.size(); i++){
        predicted_trajectories[i][0][uav_number-1] = predicted_traj.points[i].x;
        predicted_trajectories[i][1][uav_number-1] = predicted_traj.points[i].y;
        predicted_trajectories[i][2][uav_number-1] = predicted_traj.points[i].z;
        //ROS_INFO_STREAM("x : " << predicted_trajectories[i][0][uav_number-1]);
    }
    test2 = true;
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

void PointLinkStarCallback(const geometry_msgs::Pose::ConstPtr& msg, int uav_number){
    point_link_star.position = msg -> position;
    point_link_stars[uav_number-1].position = point_link_star.position;
    //ROS_INFO_STREAM("UAV" << uav_number << ": " << point_link_stars[uav_number-1].position);
}

void FutureTubeCallback(const trackers_brubotics::FutureTrajectoryTube::ConstPtr& msg, int uav_number){
    future_tube.min_radius = msg -> min_radius;
    future_tube.p0 = msg -> p0;
    future_tube.p1 = msg -> p1;
    future_tubes[uav_number-1].min_radius = future_tube.min_radius;
    future_tubes[uav_number-1].p0 = future_tube.p0;
    future_tubes[uav_number-1].p1 = future_tube.p1;
    //ROS_INFO_STREAM("UAV" << uav_number << ": " << future_tubes[uav_number-1]);
}

void SaCallback(const std_msgs::Int32::ConstPtr& msg){
    Sa_.data = msg -> data;
    //ROS_INFO_STREAM("Sa_ = " << Sa_.data);
}

void SaPerpCallback(const std_msgs::Int32::ConstPtr& msg){
    Sa_min_perp.data = msg -> data;
    //ROS_INFO_STREAM("Sa_min_perp = " << Sa_min_perp.data);
}

void TubeMinRadiusCallback(const std_msgs::Float32::ConstPtr& msg){
    tube_min_radius.data = msg -> data;
    //ROS_INFO_STREAM("tube_min_radius = " << tube_min_radius.data);
}


// | -------------------------- Function definitions -------------------------- |

// Return the distance between two points
double getDistance(const Point& p1, const Point& p2){
    double res;
    res = sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
    return res;
}

// Return the middle point of two points
Point getMiddle(const Point& pt1, const Point& pt2){
    Point res;
    res.x = (pt1.x + pt2.x)*0.5;
    res.y = (pt1.y + pt2.y)*0.5;
    res.z = (pt1.z + pt2.z)*0.5;
    return res;
}

// Calculate the pose and the height of a cylinder formed from two points
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

// From point p1, calculate the new point new_p transposed by distance in the direction formed by the director vector (p2, p1)
void GiveTranslatedPoint(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, geometry_msgs::Point& new_p, const double& distance, const double& norm){
    new_p.x = p1.x + distance * (p2.x-p1.x)/norm;
    new_p.y = p1.y + distance * (p2.y-p1.y)/norm;
    new_p.z = p1.z + distance * (p2.z-p1.z)/norm;
}

// Calculate the norm vector of two UAVs trajectories
void CalculNorm(const std::array<std::array<std::array<double, 2>, 3>, 1000> point, const int& uav1, const int& uav2, double& norm, const int i){
    norm = sqrt(  (point[i][0][uav1] - point[i][0][uav2])*(point[i][0][uav1] - point[i][0][uav2]) + (point[i][1][uav1] - point[i][1][uav2])*(point[i][1][uav1] - point[i][1][uav2]) + (point[i][2][uav1] - point[i][2][uav2])*(point[i][2][uav1] - point[i][2][uav2]));
}

// Calculate the minimal norm of two UAVs trajectories
void CalculNormMin(const std::array<std::array<std::array<double, 2>, 3>, 1000> point, const int& uav1, const int& uav2, double& norm_min, int& ind){
    double norm;
    CalculNorm(point,uav1,uav2,norm_min,0);
    for(int j=1; j<predicted_traj.points.size(); j++){

            CalculNorm(point,uav1,uav2,norm,j);

            if(norm < norm_min){
                norm_min = norm;
                ind = j;
            }
        }
}

void InitMarker(visualization_msgs::Marker& marker, const std::string name, const int id, const int type, const float r, const float g, const float b, const float a, const std::string &mesh = empty){
    marker.header.frame_id = "/common_origin";
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.id = id;
    marker.type = type; 
    if(type==10){
        marker.mesh_resource = "package://visualization_brubotics/meshes/" + mesh + ".stl";
    }
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = ros::Duration();
}

// Publish all the markers
void PublishMarkers(const std::vector<geometry_msgs::Pose>& obj_pose,
                    std::array<std::array<double, 2>, 3> ref, const std::vector<geometry_msgs::Pose>& pstar,
                    const std::vector<trackers_brubotics::FutureTrajectoryTube>& future_tubes,
                    std::array<std::array<std::array<double, 2>, 3>, 1000> predicted_trajectories, int number_uav){
    
    visualization_msgs::Marker marker;
    visualization_msgs::Marker desired_ref_sphere;
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
    visualization_msgs::Marker line_min_dist;
    visualization_msgs::Marker trajectory_sphere;
    visualization_msgs::MarkerArray trajectory_array;
    visualization_msgs::MarkerArray all_markers;
    geometry_msgs::Pose cylinder_pose;
    geometry_msgs::Point p;
    geometry_msgs::Point p_traj;
    geometry_msgs::Point p_min_dist;
    geometry_msgs::Point p_new1, p_new2, p_new3, p_new4;
    double cylinder_height;
    Point p1, p2;
    Point p21, p22;

    // loop for each drone
    for(int i=0; i<number_uav; i++){

        // small sphere at current pose

        InitMarker(marker,"current_pose_sphere",i,2,0.6f,0.6f,0.6f,0.15);
        marker.pose = obj_pose[i];
        marker.scale.x = 2*Ra;     // radius
        marker.scale.y = 2*Ra;     // radius
        marker.scale.z = 2*Ra;     // radius
        all_markers.markers.push_back(marker);

        // small sphere at applied ref pose
        
        InitMarker(marker,"applied_ref_sphere",i,2,0.6f,0.6f,0.6f,0.15);
        marker.pose.position.x = ref[0][i];
        marker.pose.position.y = ref[1][i];
        marker.pose.position.z = ref[2][i];
        marker.scale.x = 2*Ra;      // radius
        marker.scale.y = 2*Ra;      // radius
        marker.scale.z = 2*Ra;      // radius
        all_markers.markers.push_back(marker);

        // ends points of the line
        // p.x = obj_pose[i].position.x;
        // p.y = obj_pose[i].position.y;
        // p.z = obj_pose[i].position.z;
        // line.points.push_back(p);

        // Predicted trajectory
        
        InitMarker(marker,"trajectory",i,7,1.0f,0.0f,1.0f,1.0);
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.scale.x = 0.1;    // radius
        marker.scale.y = 0.1;    // radius
        marker.scale.z = 0.1;    // radius

        p_traj.x = predicted_trajectories[0][0][i];
        p_traj.y = predicted_trajectories[0][1][i];
        p_traj.z = predicted_trajectories[0][2][i];
        marker.points.push_back(p_traj);

        step = predicted_traj.points.size() / MAX_POINTS_TRAJECTORY;

        for(int j=step; j<(predicted_traj.points.size() - step); j+=step){
            p_traj.x = predicted_trajectories[j][0][i];
            p_traj.y = predicted_trajectories[j][1][i];
            p_traj.z = predicted_trajectories[j][2][i];
            marker.points.push_back(p_traj);
        }

        p_traj.x = predicted_trajectories[predicted_traj.points.size()-1][0][i];
        p_traj.y = predicted_trajectories[predicted_traj.points.size()-1][1][i];
        p_traj.z = predicted_trajectories[predicted_traj.points.size()-1][2][i];
        marker.points.push_back(p_traj);

        //ROS_INFO_STREAM("number of points: " << marker.points.size() << ", step: " << step << ", size: " << predicted_traj.points.size());

        all_markers.markers.push_back(marker);
        marker.points.clear();


        if(_DERG_strategy_id_.data == 0){

            // error sphere at applied ref pose
            InitMarker(marker, "error_sphere", i, 2, 0.8f, 0.898f, 1.0f, 0.1);
            marker.pose.position.x = ref[0][i];
            marker.pose.position.y = ref[1][i];
            marker.pose.position.z = ref[2][i];
            marker.scale.x = 2*Sa_.data;      // radius
            marker.scale.y = 2*Sa_.data;      // radius
            marker.scale.z = 2*Sa_.data;      // radius
            all_markers.markers.push_back(marker);

        }

        if(_DERG_strategy_id_.data == 1 || _DERG_strategy_id_.data == 2){

            // Blue tube
            // cylinder1 between point link star and applied ref
            
            p1.x = pstar[i].position.x;
            p1.y = pstar[i].position.y;
            p1.z = pstar[i].position.z;
            p2.x = ref[0][i];
            p2.y = ref[1][i];
            p2.z = ref[2][i];
            if(_DERG_strategy_id_.data == 1)
                InitMarker(marker, "cylinder_strategy_1", i, 10, 0.4f, 0.698f, 1.0f, 0.075, "CylinderShell_10mm");
            
            if(_DERG_strategy_id_.data == 2)
                InitMarker(marker, "cylinder_strategy_1", i, 10, 0.251f, 0.251f, 0.251f, 0.075, "CylinderShell_10mm");
            
            CylinderOrientation(p1, p2, cylinder_pose, cylinder_height);
            marker.pose = cylinder_pose;
            marker.scale.x = 0.001*2*Sa_min_perp.data;    // radius
            marker.scale.y = 0.001*2*Sa_min_perp.data;    // radius
            marker.scale.z = 0.001*cylinder_height;    // height
            all_markers.markers.push_back(marker);

            // Blue tube
            // hemisphere 1 at cylinder ends

            if(_DERG_strategy_id_.data == 1)
                InitMarker(marker, "hemisphere1_strategy_1", i, 10, 0.4f, 0.698f, 1.0f, 0.075, "HemisphereShell_10mm");
            
            if(_DERG_strategy_id_.data == 2)
                InitMarker(marker, "hemisphere1_strategy_1", i, 10, 0.251f, 0.251f, 0.251f, 0.075, "HemisphereShell_10mm");
            
            marker.pose.position.x = ref[0][i];
            marker.pose.position.y = ref[1][i];
            marker.pose.position.z = ref[2][i];
            marker.pose.orientation = cylinder_pose.orientation;
            //ROS_INFO_STREAM("sphere 1 " << cylinder_pose.orientation);      
            marker.scale.x = 0.001 * Sa_min_perp.data; 
            marker.scale.y = 0.001 * Sa_min_perp.data; 
            marker.scale.z = 0.001 * Sa_min_perp.data; 
            all_markers.markers.push_back(marker);

            // Blue tube
            // hemisphere 2 at cylinder ends
            if(_DERG_strategy_id_.data == 1)
                InitMarker(marker, "hemisphere2_strategy_1", i, 10, 0.4f, 0.698f, 1.0f, 0.075, "HemisphereShell_10mm");
            
            if(_DERG_strategy_id_.data == 2)
                InitMarker(marker, "hemisphere2_strategy_1", i, 10, 0.251f, 0.251f, 0.251f, 0.075, "HemisphereShell_10mm");
            
            marker.pose.position = pstar[i].position;
            CylinderOrientation(p2, p1, cylinder_pose, cylinder_height);
            marker.pose.orientation = cylinder_pose.orientation;
            //ROS_INFO_STREAM("sphere 2 " << cylinder_pose.orientation);
            marker.scale.x = 0.001 *Sa_min_perp.data; 
            marker.scale.y = 0.001 *Sa_min_perp.data; 
            marker.scale.z = 0.001 *Sa_min_perp.data; 
            all_markers.markers.push_back(marker);

        }

        if(_DERG_strategy_id_.data == 2){

            // Blue tube
            // cylinder between current pose and applied ref
            p21.x = obj_pose[i].position.x;
            p21.y = obj_pose[i].position.y;
            p21.z = obj_pose[i].position.z;
            p22.x = ref[0][i];
            p22.y = ref[1][i];
            p22.z = ref[2][i];
            InitMarker(marker, "cylinder_strategy_2", i, 10, 0.4f, 0.698f, 1.0f, 0.075, "CylinderShell_10mm");
            CylinderOrientation(p21, p22, cylinder_pose, cylinder_height);
            marker.pose = cylinder_pose;
            marker.scale.x = 0.001*2*Sa_min_perp.data;    // radius
            marker.scale.y = 0.001*2*Sa_min_perp.data;    // radius
            marker.scale.z = 0.001*cylinder_height;    // height
            all_markers.markers.push_back(marker);


            // Blue tube
            // hemisphere 1 at cylinder ends
            InitMarker(marker, "hemisphere1_strategy_2", i, 10, 0.4f, 0.698f, 1.0f, 0.075, "HemisphereShell_10mm");
            marker.pose.position.x = ref[0][i];
            marker.pose.position.y = ref[1][i];
            marker.pose.position.z = ref[2][i];
            marker.pose.orientation = cylinder_pose.orientation;        
            marker.scale.x = 0.001*Sa_min_perp.data; 
            marker.scale.y = 0.001*Sa_min_perp.data; 
            marker.scale.z = 0.001*Sa_min_perp.data; 
            all_markers.markers.push_back(marker);

            // Blue tube
            // hemisphere 1 at cylinder ends
            InitMarker(marker, "hemisphere2_strategy_2", i, 10, 0.4f, 0.698f, 1.0f, 0.075, "HemisphereShell_10mm");
            CylinderOrientation(p22, p21, cylinder_pose, cylinder_height);
            marker.pose.position = obj_pose[i].position;
            marker.pose.orientation = cylinder_pose.orientation;        
            marker.scale.x = 0.001*Sa_min_perp.data; 
            marker.scale.y = 0.001*Sa_min_perp.data; 
            marker.scale.z = 0.001*Sa_min_perp.data; 
            all_markers.markers.push_back(marker);

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
            cylinder1.scale.x = 2*Sa_min_perp.data;     // radius
            cylinder1.scale.y = 2*Sa_min_perp.data;     // radius
            cylinder1.scale.z = cylinder_height;        // height
            cylinder1.color.r = 0.4f;
            cylinder1.color.g = 0.698f;
            cylinder1.color.b = 1.0f;
            cylinder1.color.a = 0.2;
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
            cylinder_hemisphere12.scale.x = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere12.scale.y = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere12.scale.z = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere12.color.r = 0.4f;
            cylinder_hemisphere12.color.g = 0.698f;
            cylinder_hemisphere12.color.b = 1.0f;
            cylinder_hemisphere12.color.a = 0.15;
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
            cylinder_hemisphere11.scale.x = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere11.scale.y = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere11.scale.z = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere11.color.r = 0.4f;
            cylinder_hemisphere11.color.g = 0.698f;
            cylinder_hemisphere11.color.b = 1.0f;
            cylinder_hemisphere11.color.a = 0.15;
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
            cylinder2.scale.x = 2*Sa_min_perp.data;    // radius
            cylinder2.scale.y = 2*Sa_min_perp.data;    // radius
            cylinder2.scale.z = cylinder_height;    // height
            cylinder2.color.r = 0.251f;
            cylinder2.color.g = 0.251f;
            cylinder2.color.b = 0.251f;
            cylinder2.color.a = 0.1;
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
            cylinder_hemisphere22.scale.x = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere22.scale.y = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere22.scale.z = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere22.color.r = 0.251f;
            cylinder_hemisphere22.color.g = 0.251f;
            cylinder_hemisphere22.color.b = 0.251f;
            cylinder_hemisphere22.color.a = 0.1;
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
            cylinder_hemisphere21.scale.x = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere21.scale.y = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere21.scale.z = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere21.color.r = 0.251f;
            cylinder_hemisphere21.color.g = 0.251f;
            cylinder_hemisphere21.color.b = 0.251f;
            cylinder_hemisphere21.color.a = 0.1;
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
            cylinder3.scale.x = 2*future_tubes[i].min_radius;   // radius
            cylinder3.scale.y = 2*future_tubes[i].min_radius;   // radius
            cylinder3.scale.z = cylinder_height;                // height
            cylinder3.color.r = 0.749f;
            cylinder3.color.g = 0.647f;
            cylinder3.color.b = 0.412f;
            cylinder3.color.a = 0.5;
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
            cylinder_hemisphere32.scale.x = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder_hemisphere32.scale.y = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder_hemisphere32.scale.z = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder_hemisphere32.color.r = 0.749f;
            cylinder_hemisphere32.color.g = 0.647f;
            cylinder_hemisphere32.color.b = 0.412f;
            cylinder_hemisphere32.color.a = 0.5;
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
            CylinderOrientation(p24, p23, cylinder_pose, cylinder_height);
            cylinder_hemisphere31.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere31.scale.x = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder_hemisphere31.scale.y = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder_hemisphere31.scale.z = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder_hemisphere31.color.r = 0.749f;
            cylinder_hemisphere31.color.g = 0.647f;
            cylinder_hemisphere31.color.b = 0.412f;
            cylinder_hemisphere31.color.a = 0.5;
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
            cylinder1.scale.x = 2*Sa_min_perp.data;     // radius
            cylinder1.scale.y = 2*Sa_min_perp.data;     // radius
            cylinder1.scale.z = cylinder_height;        // height
            cylinder1.color.r = 0.4f;
            cylinder1.color.g = 0.698f;
            cylinder1.color.b = 1.0f;
            cylinder1.color.a = 0.15;
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
            cylinder_hemisphere12.scale.x = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere12.scale.y = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere12.scale.z = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere12.color.r = 0.4f;
            cylinder_hemisphere12.color.g = 0.698f;
            cylinder_hemisphere12.color.b = 1.0f;
            cylinder_hemisphere12.color.a = 0.15;
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
            cylinder_hemisphere11.scale.x = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere11.scale.y = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere11.scale.z = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere11.color.r = 0.4f;
            cylinder_hemisphere11.color.g = 0.698f;
            cylinder_hemisphere11.color.b = 1.0f;
            cylinder_hemisphere11.color.a = 0.15;
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
            cylinder2.scale.x = 2*Sa_min_perp.data;    // radius
            cylinder2.scale.y = 2*Sa_min_perp.data;    // radius
            cylinder2.scale.z = cylinder_height;       // height
            cylinder2.color.r = 0.251f;
            cylinder2.color.g = 0.251f;
            cylinder2.color.b = 0.251f;
            cylinder2.color.a = 0.1;
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
            cylinder_hemisphere22.scale.x = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere22.scale.y = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere22.scale.z = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere22.color.r = 0.251f;
            cylinder_hemisphere22.color.g = 0.251f;
            cylinder_hemisphere22.color.b = 0.251f;
            cylinder_hemisphere22.color.a = 0.1;
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
            cylinder_hemisphere21.scale.x = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere21.scale.y = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere21.scale.z = 0.001*Sa_min_perp.data; 
            cylinder_hemisphere21.color.r = 0.251f;
            cylinder_hemisphere21.color.g = 0.251f;
            cylinder_hemisphere21.color.b = 0.251f;
            cylinder_hemisphere21.color.a = 0.1;
            cylinder_hemisphere21.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere21);

            // Orange tube
            // cylinder3 between point link star and applied ref
            Point p25, p26;
            p25.x = future_tubes[i].p1.x;
            p25.y = future_tubes[i].p1.y;
            p25.z = future_tubes[i].p1.z;
            p26.x = future_tubes[i].p0.x;
            p26.y = future_tubes[i].p0.y;
            p26.z = future_tubes[i].p0.z;
            cylinder3.header.frame_id = "/common_origin";
            cylinder3.id = i;
            cylinder3.header.stamp = ros::Time::now();
            cylinder3.ns = "cylinder3";
            cylinder3.type = visualization_msgs::Marker::CYLINDER; 
            cylinder3.action = visualization_msgs::Marker::ADD;
            CylinderOrientation(p25, p26, cylinder_pose, cylinder_height);
            cylinder3.pose = cylinder_pose;
            cylinder3.scale.x = 2*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder3.scale.y = 2*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder3.scale.z = cylinder_height;                // height
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
            cylinder_hemisphere32.pose.position.x = future_tubes[i].p0.x;
            cylinder_hemisphere32.pose.position.y = future_tubes[i].p0.y;
            cylinder_hemisphere32.pose.position.z = future_tubes[i].p0.z;
            cylinder_hemisphere32.pose.orientation = cylinder_pose.orientation;     
            cylinder_hemisphere32.scale.x = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder_hemisphere32.scale.y = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder_hemisphere32.scale.z = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
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
            cylinder_hemisphere31.pose.position.x = future_tubes[i].p1.x;
            cylinder_hemisphere31.pose.position.y = future_tubes[i].p1.y;
            cylinder_hemisphere31.pose.position.z = future_tubes[i].p1.z;
            CylinderOrientation(p24, p23, cylinder_pose, cylinder_height);
            cylinder_hemisphere31.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere31.scale.x = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder_hemisphere31.scale.y = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder_hemisphere31.scale.z = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            cylinder_hemisphere31.color.r = 0.8f;
            cylinder_hemisphere31.color.g = 0.0f;
            cylinder_hemisphere31.color.b = 0.0f;
            cylinder_hemisphere31.color.a = 0.5;
            cylinder_hemisphere31.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere31);

        }

        if(_DERG_strategy_id_.data == 5){

            // Orange tube
            // cylinder3 between point link star and applied ref
            Point p35, p36;
            p35.x = obj_pose[i].position.x;
            p35.y = obj_pose[i].position.y;
            p35.z = obj_pose[i].position.z;
            p36.x = ref[0][i];
            p36.y = ref[1][i];
            p36.z = ref[2][i];
            cylinder3.header.frame_id = "/common_origin";
            cylinder3.id = i;
            cylinder3.header.stamp = ros::Time::now();
            cylinder3.ns = "cylinder5";
            cylinder3.type = visualization_msgs::Marker::CYLINDER; 
            cylinder3.action = visualization_msgs::Marker::ADD;
            CylinderOrientation(p35, p36, cylinder_pose, cylinder_height);
            cylinder3.pose = cylinder_pose;
            cylinder3.scale.x = 2*tube_min_radius.data;   // radius
            cylinder3.scale.y = 2*tube_min_radius.data;   // radius
            cylinder3.scale.z = cylinder_height;          // height
            cylinder3.color.r = 0.749f;
            cylinder3.color.g = 0.647f;
            cylinder3.color.b = 0.412f;
            cylinder3.color.a = 0.5;
            all_markers.markers.push_back(cylinder3);

            // Orange tube
            // sphere32 at cylinder ends
            cylinder_hemisphere32.header.frame_id = "/common_origin";
            cylinder_hemisphere32.header.stamp = ros::Time::now();
            cylinder_hemisphere32.ns = "cylinder_hemisphere52";
            cylinder_hemisphere32.id = i;
            cylinder_hemisphere32.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere32.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere32.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere32.pose.position.x = ref[0][i];
            cylinder_hemisphere32.pose.position.y = ref[1][i];
            cylinder_hemisphere32.pose.position.z = ref[2][i];
            cylinder_hemisphere32.pose.orientation = cylinder_pose.orientation;     
            cylinder_hemisphere32.scale.x = 0.001*tube_min_radius.data;     // Sa_min_perp radius
            cylinder_hemisphere32.scale.y = 0.001*tube_min_radius.data;     // Sa_min_perp radius 
            cylinder_hemisphere32.scale.z = 0.001*tube_min_radius.data;     // Sa_min_perp radius 
            cylinder_hemisphere32.color.r = 0.749f;
            cylinder_hemisphere32.color.g = 0.647f;
            cylinder_hemisphere32.color.b = 0.412f;
            cylinder_hemisphere32.color.a = 0.5;
            cylinder_hemisphere32.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere32);

            // Orange tube
            // sphere31 at cylinder ends
            cylinder_hemisphere31.header.frame_id = "/common_origin";
            cylinder_hemisphere31.header.stamp = ros::Time::now();
            cylinder_hemisphere31.ns = "cylinder_hemisphere51";
            cylinder_hemisphere31.id = i;
            cylinder_hemisphere31.type = visualization_msgs::Marker::MESH_RESOURCE; 
            cylinder_hemisphere31.mesh_resource = "package://visualization_brubotics/meshes/hemisphere1.stl";
            cylinder_hemisphere31.action = visualization_msgs::Marker::ADD;
            cylinder_hemisphere31.pose.position = obj_pose[i].position;
            CylinderOrientation(p36, p35, cylinder_pose, cylinder_height);
            cylinder_hemisphere31.pose.orientation = cylinder_pose.orientation;
            cylinder_hemisphere31.scale.x = 0.001*tube_min_radius.data;     // Sa_min_perp radius 
            cylinder_hemisphere31.scale.y = 0.001*tube_min_radius.data;     // Sa_min_perp radius 
            cylinder_hemisphere31.scale.z = 0.001*tube_min_radius.data;     // Sa_min_perp radius 
            cylinder_hemisphere31.color.r = 0.749f;
            cylinder_hemisphere31.color.g = 0.647f;
            cylinder_hemisphere31.color.b = 0.412f;
            cylinder_hemisphere31.color.a = 0.5;
            cylinder_hemisphere31.lifetime = ros::Duration();
            all_markers.markers.push_back(cylinder_hemisphere31);

        }

    }

    if(_DERG_strategy_id_.data == 5){

        // line for minimal distance
        double norm_min;
        int ind;
        
        // calculate the minimal norm and return index
        CalculNormMin(predicted_trajectories,0,1,norm_min,ind);

        p_new1.x = predicted_trajectories[ind][0][0];
        p_new1.y = predicted_trajectories[ind][1][0];
        p_new1.z = predicted_trajectories[ind][2][0];
        
        p_new2.x = predicted_trajectories[ind][0][1];
        p_new2.y = predicted_trajectories[ind][1][1];
        p_new2.z = predicted_trajectories[ind][2][1];

        // calculate the coordinates of the desired ref position translated by radius Ra
        GiveTranslatedPoint(p_new1,p_new2,p_new3,Ra,norm_min);
        GiveTranslatedPoint(p_new2,p_new1,p_new4,Ra,norm_min);

        line_min_dist.points.push_back(p_new3);
        line_min_dist.points.push_back(p_new4);

        line_min_dist.header.frame_id = "/common_origin";
        line_min_dist.id = 0;
        line_min_dist.header.stamp = ros::Time::now();
        line_min_dist.ns = "minimal_distance";
        line_min_dist.type = visualization_msgs::Marker::LINE_STRIP; 
        line_min_dist.action = visualization_msgs::Marker::ADD;
        line_min_dist.scale.x = 0.05; // width 
        line_min_dist.color.r = 0.0f;
        line_min_dist.color.g = 0.0f;
        line_min_dist.color.b = 0.0f;
        line_min_dist.color.a = 1.0;
        all_markers.markers.push_back(line_min_dist);

        // sphere at desired reference pose...
        desired_ref_sphere.header.frame_id = "/common_origin";
        desired_ref_sphere.header.stamp = ros::Time::now();
        desired_ref_sphere.ns = "desired_ref_sphere";
        desired_ref_sphere.type = visualization_msgs::Marker::SPHERE; 
        desired_ref_sphere.action = visualization_msgs::Marker::ADD;
        desired_ref_sphere.pose.orientation.x = 0;
        desired_ref_sphere.pose.orientation.y = 0;
        desired_ref_sphere.pose.orientation.z = 0;
        desired_ref_sphere.scale.x = 2*Ra; 
        desired_ref_sphere.scale.y = 2*Ra; 
        desired_ref_sphere.scale.z = 2*Ra; 
        desired_ref_sphere.color.r = 0.6f;
        desired_ref_sphere.color.g = 0.6f;
        desired_ref_sphere.color.b = 0.6f;
        desired_ref_sphere.color.a = 0.15;
        desired_ref_sphere.lifetime = ros::Duration();

        desired_ref_sphere.id = 0;
        desired_ref_sphere.pose.position = p_new1;
        all_markers.markers.push_back(desired_ref_sphere);

        desired_ref_sphere.id = 1;
        desired_ref_sphere.pose.position = p_new2;
        all_markers.markers.push_back(desired_ref_sphere);

        }


    // // red line betweetn both current uav pose
    // line.header.frame_id = "/common_origin";
    // line.id = 0;
    // line.header.stamp = ros::Time::now();
    // line.ns = "line";
    // line.type = visualization_msgs::Marker::LINE_STRIP; 
    // line.action = visualization_msgs::Marker::ADD;
    // line.pose.orientation.w = 1.0;
    // line.scale.x = 0.05; // width
    // line.color.r = 1.0f;
    // line.color.g = 0.0f;
    // line.color.b = 0.0f;
    // line.color.a = 1.0;
    // all_markers.markers.push_back(line);

    marker_publisher_.publish(all_markers);

}



// | ---------------------------------- MAIN ---------------------------------- |

int main(int argc, char **argv){
    
    // Initialization
    // ^^^^^^^^^^^^^^

    // init node
    ros::init(argc, argv,"current_pose_sphere");
    ros::NodeHandle n;
    ros::Rate r(30);

    // Subscribers and publishers
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^

    // Subscribe
    diagnostics_subscriber_ = n.subscribe("mrs_drone_spawner/diagnostics", 1, DiagnosticsCallback);
    DERG_strategy_id_subscriber_ = n.subscribe("uav1/control_manager/dergbryan_tracker/derg_strategy_id", 1, DERGStrategyIdCallback);
    Sa_subscriber_ = n.subscribe("uav1/control_manager/dergbryan_tracker/sa_max", 1, SaCallback);
    Sa_perp_subscriber_ = n.subscribe("uav1/control_manager/dergbryan_tracker/sa_perp_max", 1, SaPerpCallback);
    tube_min_radius_subscriber_ = n.subscribe("uav1/control_manager/dergbryan_tracker/tube_min_radius", 1, TubeMinRadiusCallback);

    //  We wait until we receive the message for the diagnostics_subscriber_
    while(!test1){
        ros::spinOnce();
        r.sleep();

    }

    number_of_uav = diagnostics.active_vehicles.size();

    // create subscribers
    std::vector<boost::function<void (const geometry_msgs::PoseArray::ConstPtr&)>> f1;
    f1.resize(MAX_UAV_NUMBER);
    std::vector<boost::function<void (const mrs_msgs::FutureTrajectory::ConstPtr&)>> f2;
    f2.resize(MAX_UAV_NUMBER);
    std::vector<boost::function<void (const geometry_msgs::Pose::ConstPtr&)>> f3;
    f3.resize(MAX_UAV_NUMBER);
    std::vector<boost::function<void (const mrs_msgs::FutureTrajectory::ConstPtr&)>> f4;
    f4.resize(MAX_UAV_NUMBER);
    std::vector<boost::function<void (const trackers_brubotics::FutureTrajectoryTube::ConstPtr&)>> f5;
    f5.resize(MAX_UAV_NUMBER);


    for(int i=0; i<diagnostics.active_vehicles.size(); i++){

        // create subscribers for current uav pose
        f1[i] = boost::bind(CurrentPoseCallback, _1, i+1);
        uav_current_pose_subscribers_[i] = n.subscribe(diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/custom_predicted_poses", 10, f1[i]);

        // create subscribers for uav applied ref
        f2[i] = boost::bind(AppliedRefCallback, _1, i+1);
        uav_applied_ref_subscribers_[i] = n.subscribe(diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/uav_applied_ref", 10, f2[i]);

        // create subscribers for uav point link star
        f3[i] = boost::bind(PointLinkStarCallback, _1, i+1);
        point_link_star_subscribers_[i] = n.subscribe(diagnostics.active_vehicles[i] +"/control_manager/dergbryan_tracker/point_link_star", 10, f3[i]);

        // create subscribers for uav predicted trajectory
        f4[i] = boost::bind(PredictedTrajectoryCallback, _1, i+1);
        predicted_trajectory_subscribers_[i] = n.subscribe(diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/predicted_trajectory", 10, f4[i]);

        // create subscribers for uav future tube
        f5[i] = boost::bind(FutureTubeCallback, _1, i+1);
        future_tube_subscribers_[i] = n.subscribe(diagnostics.active_vehicles[i] +"/control_manager/dergbryan_tracker/future_trajectory_tube", 10, f5[i]);

    }   


    while(!test2){
        ros::spinOnce();
        r.sleep();

    }

    // create one publisher for all the all the markers
    marker_publisher_ = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);

    // Display
    // ^^^^^^^

    while(ros::ok){
        PublishMarkers(uav_current_poses, uav_applied_ref, point_link_stars, future_tubes, predicted_trajectories, number_of_uav);                   
        ros::spinOnce();
        r.sleep();

    }

    return 0;

}