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
void CalculNorm(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, double& norm);
void CalculNormMin(const std::array<std::array<std::array<double, 2>, 3>, 1000> point, const int& uav1, const int& uav2, double& norm_min, int& ind);
void GiveTranslatedPoint(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, geometry_msgs::Point& new_p, const double& distance, const double& norm);
void ShortestDistanceLines(visualization_msgs::MarkerArray& markers, visualization_msgs::Marker& marker, const std::array<std::array<std::array<double, 2>, 3>, 1000> point, const double& radius, const int& number_uav);
void Trajectory(visualization_msgs::Marker& marker, int ind_uav, const std::array<std::array<std::array<double, 2>, 3>, 1000>& predicted_trajectories, const int& type);
void RedLines(visualization_msgs::MarkerArray& markers, visualization_msgs::Marker& marker, const std::vector<geometry_msgs::Pose>& obj_pose, const int& number_uav, const double& radius);
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
void GiveTranslatedPoint(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, geometry_msgs::Point& new_p, const double& distance, double& norm){
    new_p.x = p1.x + distance * (p2.x-p1.x)/norm;
    new_p.y = p1.y + distance * (p2.y-p1.y)/norm;
    new_p.z = p1.z + distance * (p2.z-p1.z)/norm;
}

// Calculate the norm vector of two UAVs trajectories
void CalculNorm(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, double& norm){
    norm = sqrt(  (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z) );
}

// Calculate the minimal norm of two UAVs trajectories
void CalculNormMin(const std::array<std::array<std::array<double, 2>, 3>, 1000> point, const int& uav1, const int& uav2, double& norm_min, int& ind){
    double norm;
    geometry_msgs::Point ptest1, ptest2;
    ptest1.x = point[0][0][uav1];
    ptest1.y = point[0][1][uav1];
    ptest1.z = point[0][2][uav1];
                
    ptest2.x = point[0][0][uav2];
    ptest2.y = point[0][1][uav2];
    ptest2.z = point[0][2][uav2];
    CalculNorm(ptest1, ptest2, norm_min);
    for(int j=1; j<predicted_traj.points.size(); j++){
            
            ptest1.x = point[j][0][uav1];
            ptest1.y = point[j][1][uav1];
            ptest1.z = point[j][2][uav1];
                
            ptest2.x = point[j][0][uav2];
            ptest2.y = point[j][1][uav2];
            ptest2.z = point[j][2][uav2];
            ROS_INFO_STREAM("p1 : " << ptest1 << " p2 : " << ptest2);
            CalculNorm(ptest1, ptest2, norm);

            if(norm < norm_min){
                norm_min = norm;
                ind = j;
            }
        }
}

void InitMarker(visualization_msgs::Marker& marker, const std::string name, const int id, const int type, const float r, const float g, const float b, const float a, const std::string &mesh = empty){
    //Clear marker pose
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0;

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

void ShortestDistanceLines(visualization_msgs::MarkerArray& markers, visualization_msgs::Marker& marker, const std::array<std::array<std::array<double, 2>, 3>, 1000> point, const double& radius, const int& number_uav){
    // line for minimal distance
    double norm_min;
    int ind;
    geometry_msgs::Point p1, p2, p_new1, p_new2;

    for(int i=0; i<(number_uav-1); i++){

        for(int j=i+1; j<number_uav; j++){

            // calculate the minimal norm and return index
            CalculNormMin(point,i,j,norm_min,ind);
            p1.x = point[ind][0][i];
            p1.y = point[ind][1][i];
            p1.z = point[ind][2][i];
                
            p2.x = point[ind][0][j];
            p2.y = point[ind][1][j];
            p2.z = point[ind][2][j];
            InitMarker(marker, "minimal_distance", i*100+j, 4, 0.0f, 0.0f, 0.0f, 1.0);
            // calculate the coordinates of the desired ref position translated by radius Ra
            GiveTranslatedPoint(p1,p2,p_new1,radius,norm_min);
            GiveTranslatedPoint(p2,p1,p_new2,radius,norm_min);

            marker.points.push_back(p_new1);
            marker.points.push_back(p_new2);

            marker.scale.x = 0.05; // width 

            markers.markers.push_back(marker);

            // spheres at desired reference pose
            InitMarker(marker, "desired_ref_sphere", i*100+j, 2, 0.6f, 0.6f, 0.6f, 0.15);
            marker.scale.x = 2*Ra; 
            marker.scale.y = 2*Ra; 
            marker.scale.z = 2*Ra; 
            marker.pose.position = p1;
            markers.markers.push_back(marker);

            marker.id = i*100+j+100000;
            marker.pose.position = p2;
            markers.markers.push_back(marker);
        }
    } 
}

void Trajectory(visualization_msgs::Marker& marker, int ind_uav, const std::array<std::array<std::array<double, 2>, 3>, 1000>& predicted_trajectories, const int& type){
    
    geometry_msgs::Point p_traj;

    if(type==4){
        InitMarker(marker,"trajectory line strip", ind_uav,type,1.0f,0.0f,1.0f,1.0);
        marker.scale.x = 0.05;    // width of the line strip
    }
    if(type==7){
        InitMarker(marker,"trajectory sphere list", ind_uav,type,1.0f,0.0f,1.0f,1.0);
        marker.scale.x = 0.1;    // radius of the spheres
        marker.scale.y = 0.1;    // radius
        marker.scale.z = 0.1;    // radius
    }

    step = predicted_traj.points.size() / MAX_POINTS_TRAJECTORY;

    for(int j=0; j<(predicted_traj.points.size() - step); j+=step){
        p_traj.x = predicted_trajectories[j][0][ind_uav];
        p_traj.y = predicted_trajectories[j][1][ind_uav];
        p_traj.z = predicted_trajectories[j][2][ind_uav];
        marker.points.push_back(p_traj);
    }

    p_traj.x = predicted_trajectories[predicted_traj.points.size()-1][0][ind_uav];
    p_traj.y = predicted_trajectories[predicted_traj.points.size()-1][1][ind_uav];
    p_traj.z = predicted_trajectories[predicted_traj.points.size()-1][2][ind_uav];
    marker.points.push_back(p_traj);

    //ROS_INFO_STREAM("number of points: " << marker.points.size() << ", step: " << step << ", size: " << predicted_traj.points.size());
}

void RedLines(visualization_msgs::MarkerArray& markers, visualization_msgs::Marker& marker, const std::vector<geometry_msgs::Pose>& obj_pose, const int& number_uav, const double& radius){
    geometry_msgs::Point p1, p2, p_new;
    double norm;
    for(int i=0; i<(number_uav-1); i++){
        
        for(int j=i+1; j<number_uav; j++){
            InitMarker(marker,"red_line",i*100+j,4,1.0f,0.0f,0.0f,1.0);    
            marker.scale.x = 0.05; // width

            p1.x = obj_pose[i].position.x;
            p1.y = obj_pose[i].position.y;
            p1.z = obj_pose[i].position.z;

            
            p2.x = obj_pose[j].position.x;
            p2.y = obj_pose[j].position.y;
            p2.z = obj_pose[j].position.z;

            CalculNorm(p1, p2, norm);

            GiveTranslatedPoint(p1,p2,p_new,radius,norm);
            marker.points.push_back(p_new);

            GiveTranslatedPoint(p2,p1,p_new,radius,norm);
            marker.points.push_back(p_new);

            markers.markers.push_back(marker);
            marker.points.clear();
        }
    }
}

// Publish all the markers
void PublishMarkers(const std::vector<geometry_msgs::Pose>& obj_pose,
                    std::array<std::array<double, 2>, 3> ref, const std::vector<geometry_msgs::Pose>& pstar,
                    const std::vector<trackers_brubotics::FutureTrajectoryTube>& future_tubes,
                    std::array<std::array<std::array<double, 2>, 3>, 1000> predicted_trajectories, int number_uav){
    
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray all_markers;
    geometry_msgs::Pose cylinder_pose;
    double cylinder_height;
    Point p11, p12;
    Point p21, p22;
    Point p31, p32;
    Point p41, p42;

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

        // Predicted trajectory sphere list
        Trajectory(marker, i, predicted_trajectories, 7);
        all_markers.markers.push_back(marker);
        marker.points.clear();
        
        // Predicted trajectory line strip
        Trajectory(marker, i, predicted_trajectories, 4);
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

            //Strategy 1 part
        if(_DERG_strategy_id_.data == 1 || _DERG_strategy_id_.data == 2 || _DERG_strategy_id_.data == 3 || _DERG_strategy_id_.data == 4){  

            // Blue tube

            // cylinder1 between point link star and applied ref
            p11.x = pstar[i].position.x;
            p11.y = pstar[i].position.y;
            p11.z = pstar[i].position.z;
            p12.x = ref[0][i];
            p12.y = ref[1][i];
            p12.z = ref[2][i];
            if(_DERG_strategy_id_.data == 1)
                InitMarker(marker, "cylinder_strategy_1", i, 10, 0.4f, 0.698f, 1.0f, 0.075, "CylinderShell_10mm");
            
            if(_DERG_strategy_id_.data == 2 || _DERG_strategy_id_.data == 3 || _DERG_strategy_id_.data == 4)
                InitMarker(marker, "cylinder_strategy_1", i, 10, 0.251f, 0.251f, 0.251f, 0.05, "CylinderShell_10mm");
            
            CylinderOrientation(p11, p12, cylinder_pose, cylinder_height);
            marker.pose = cylinder_pose;
            marker.scale.x = 0.001*2*Sa_min_perp.data;    // radius
            marker.scale.y = 0.001*2*Sa_min_perp.data;    // radius
            marker.scale.z = 0.001*cylinder_height;    // height
            all_markers.markers.push_back(marker);

            // hemisphere 1 at cylinder ends
            if(_DERG_strategy_id_.data == 1)
                InitMarker(marker, "hemisphere1_strategy_1", i, 10, 0.4f, 0.698f, 1.0f, 0.075, "HemisphereShell_10mm");
            
            if(_DERG_strategy_id_.data == 2 || _DERG_strategy_id_.data == 3 || _DERG_strategy_id_.data == 4)
                InitMarker(marker, "hemisphere1_strategy_1", i, 10, 0.251f, 0.251f, 0.251f, 0.05, "HemisphereShell_10mm");
            
            marker.pose.position.x = ref[0][i];
            marker.pose.position.y = ref[1][i];
            marker.pose.position.z = ref[2][i];
            marker.pose.orientation = cylinder_pose.orientation;
            //ROS_INFO_STREAM("sphere 1 " << cylinder_pose.orientation);      
            marker.scale.x = 0.001 * Sa_min_perp.data; 
            marker.scale.y = 0.001 * Sa_min_perp.data; 
            marker.scale.z = 0.001 * Sa_min_perp.data; 
            all_markers.markers.push_back(marker);

            // hemisphere 2 at cylinder ends
            if(_DERG_strategy_id_.data == 1)
                InitMarker(marker, "hemisphere2_strategy_1", i, 10, 0.4f, 0.698f, 1.0f, 0.075, "HemisphereShell_10mm");
            
            if(_DERG_strategy_id_.data == 2 || _DERG_strategy_id_.data == 3)
                InitMarker(marker, "hemisphere2_strategy_1", i, 10, 0.251f, 0.251f, 0.251f, 0.05, "HemisphereShell_10mm");
            
            marker.pose.position = pstar[i].position;
            CylinderOrientation(p12, p11, cylinder_pose, cylinder_height);
            marker.pose.orientation = cylinder_pose.orientation;
            //ROS_INFO_STREAM("sphere 2 " << cylinder_pose.orientation);
            marker.scale.x = 0.001 *Sa_min_perp.data; 
            marker.scale.y = 0.001 *Sa_min_perp.data; 
            marker.scale.z = 0.001 *Sa_min_perp.data; 
            all_markers.markers.push_back(marker);
        }

            //Strategy 2 part
        if(_DERG_strategy_id_.data == 2 || _DERG_strategy_id_.data == 3 || _DERG_strategy_id_.data == 4){

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

            //Strategy 3 part
        if(_DERG_strategy_id_.data == 3 || _DERG_strategy_id_.data == 5){

            // Orange tube

            // cylinder3 between point link star and applied ref
            p31.x = obj_pose[i].position.x;
            p31.y = obj_pose[i].position.y;
            p31.z = obj_pose[i].position.z;
            p32.x = ref[0][i];
            p32.y = ref[1][i];
            p32.z = ref[2][i];
            InitMarker(marker, "cylinder_strategy_3", i, 10, 0.749f, 0.647f, 0.412f, 0.25, "CylinderShell_10mm");
            CylinderOrientation(p31, p32, cylinder_pose, cylinder_height);
            marker.pose = cylinder_pose;
            marker.scale.x = 0.001*2*future_tubes[i].min_radius;   // radius
            marker.scale.y = 0.001*2*future_tubes[i].min_radius;   // radius
            marker.scale.z = 0.001*cylinder_height;                // height
            all_markers.markers.push_back(marker);

            // hemisphere1 at cylinder ends
            InitMarker(marker, "hemisphere1_strategy_3", i, 10, 0.749f, 0.647f, 0.412f, 0.25, "HemisphereShell_10mm");
            marker.pose.position.x = ref[0][i];
            marker.pose.position.y = ref[1][i];
            marker.pose.position.z = ref[2][i];
            marker.pose.orientation = cylinder_pose.orientation;     
            marker.scale.x = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            marker.scale.y = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            marker.scale.z = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            all_markers.markers.push_back(marker);

            // hemisphere2 at cylinder ends
            InitMarker(marker, "hemisphere2_strategy_3", i, 10, 0.749f, 0.647f, 0.412f, 0.25, "HemisphereShell_10mm");
            marker.pose.position = obj_pose[i].position;
            CylinderOrientation(p32, p31, cylinder_pose, cylinder_height);
            marker.pose.orientation = cylinder_pose.orientation;
            marker.scale.x = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            marker.scale.y = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            marker.scale.z = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            all_markers.markers.push_back(marker);
        }

            //Strategy 4 part
        if(_DERG_strategy_id_.data == 4){
            
            // Orange tube

            // cylinder3 between point link star and applied ref
            p41.x = future_tubes[i].p1.x;
            p41.y = future_tubes[i].p1.y;
            p41.z = future_tubes[i].p1.z;
            p42.x = future_tubes[i].p0.x;
            p42.y = future_tubes[i].p0.y;
            p42.z = future_tubes[i].p0.z;
            InitMarker(marker, "cylinder_strategy_4", i, 10, 0.749f, 0.647f, 0.412f, 0.25, "CylinderShell_10mm");
            CylinderOrientation(p41, p42, cylinder_pose, cylinder_height);
            marker.pose = cylinder_pose;
            marker.scale.x = 0.001*2*future_tubes[i].min_radius;   // Sa_min_perp radius
            marker.scale.y = 0.001*2*future_tubes[i].min_radius;   // Sa_min_perp radius
            marker.scale.z = 0.001*cylinder_height;                // height
            all_markers.markers.push_back(marker);

            // Red hemisphere1 at cylinder ends
            InitMarker(marker, "hemisphere1_strategy_4", i, 10, 0.8f, 0.0f, 0.0f, 0.25, "HemisphereShell_10mm");
            marker.pose.position.x = future_tubes[i].p0.x;
            marker.pose.position.y = future_tubes[i].p0.y;
            marker.pose.position.z = future_tubes[i].p0.z;
            marker.pose.orientation = cylinder_pose.orientation;     
            marker.scale.x = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            marker.scale.y = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            marker.scale.z = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            all_markers.markers.push_back(marker);

            // Red hemisphere2 at cylinder ends
            InitMarker(marker, "hemisphere2_strategy_4", i, 10, 0.8f, 0.0f, 0.0f, 0.25, "HemisphereShell_10mm");
            marker.pose.position.x = future_tubes[i].p1.x;
            marker.pose.position.y = future_tubes[i].p1.y;
            marker.pose.position.z = future_tubes[i].p1.z;
            CylinderOrientation(p42, p41, cylinder_pose, cylinder_height);
            marker.pose.orientation = cylinder_pose.orientation;
            marker.scale.x = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            marker.scale.y = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            marker.scale.z = 0.001*future_tubes[i].min_radius;   // Sa_min_perp radius
            all_markers.markers.push_back(marker);
        }

            //Strategy 5 part
        if(_DERG_strategy_id_.data == 5){

        }

    }

    // red lines between uavs current position
    RedLines(all_markers, marker, obj_pose, number_uav, Ra);

    //Shortest distance line (which is initialy for strategy 5)
    ShortestDistanceLines(all_markers, marker, predicted_trajectories, Ra, number_uav);
    
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