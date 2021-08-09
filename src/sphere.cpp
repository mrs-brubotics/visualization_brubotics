#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mrs_msgs/SpawnerDiagnostics.h>
#include <mrs_msgs/FutureTrajectory.h>
#include <mrs_msgs/FuturePoint.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <Eigen/Dense>

#include <ros/ros.h>

#include <iostream>
#include <string>

#define MAX_UAV_NUMBER 10


double Ra = 0.35;
double Sa_ = 1.5;
double marker_radius = 1.5;
bool test1 = false;
std::array<std::array<double, 2>, 3> uav_applied_ref;
// int x_0,y_0,z_0,R;
// std::array<std::array<int, 1000>, 3> points;

// Publishers and subscribers
ros::Publisher marker_pub;
ros::Subscriber diagnostics_sub;
ros::Subscriber DERG_strategy_id_sub;
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

// Hemispheres
//void Sphere_algo(int x, int y, int z, int R);
//int Give_min(int z,int R);
//int Give_max(int z,int R);
//void Circle_sphere(int Rcurrent, int cstmin, int cstmax, int z, int& comp);
void DiagnosticsCallback(const mrs_msgs::SpawnerDiagnostics::ConstPtr& diagnostics);
void CurrentPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg, int uav_number);
void AppliedRefCallback(const mrs_msgs::FutureTrajectory::ConstPtr& msg);
void PointLinkStarCallback(const geometry_msgs::Pose::ConstPtr& msg, int uav_number);
void DERGStrategyIdCallback(const std_msgs::Int32::ConstPtr& msg);
double getDistance(const Point& p1, const Point& p2);
Point getMiddle(const Point& pt1, const Point& pt2);
void CylinderOrientation(const Point &p1,const Point &p2, geometry_msgs::Pose& cylinder_pose, double& cylinder_height);
void PublishMarkers(const std::vector<geometry_msgs::Pose>& obj_pose, std::array<std::array<double, 2>, 3> ref, const std::vector<geometry_msgs::Pose>& pstar);


// Hemispheres
// void Sphere_algo(int x0, int y0, int z0, int R){
//     int i=0;
//     for(int z = 0; z<R; z++){
//         int Rmin = Give_min(z,R);
//         int Rmax = Give_max(z,R);
//         int cstmax = R*R + R - z*z;
//         int cstmin = cstmax - 2*R;
//         for(int Rcurr = Rmin; Rcurr < Rmax; Rcurr++){
//             Circle_sphere(Rcurr,cstmin,cstmax,z,i);
//         }
//     }
// }

// int Give_min(int z,int R){
//     int rmin = R;
//     int Arcmin = R * R - R;
//     int mins = Arcmin - z * z;
//     if(mins < 0){
//         rmin=0;
//     }   
//     else{
//         while(mins < Arcmin){
//             rmin = rmin - 1;
//             Arcmin = Arcmin - 2 * rmin;
//         }
//     }
//     return rmin;
// }

// int Give_max(int z,int R){
//     int rmax = R;
//     int Arcmax = R * R - R;
//     int maxs = Arcmax + 2 * R- z * z;
//     while(maxs < Arcmax){
//         rmax = rmax - 1;
//         Arcmax = Arcmax - 2 * rmax;
//     }
//     return rmax;
// }

// void Circle_sphere(int Rcurrent, int cstmin,int cstmax, int z, int& comp){
//     int x=0;
//     int y=Rcurrent;
//     int delta = Rcurrent;
//     int cst_local_min = Rcurrent * Rcurrent + R - cstmax;
//     int cst_local_max = Rcurrent * Rcurrent + R - cstmin;
//     while(y >= x){
//         if((delta >= cst_local_min) && (delta<cst_local_max)){
//             points[0][comp] = x;
//             points[1][comp] = y;
//             points[2][comp] = z;
//             ROS_INFO_STREAM("point " << comp << " x= " << x << " y= " << y << " z= " << z);
//             comp++;
//         }
//         if(delta > 2*x){
//             delta = delta - 2*x-1;
//             x = x+1;
//         }
//         else if(delta <= 2*(R-y)){
//             delta = delta + 2*y -1;
//             y = y-1;
//         } else{
//             delta = delta + 2*(y-x-1);
//             x=x+1;
//             y=y-1;
//         }
//     }
// }

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
    ROS_INFO_STREAM("UAV" << uav_number << ": " << point_link_stars[uav_number-1].position);
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
    visualization_msgs::Marker error_sphere;
    visualization_msgs::Marker current_pose_marker;
    visualization_msgs::Marker sphere_applied_ref;
    visualization_msgs::Marker cylinder;
    visualization_msgs::Marker cylinder_sphere1;
    visualization_msgs::Marker cylinder_sphere2;
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
        error_sphere.scale.x = 2*Sa_; 
        error_sphere.scale.y = 2*Sa_; 
        error_sphere.scale.z = 2*Sa_; 
        error_sphere.color.r = 0.8f;
        error_sphere.color.g = 0.965f;
        error_sphere.color.b = 1.0f;
        error_sphere.color.a = 0.40;
        error_sphere.lifetime = ros::Duration();
        all_markers.markers.push_back(error_sphere);

        // small sphere at current pose
        // current_pose_marker.header.frame_id = "/common_origin";
        // current_pose_marker.header.stamp = ros::Time::now();
        // current_pose_marker.ns = "current_pose_marker";
        // current_pose_marker.id = i;
        // current_pose_marker.type = visualization_msgs::Marker::SPHERE; 
        // current_pose_marker.action = visualization_msgs::Marker::ADD;
        // current_pose_marker.pose = obj_pose[i];
        // current_pose_marker.scale.x = 2*Ra; 
        // current_pose_marker.scale.y = 2*Ra; 
        // current_pose_marker.scale.z = 2*Ra; 
        // current_pose_marker.color.r = 0.6f;
        // current_pose_marker.color.g = 0.6f;
        // current_pose_marker.color.b = 0.6f;
        // current_pose_marker.color.a = 0.50;
        // current_pose_marker.lifetime = ros::Duration();
        // all_markers.markers.push_back(current_pose_marker);



        // small sphere at applied ref pose
        // sphere_applied_ref.header.frame_id = "/common_origin";
        // sphere_applied_ref.header.stamp = ros::Time::now();
        // sphere_applied_ref.ns = "error_sphere";
        // sphere_applied_ref.id = i;
        // sphere_applied_ref.type = visualization_msgs::Marker::SPHERE; 
        // sphere_applied_ref.action = visualization_msgs::Marker::ADD;
        // sphere_applied_ref.pose.position.x = ref[0][i];
        // sphere_applied_ref.pose.position.y = ref[1][i];
        // sphere_applied_ref.pose.position.z = ref[2][i];
        // sphere_applied_ref.pose.orientation.x = 0;
        // sphere_applied_ref.pose.orientation.y = 0;
        // sphere_applied_ref.pose.orientation.z = 0;
        // sphere_applied_ref.pose.orientation.w = 0.0;
        // sphere_applied_ref.scale.x = 2*Ra; 
        // sphere_applied_ref.scale.y = 2*Ra; 
        // sphere_applied_ref.scale.z = 2*Ra; 
        // sphere_applied_ref.color.r = 0.6f;
        // sphere_applied_ref.color.g = 0.6f;
        // sphere_applied_ref.color.b = 0.6f;
        // sphere_applied_ref.color.a = 0.50;
        // sphere_applied_ref.lifetime = ros::Duration();
        // all_markers.markers.push_back(sphere_applied_ref);

        // cylinder between current pose and applied ref
        Point p1, p2;
        p1.x = pstar[i].position.x;
        p1.y = pstar[i].position.y;
        p1.z = pstar[i].position.z;
        p2.x = ref[0][i];
        p2.y = ref[1][i];
        p2.z = ref[2][i];
        cylinder.header.frame_id = "/common_origin";
        cylinder.id = i;
        cylinder.header.stamp = ros::Time::now();
        cylinder.ns = "cylinder";
        cylinder.type = visualization_msgs::Marker::CYLINDER; 
        cylinder.action = visualization_msgs::Marker::ADD;
        CylinderOrientation(p1, p2, cylinder_pose, cylinder_height);
        cylinder.pose = cylinder_pose;
        cylinder.scale.x = 0.50;            // x radius
        cylinder.scale.y = 0.50;            // y radius
        cylinder.scale.z = cylinder_height; // height
        cylinder.color.r = 0.537f;
        cylinder.color.g = 0.769f;
        cylinder.color.b = 0.956f;
        cylinder.color.a = 0.60;
        all_markers.markers.push_back(cylinder);

        // sphere1 at cylinder ends
        cylinder_sphere1.header.frame_id = "/common_origin";
        cylinder_sphere1.header.stamp = ros::Time::now();
        cylinder_sphere1.ns = "cylinder_sphere1";
        cylinder_sphere1.id = i;
        cylinder_sphere1.type = visualization_msgs::Marker::SPHERE; 
        cylinder_sphere1.action = visualization_msgs::Marker::ADD;
        cylinder_sphere1.pose = obj_pose[i];
        cylinder_sphere1.scale.x = 0.50; 
        cylinder_sphere1.scale.y = 0.50; 
        cylinder_sphere1.scale.z = 0.50; 
        cylinder_sphere1.color.r = 0.537f;
        cylinder_sphere1.color.g = 0.769f;
        cylinder_sphere1.color.b = 0.956f;
        cylinder_sphere1.color.a = 0.60;
        cylinder_sphere1.lifetime = ros::Duration();
        all_markers.markers.push_back(cylinder_sphere1);

        // sphere2 at cylinder ends
        cylinder_sphere2.header.frame_id = "/common_origin";
        cylinder_sphere2.header.stamp = ros::Time::now();
        cylinder_sphere2.ns = "cylinder_sphere2";
        cylinder_sphere2.id = i;
        cylinder_sphere2.type = visualization_msgs::Marker::SPHERE; 
        cylinder_sphere2.action = visualization_msgs::Marker::ADD;
        cylinder_sphere2.pose.position.x = ref[0][i];
        cylinder_sphere2.pose.position.y = ref[1][i];
        cylinder_sphere2.pose.position.z = ref[2][i];
        cylinder_sphere2.pose.orientation.x = 0;
        cylinder_sphere2.pose.orientation.y = 0;
        cylinder_sphere2.pose.orientation.z = 0;
        cylinder_sphere2.pose.orientation.w = 0.0;        
        cylinder_sphere2.scale.x = 0.50; 
        cylinder_sphere2.scale.y = 0.50; 
        cylinder_sphere2.scale.z = 0.50; 
        cylinder_sphere2.color.r = 0.537f;
        cylinder_sphere2.color.g = 0.769f;
        cylinder_sphere2.color.b = 0.956f;
        cylinder_sphere2.color.a = 0.60;
        cylinder_sphere2.lifetime = ros::Duration();
        all_markers.markers.push_back(cylinder_sphere2);

        // ends points of the line
        p.x = ref[0][i];
        p.y = ref[1][i];
        p.z = ref[2][i];
        line.points.push_back(p);
    }

    // line betweetn both current uav pose
    line.header.frame_id = "/common_origin";
    line.id = 2;
    line.header.stamp = ros::Time::now();
    line.ns = "line";
    line.type = visualization_msgs::Marker::LINE_STRIP; 
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 0.0;
    line.scale.x = 0.05; // width
    line.color.r = 0.0f;
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
    ros::init(argc, argv,"current_pose_marker");
    ros::NodeHandle n;
    ros::Rate r(30);

    // Subscribers and publishers //
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // create subscriber for active UAV list
    diagnostics_sub = n.subscribe("mrs_drone_spawner/diagnostics", 1, DiagnosticsCallback);
    DERG_strategy_id_sub = n.subscribe("uav1/control_manager/dergbryan_tracker/derg_strategy_id", 1, DERGStrategyIdCallback);

    //Sphere_algo(0, 0, 0, 10);

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
        if(_DERG_strategy_id_.data == 0){
            PublishMarkers(uav_current_poses, uav_applied_ref, point_link_stars);
        }
        else if(_DERG_strategy_id_.data == 1){
            PublishMarkers(uav_current_poses, uav_applied_ref, point_link_stars);
        }                    
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}