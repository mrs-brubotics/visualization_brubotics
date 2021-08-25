#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <mrs_msgs/SpawnerDiagnostics.h>
#include <mrs_msgs/FutureTrajectory.h>
#include <mrs_msgs/FuturePoint.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <trackers_brubotics/FutureTrajectoryTube.h> // custom ROS message
#include <mrs_lib/param_loader.h>

#include <string>
#include <iostream>

class ERGVisualization{
    private:
        visualization_msgs::MarkerArray marker_array;
        std::string frame_id;
        std_msgs::ColorRGBA color;
    public:
        ERGVisualization(std::string frame);
        void addOneSphere(const Eigen::Matrix<double, 3, 1>& sphere_center,
                          const Eigen::Matrix<double, 1, 1>& sphere_radii, 
                          int id, 
                          const std::string& ns);
        void addOneCylinder(const Eigen::Matrix<double, 6, 1>& cylinder_startendpoint,
                            const Eigen::Matrix<double, 1, 1>& cylinder_radii,
                            int id,
                            const std::string& ns);
        void publishMarkers(ros::Publisher);

        void changeMarkersColor(float r, float g, float b, float a);
                
        void deleteAllMarkers();


};

// Callbacks
void DiagnosticsCallback(const mrs_msgs::SpawnerDiagnostics::ConstPtr& diagnostics);
void DERGStrategyIdCallback(const std_msgs::Int32::ConstPtr& msg);
void PredictedTrajectoryCallback(const mrs_msgs::FutureTrajectory::ConstPtr& msg, int uav_number);
void GoalPoseCallback(const mrs_msgs::ReferenceStamped::ConstPtr& msg, int uav_number);

void CurrentPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg, int uav_number);
void AppliedRefCallback(const mrs_msgs::FutureTrajectory::ConstPtr& msg, int uav_number);
void PointLinkStarCallback(const geometry_msgs::Pose::ConstPtr& msg, int uav_number);
void FutureTubeCallback(const trackers_brubotics::FutureTrajectoryTube::ConstPtr& msg, int uav_number);

void SaCallback(const std_msgs::Int32::ConstPtr& msg);
void SaPerpCallback(const std_msgs::Int32::ConstPtr& msg);
void TubeMinRadiusCallback(const std_msgs::Float32::ConstPtr& msg);