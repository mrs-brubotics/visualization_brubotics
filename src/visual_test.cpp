#include <visual.h>

#define MAX_UAV_NUMBER 10       // Maximum number of UAVs for the visualization, used to initialized vectors and arrays


// | ------------------------ Publishers and subscribers ----------------------- |

ros::Publisher marker_array_pub;

// Subscribers
ros::Subscriber diagnostics_subscriber_;
ros::Subscriber DERG_strategy_id_subscriber_;
ros::Subscriber Sa_subscriber_;             
ros::Subscriber Sa_perp_subscriber_;
ros::Subscriber tube_min_radius_subscriber_;
std::vector<ros::Subscriber> goal_pose_subscribers_(MAX_UAV_NUMBER);
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
std::vector<geometry_msgs::PoseStamped> frame_pose(MAX_UAV_NUMBER);
std::vector<mrs_msgs::Reference> goal_pose(MAX_UAV_NUMBER);

// | -------------------------------- Parameters -------------------------------- |

const std::string empty = std::string();
bool test1 = false;
bool test2 = false;
int step;           // displayed trajectory points step
double Ra = 0.35;   // drone's radius
int number_of_uav;
std::array<std::array<double, 2>, 3> uav_applied_ref;
std::array<std::array<std::array<double, 2>, 3>, 1000> predicted_trajectories;
int number_of_point_traj;
std::string s_label;
std::vector<float> color_current_pose_sphere(4);
std::vector<float> color_applied_ref_sphere(4);
std::vector<float> color_shortest_distance_lines(4);
std::vector<float> color_trajectory(4);
std::vector<float> color_red_lines(4);
std::vector<float> color_desired_ref_sphere(4);
std::vector<float> color_error_sphere(4);
std::vector<float> color_strategy_1_cylinder_strategy_1(4);
std::vector<float> color_strategy_2_3_4_cylinder_strategy_1(4);
std::vector<float> color_cylinder_strategy_2(4);
std::vector<float> color_cylinder_strategy_3(4);
std::vector<float> color_cylinder_strategy_4(4);
std::vector<float> color_hemisphere1_strategy_4(4);
std::vector<float> color_hemisphere2_strategy_4(4);
std::vector<float> UAV_text_label(4);

ERGVisualization visualisation_rviz("/common_origin"); //map is the frame in which you want to add your objects it is common_origin for the derg program

int main(int argc, char** argv){

    ros::init(argc, argv, "derg_vis_test");
    ros::NodeHandle n;
    ros::Rate r(30);

    ros::NodeHandle nh;

    
    // Subscribers and publishers
    //Publishers
    marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_brubotics/markers", 10);

    // Subscribers
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

    // Load parameters from visual.yaml file
    nh.getParam("/visualization_brubotics/NUMBER_OF_POINTS_TRAJECTORY", number_of_point_traj);
    nh.getParam("/visualization_brubotics/all_strategies/current_pose_sphere/r", color_current_pose_sphere[0]);
    nh.getParam("/visualization_brubotics/all_strategies/current_pose_sphere/g", color_current_pose_sphere[1]);
    nh.getParam("/visualization_brubotics/all_strategies/current_pose_sphere/b", color_current_pose_sphere[2]);
    nh.getParam("/visualization_brubotics/all_strategies/current_pose_sphere/alpha", color_current_pose_sphere[3]);
    nh.getParam("/visualization_brubotics/all_strategies/applied_ref_sphere/r", color_applied_ref_sphere[0]);
    nh.getParam("/visualization_brubotics/all_strategies/applied_ref_sphere/g", color_applied_ref_sphere[1]);
    nh.getParam("/visualization_brubotics/all_strategies/applied_ref_sphere/b", color_applied_ref_sphere[2]);
    nh.getParam("/visualization_brubotics/all_strategies/applied_ref_sphere/alpha", color_applied_ref_sphere[3]);   
    nh.getParam("/visualization_brubotics/all_strategies/trajectory/r", color_trajectory[0]);
    nh.getParam("/visualization_brubotics/all_strategies/trajectory/g", color_trajectory[1]);
    nh.getParam("/visualization_brubotics/all_strategies/trajectory/b", color_trajectory[2]);
    nh.getParam("/visualization_brubotics/all_strategies/trajectory/alpha", color_trajectory[3]);
    nh.getParam("/visualization_brubotics/strategy_0/error_sphere/r", color_error_sphere[0]);
    nh.getParam("/visualization_brubotics/strategy_0/error_sphere/g", color_error_sphere[1]);
    nh.getParam("/visualization_brubotics/strategy_0/error_sphere/b", color_error_sphere[2]);
    nh.getParam("/visualization_brubotics/strategy_0/error_sphere/alpha", color_error_sphere[3]);
    nh.getParam("/visualization_brubotics/all_strategies/red_lines/r", color_red_lines[0]);
    nh.getParam("/visualization_brubotics/all_strategies/red_lines/g", color_red_lines[1]);
    nh.getParam("/visualization_brubotics/all_strategies/red_lines/b", color_red_lines[2]);
    nh.getParam("/visualization_brubotics/all_strategies/red_lines/alpha", color_red_lines[3]);
    nh.getParam("/visualization_brubotics/all_strategies/shortest_distance_lines/r", color_shortest_distance_lines[0]);
    nh.getParam("/visualization_brubotics/all_strategies/shortest_distance_lines/g", color_shortest_distance_lines[1]);
    nh.getParam("/visualization_brubotics/all_strategies/shortest_distance_lines/b", color_shortest_distance_lines[2]);
    nh.getParam("/visualization_brubotics/all_strategies/shortest_distance_lines/alpha", color_shortest_distance_lines[3]);
    nh.getParam("/visualization_brubotics/all_strategies/desired_ref_sphere/r", color_desired_ref_sphere[0]);
    nh.getParam("/visualization_brubotics/all_strategies/desired_ref_sphere/g", color_desired_ref_sphere[1]);
    nh.getParam("/visualization_brubotics/all_strategies/desired_ref_sphere/b", color_desired_ref_sphere[2]);
    nh.getParam("/visualization_brubotics/all_strategies/desired_ref_sphere/alpha", color_desired_ref_sphere[3]);
    nh.getParam("/visualization_brubotics/strategy_1_tube/strategy_1/cylinder_strategy_1/r", color_strategy_1_cylinder_strategy_1[0]);
    nh.getParam("/visualization_brubotics/strategy_1_tube/strategy_1/cylinder_strategy_1/g", color_strategy_1_cylinder_strategy_1[1]);
    nh.getParam("/visualization_brubotics/strategy_1_tube/strategy_1/cylinder_strategy_1/b", color_strategy_1_cylinder_strategy_1[2]);
    nh.getParam("/visualization_brubotics/strategy_1_tube/strategy_1/cylinder_strategy_1/alpha", color_strategy_1_cylinder_strategy_1[3]);
    nh.getParam("/visualization_brubotics/strategy_1_tube/strategy_2_3_4/cylinder_strategy_1/r", color_strategy_2_3_4_cylinder_strategy_1[0]);
    nh.getParam("/visualization_brubotics/strategy_1_tube/strategy_2_3_4/cylinder_strategy_1/g", color_strategy_2_3_4_cylinder_strategy_1[1]);
    nh.getParam("/visualization_brubotics/strategy_1_tube/strategy_2_3_4/cylinder_strategy_1/b", color_strategy_2_3_4_cylinder_strategy_1[2]);
    nh.getParam("/visualization_brubotics/strategy_1_tube/strategy_2_3_4/cylinder_strategy_1/alpha", color_strategy_2_3_4_cylinder_strategy_1[3]);
    nh.getParam("/visualization_brubotics/strategy_2_tube/cylinder_strategy_2/r", color_cylinder_strategy_2[0]);
    nh.getParam("/visualization_brubotics/strategy_2_tube/cylinder_strategy_2/g", color_cylinder_strategy_2[1]);
    nh.getParam("/visualization_brubotics/strategy_2_tube/cylinder_strategy_2/b", color_cylinder_strategy_2[2]);
    nh.getParam("/visualization_brubotics/strategy_2_tube/cylinder_strategy_2/alpha", color_cylinder_strategy_2[3]);
    nh.getParam("/visualization_brubotics/strategy_3_tube/cylinder_strategy_3/r", color_cylinder_strategy_3[0]);
    nh.getParam("/visualization_brubotics/strategy_3_tube/cylinder_strategy_3/g", color_cylinder_strategy_3[1]);
    nh.getParam("/visualization_brubotics/strategy_3_tube/cylinder_strategy_3/b", color_cylinder_strategy_3[2]);
    nh.getParam("/visualization_brubotics/strategy_3_tube/cylinder_strategy_3/alpha", color_cylinder_strategy_3[3]);
    nh.getParam("/visualization_brubotics/strategy_4_tube/cylinder_strategy_4/r", color_cylinder_strategy_4[0]);
    nh.getParam("/visualization_brubotics/strategy_4_tube/cylinder_strategy_4/g", color_cylinder_strategy_4[1]);
    nh.getParam("/visualization_brubotics/strategy_4_tube/cylinder_strategy_4/b", color_cylinder_strategy_4[2]);
    nh.getParam("/visualization_brubotics/strategy_4_tube/cylinder_strategy_4/alpha", color_cylinder_strategy_4[3]);
    nh.getParam("/visualization_brubotics/strategy_4_tube/hemisphere1_strategy_4/r", color_hemisphere1_strategy_4[0]);
    nh.getParam("/visualization_brubotics/strategy_4_tube/hemisphere1_strategy_4/g", color_hemisphere1_strategy_4[1]);
    nh.getParam("/visualization_brubotics/strategy_4_tube/hemisphere1_strategy_4/b", color_hemisphere1_strategy_4[2]);
    nh.getParam("/visualization_brubotics/strategy_4_tube/hemisphere1_strategy_4/alpha", color_hemisphere1_strategy_4[3]);
    nh.getParam("/visualization_brubotics/strategy_4_tube/hemisphere2_strategy_4/r", color_hemisphere2_strategy_4[0]);
    nh.getParam("/visualization_brubotics/strategy_4_tube/hemisphere2_strategy_4/g", color_hemisphere2_strategy_4[1]);
    nh.getParam("/visualization_brubotics/strategy_4_tube/hemisphere2_strategy_4/b", color_hemisphere2_strategy_4[2]);
    nh.getParam("/visualization_brubotics/strategy_4_tube/hemisphere2_strategy_4/alpha", color_hemisphere2_strategy_4[3]);
    nh.getParam("/visualization_brubotics/all_strategies/UAV_text_label/r", UAV_text_label[0]);
    nh.getParam("/visualization_brubotics/all_strategies/UAV_text_label/g", UAV_text_label[1]);
    nh.getParam("/visualization_brubotics/all_strategies/UAV_text_label/b", UAV_text_label[2]);
    nh.getParam("/visualization_brubotics/all_strategies/UAV_text_label/alpha", UAV_text_label[3]);


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
    std::vector<boost::function<void (const mrs_msgs::ReferenceStamped::ConstPtr&)>> f6;
    f6.resize(MAX_UAV_NUMBER);

    for(int i=0; i<diagnostics.active_vehicles.size(); i++){

        // create subscribers for current uav pose
        f1[i] = boost::bind(CurrentPoseCallback, _1, i+1);
        uav_current_pose_subscribers_[i] = n.subscribe(diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/custom_predicted_poses", 1, f1[i]);

        // create subscribers for uav applied ref
        f2[i] = boost::bind(AppliedRefCallback, _1, i+1);
        uav_applied_ref_subscribers_[i] = n.subscribe(diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/uav_applied_ref", 1, f2[i]);

        // create subscribers for uav point link star
        f3[i] = boost::bind(PointLinkStarCallback, _1, i+1);
        point_link_star_subscribers_[i] = n.subscribe(diagnostics.active_vehicles[i] +"/control_manager/dergbryan_tracker/point_link_star", 1, f3[i]);

        // create subscribers for uav predicted trajectory
        f4[i] = boost::bind(PredictedTrajectoryCallback, _1, i+1);
        predicted_trajectory_subscribers_[i] = n.subscribe(diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/predicted_trajectory", 1, f4[i]);

        // create subscribers for uav future tube
        f5[i] = boost::bind(FutureTubeCallback, _1, i+1);
        future_tube_subscribers_[i] = n.subscribe(diagnostics.active_vehicles[i] +"/control_manager/dergbryan_tracker/future_trajectory_tube", 1, f5[i]);

        // create subscribers for uav future tube
        f6[i] = boost::bind(GoalPoseCallback, _1, i+1);
        goal_pose_subscribers_[i] = n.subscribe(diagnostics.active_vehicles[i] + "/control_manager/dergbryan_tracker/goal_pose", 1, f6[i]);
    }   

    while(!test2){
        ros::spinOnce();
        r.sleep();
    }

    Eigen::Matrix<double, 1, 1> sphere_radii;
    Eigen::Matrix<double, 3, 1> c;
    Eigen::Matrix<double, 6, 1> cylinder_startendpoints;
    Eigen::Matrix<double, 1, 1> cylinder_radii;

    while(ros::ok()){

        for(uint i = 0; i < number_of_uav; i++){
            c = {uav_current_poses[i].position.x, 
                 uav_current_poses[i].position.y, 
                 uav_current_poses[i].position.z};
            sphere_radii(0) = 2*Ra;
            visualisation_rviz.changeMarkersColor(color_current_pose_sphere[0], 
                                                  color_current_pose_sphere[1], 
                                                  color_current_pose_sphere[2], 
                                                  color_current_pose_sphere[3]);
            visualisation_rviz.addOneSphere(c, 
                                            sphere_radii, 
                                            i, 
                                            "current_pose_sphere");
            
            c = {uav_applied_ref[0][i], 
                 uav_applied_ref[1][i], 
                 uav_applied_ref[2][i]};
            sphere_radii(0) = 2*Ra;
            visualisation_rviz.changeMarkersColor(color_applied_ref_sphere[0], 
                                                  color_applied_ref_sphere[1], 
                                                  color_applied_ref_sphere[2], 
                                                  color_applied_ref_sphere[3]);
            visualisation_rviz.addOneSphere(c, 
                                            sphere_radii, 
                                            i, 
                                            "applied_ref_sphere");

            cylinder_startendpoints.block(0,0,3,1) = Eigen::Matrix<double, 3, 1> {point_link_stars[i].position.x,
                                                                                point_link_stars[i].position.y,
                                                                                point_link_stars[i].position.z};
            cylinder_startendpoints.block(3,0,3,1) = Eigen::Matrix<double, 3, 1> {uav_applied_ref[0][i],
                                                                                uav_applied_ref[1][i],
                                                                                uav_applied_ref[2][i]};
            cylinder_radii(0) = Sa_min_perp.data;
            visualisation_rviz.changeMarkersColor(color_applied_ref_sphere[0], color_applied_ref_sphere[1], color_applied_ref_sphere[2], color_applied_ref_sphere[3]);
            visualisation_rviz.addOneCylinder(cylinder_startendpoints,
                                          cylinder_radii,
                                          i,
                                          "cylinder_strategy_1");

        }
        ros::spinOnce();
        visualisation_rviz.publishMarkers(marker_array_pub);
        r.sleep();
    }
    ros::shutdown();
    
}

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

void GoalPoseCallback(const mrs_msgs::ReferenceStamped::ConstPtr& msg, int uav_number){
    frame_pose[uav_number-1].header = msg -> header;
    goal_pose[uav_number-1] = msg -> reference;
    // ROS_INFO_STREAM("goal pose : " << goal_pose[uav_number-1]);
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