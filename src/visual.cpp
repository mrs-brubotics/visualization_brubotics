#include <visual.h>

void DERGVisualization::addOneSphere(const Eigen::Matrix<double, 3, 1>& sphere_center,
                                    const Eigen::Matrix<double, 1, 1>& sphere_radii,
                                    int id,
                                    const std::string& ns){
  
    visualization_msgs::Marker marker_sphere;
    marker_sphere.header.frame_id = frame_id;
    marker_sphere.header.stamp = ros::Time::now();
    marker_sphere.ns = ns;
    marker_sphere.id = id;
    marker_sphere.type = visualization_msgs::Marker::SPHERE; 
    marker_sphere.action = visualization_msgs::Marker::ADD;
    marker_sphere.pose.position.x = sphere_center(0,0); 
    marker_sphere.pose.position.y = sphere_center(1,0); 
    marker_sphere.pose.position.z = sphere_center(2,0);  
    marker_sphere.pose.orientation.x = 0.0;
    marker_sphere.pose.orientation.y = 0.0;
    marker_sphere.pose.orientation.z = 0.0;
    marker_sphere.pose.orientation.w = 1.0;
    marker_sphere.scale.x = 2*sphere_radii[0]; 
    marker_sphere.scale.y = 2*sphere_radii[0]; 
    marker_sphere.scale.z = 2*sphere_radii[0]; 
    marker_sphere.color.r = color.r;
    marker_sphere.color.g = color.g;
    marker_sphere.color.b = color.b;
    marker_sphere.color.a = color.a;
    marker_sphere.lifetime = ros::Duration();
    marker_array.markers.push_back(marker_sphere);

}

void DERGVisualization::addOneCylinder(const Eigen::Matrix<double, 6, 1>& cylinder_startendpoint,
                                      const Eigen::Matrix<double, 1, 1>& cylinder_radii,
                                      int id,
                                      const std::string& ns){
    visualization_msgs::Marker marker_cylinder;
    Eigen::Matrix<double, 3, 1> cylinder_startpoint = cylinder_startendpoint.block(0,0,3,1);
    Eigen::Matrix<double, 3, 1> cylinder_endpoint = cylinder_startendpoint.block(3,0,3,1);
    Eigen::Matrix<double, 3, 1> cylinder_center = (cylinder_startpoint + cylinder_endpoint)*0.5;


    marker_cylinder.header.frame_id = frame_id;
    marker_cylinder.header.stamp = ros::Time::now();
    marker_cylinder.ns = ns;
    marker_cylinder.id = id;
    marker_cylinder.type = visualization_msgs::Marker::MESH_RESOURCE; 
    marker_cylinder.mesh_resource = "package://visualization_brubotics/meshes/CylinderShell_10mm.stl";
    marker_cylinder.action = visualization_msgs::Marker::ADD;
    marker_cylinder.pose.position.x = cylinder_center[0]; 
    marker_cylinder.pose.position.y = cylinder_center[1];
    marker_cylinder.pose.position.z = cylinder_center[2];


    Eigen::Vector3d cylinder_z_direction(cylinder_startpoint-cylinder_endpoint);
    Eigen::Vector3d origin_z_direction(0., 0., 1.);
    cylinder_z_direction.normalize();
    Eigen::Vector3d axis;
    axis = origin_z_direction.cross(cylinder_z_direction);
    axis.normalize();
    double angle = acos(cylinder_z_direction.dot(origin_z_direction));
    double qx = axis.x() * sin(angle/2);
    double qy = axis.y() * sin(angle/2);
    double qz = axis.z() * sin(angle/2);
    double qw = cos(angle/2);
    double qnorm = sqrt(qx*qx + qy*qy + qz*qz + qw *qw);
    marker_cylinder.pose.orientation.x = qx/qnorm;
    marker_cylinder.pose.orientation.y = qy/qnorm;
    marker_cylinder.pose.orientation.z = qz/qnorm;
    marker_cylinder.pose.orientation.w = qw/qnorm;
    
    marker_cylinder.scale.x = 0.001*2.0*cylinder_radii[0];
    marker_cylinder.scale.y = 0.001*2.0*cylinder_radii[0];
    marker_cylinder.scale.z = 0.001*(cylinder_startpoint - cylinder_endpoint).norm();
    marker_cylinder.color.r = color.r;
    marker_cylinder.color.g = color.g;
    marker_cylinder.color.b = color.b;
    marker_cylinder.color.a = color.a;
    marker_cylinder.lifetime = ros::Duration();
    marker_array.markers.push_back(marker_cylinder);

}

void DERGVisualization::addOneHemisphere(const Eigen::Matrix<double, 6, 1>& cylinder_startendpoint,
                                      const Eigen::Matrix<double, 1, 1>& hemisphere_radii,
                                      int id,
                                      const std::string& ns){
    visualization_msgs::Marker markher_hemisphere;
    Eigen::Matrix<double, 3, 1> cylinder_startpoint = cylinder_startendpoint.block(0,0,3,1);
    Eigen::Matrix<double, 3, 1> cylinder_endpoint = cylinder_startendpoint.block(3,0,3,1);

    markher_hemisphere.header.frame_id = frame_id;
    markher_hemisphere.header.stamp = ros::Time::now();
    markher_hemisphere.ns = ns;
    markher_hemisphere.id = id;
    markher_hemisphere.type = visualization_msgs::Marker::MESH_RESOURCE; 
    markher_hemisphere.mesh_resource = "package://visualization_brubotics/meshes/HemisphereShell_10mm.stl";
    markher_hemisphere.action = visualization_msgs::Marker::ADD;
    markher_hemisphere.pose.position.x = cylinder_startpoint[0]; 
    markher_hemisphere.pose.position.y = cylinder_startpoint[1];
    markher_hemisphere.pose.position.z = cylinder_startpoint[2];

    Eigen::Vector3d cylinder_z_direction(cylinder_startpoint-cylinder_endpoint);
    Eigen::Vector3d origin_z_direction(0., 0., 1.);
    cylinder_z_direction.normalize();
    Eigen::Vector3d axis;
    axis = origin_z_direction.cross(cylinder_z_direction);
    axis.normalize();
    double angle = acos(cylinder_z_direction.dot(origin_z_direction));
    double qx = axis.x() * sin(angle/2);
    double qy = axis.y() * sin(angle/2);
    double qz = axis.z() * sin(angle/2);
    double qw = cos(angle/2);
    double qnorm = sqrt(qx*qx + qy*qy + qz*qz + qw *qw);
    markher_hemisphere.pose.orientation.x = qx/qnorm;
    markher_hemisphere.pose.orientation.y = qy/qnorm;
    markher_hemisphere.pose.orientation.z = qz/qnorm;
    markher_hemisphere.pose.orientation.w = qw/qnorm;
    
    markher_hemisphere.scale.x = 0.001*hemisphere_radii[0];
    markher_hemisphere.scale.y = 0.001*hemisphere_radii[0];
    markher_hemisphere.scale.z = 0.001*hemisphere_radii[0];
    markher_hemisphere.color.r = color.r;
    markher_hemisphere.color.g = color.g;
    markher_hemisphere.color.b = color.b;
    markher_hemisphere.color.a = color.a;
    markher_hemisphere.lifetime = ros::Duration();
    marker_array.markers.push_back(markher_hemisphere);

}
void DERGVisualization::addOneTrajectoryLine(const std::vector<mrs_msgs::FutureTrajectory>& predicted_traj,
                                            int id,
                                            int number_of_point,
                                            const std::string& ns,
                                            double width){
    visualization_msgs::Marker marker_trajectory_l;
    geometry_msgs::Point p_traj_l;
    int step;

    marker_trajectory_l.header.frame_id = frame_id;
    marker_trajectory_l.header.stamp = ros::Time::now();
    marker_trajectory_l.ns = ns;
    marker_trajectory_l.id = id;
    marker_trajectory_l.type = visualization_msgs::Marker::LINE_STRIP; 
    marker_trajectory_l.action = visualization_msgs::Marker::ADD;
    step = predicted_traj[id].points.size() / number_of_point;
    
    for(int j=0; j<(predicted_traj[id].points.size() - step); j+=step){
        p_traj_l.x = (double) predicted_traj[id].points[j].x;
        p_traj_l.y = (double) predicted_traj[id].points[j].y;
        p_traj_l.z = (double) predicted_traj[id].points[j].z;
        marker_trajectory_l.points.push_back(p_traj_l);
    }

    p_traj_l.x = (double) predicted_traj[id].points[predicted_traj[id].points.size()-1].x;
    p_traj_l.y = (double) predicted_traj[id].points[predicted_traj[id].points.size()-1].y;
    p_traj_l.z = (double) predicted_traj[id].points[predicted_traj[id].points.size()-1].z;
    marker_trajectory_l.points.push_back(p_traj_l);

    marker_trajectory_l.scale.x = width; // width of the line strip
    marker_trajectory_l.color.r = color.r;
    marker_trajectory_l.color.g = color.g;
    marker_trajectory_l.color.b = color.b;
    marker_trajectory_l.color.a = color.a;
    marker_trajectory_l.lifetime = ros::Duration();
    marker_array.markers.push_back(marker_trajectory_l);

}
void DERGVisualization::addOneTrajectorySpheres(const std::vector<mrs_msgs::FutureTrajectory>& predicted_traj,
                                            int id,
                                            int number_of_point,
                                            const std::string& ns,
                                            double traj_sphere_radii){
    visualization_msgs::Marker marker_trajectory_s;
    geometry_msgs::Point p_traj_s;
    int step;
    marker_trajectory_s.header.frame_id = frame_id;
    marker_trajectory_s.header.stamp = ros::Time::now();
    marker_trajectory_s.ns = ns;
    marker_trajectory_s.id = id;
    marker_trajectory_s.type = visualization_msgs::Marker::SPHERE_LIST; 
    marker_trajectory_s.action = visualization_msgs::Marker::ADD;
    step = predicted_traj[id].points.size() / number_of_point;
    marker_trajectory_s.pose.position.x = 0;
    marker_trajectory_s.pose.position.y = 0;
    marker_trajectory_s.pose.position.z = 0;
    for(int j=0; j<(predicted_traj[id].points.size() - step); j+=step){
        p_traj_s.x = (double) predicted_traj[id].points[j].x;
        p_traj_s.y = (double) predicted_traj[id].points[j].y;
        p_traj_s.z = (double) predicted_traj[id].points[j].z;
        marker_trajectory_s.points.push_back(p_traj_s);
    }
    p_traj_s.x = (double) predicted_traj[id].points[predicted_traj[id].points.size()-1].x;
    p_traj_s.y = (double) predicted_traj[id].points[predicted_traj[id].points.size()-1].y;
    p_traj_s.z = (double) predicted_traj[id].points[predicted_traj[id].points.size()-1].z;
    marker_trajectory_s.points.push_back(p_traj_s);
    marker_trajectory_s.scale.x = 2*traj_sphere_radii;    // radius of the spheres
    marker_trajectory_s.scale.y = 2*traj_sphere_radii;    // radius
    marker_trajectory_s.scale.z = 2*traj_sphere_radii;    // radius
    marker_trajectory_s.color.r = color.r;
    marker_trajectory_s.color.g = color.g;
    marker_trajectory_s.color.b = color.b;
    marker_trajectory_s.color.a = color.a;
    marker_trajectory_s.lifetime = ros::Duration();
    marker_array.markers.push_back(marker_trajectory_s);

}

void DERGVisualization::addOneTrajectoryArrow(const std::vector<mrs_msgs::FutureTrajectory>& predicted_traj,
                                            const int& id,
                                            const int& number_of_point,
                                            const std::string& ns,
                                            const double& shaft,
                                            const double& head){
    visualization_msgs::Marker marker_trajectory_a;
    geometry_msgs::Point p_traj_a;
    int step;
    int arrow_index=0;
    
    marker_trajectory_a.header.frame_id = frame_id;
    marker_trajectory_a.header.stamp = ros::Time::now();
    marker_trajectory_a.ns = ns;
    marker_trajectory_a.pose.position.x = 0.;
    marker_trajectory_a.pose.position.y = 0.;
    marker_trajectory_a.pose.position.z = 0.;
    marker_trajectory_a.pose.orientation.x = 0.;
    marker_trajectory_a.pose.orientation.y = 0.;
    marker_trajectory_a.pose.orientation.z = 0.;
    marker_trajectory_a.pose.orientation.w = 1.0;

    marker_trajectory_a.color.r = color.r;
    marker_trajectory_a.color.g = color.g;
    marker_trajectory_a.color.b = color.b;
    marker_trajectory_a.color.a = color.a;
    marker_trajectory_a.lifetime = ros::Duration();
    marker_trajectory_a.scale.x = shaft;    // shaft diameter
    marker_trajectory_a.scale.y = head;    // head diameter
            // head length (=0 is default length of an Rviz arrow)

    step = predicted_traj[id].points.size() / number_of_point;
    
    for(int j=0; j<(predicted_traj[id].points.size() - step); j+=2*step){
        marker_trajectory_a.points.clear();
        marker_trajectory_a.id = arrow_index+1000*id;
        arrow_index += 1;
        for(int i=0; i<2; i++){
            p_traj_a.x = predicted_traj[id].points[j+i*step].x;
            p_traj_a.y = predicted_traj[id].points[j+i*step].y;
            p_traj_a.z = predicted_traj[id].points[j+i*step].z;
            marker_trajectory_a.points.push_back(p_traj_a);
        }
        marker_array.markers.push_back(marker_trajectory_a);
    }
}

void DERGVisualization::addCurrentPoseLines(const std::vector<geometry_msgs::Pose>& current_poses,
                                   const int& id,
                                   const std::string& ns,
                                   const double& width,
                                   const double& radius,
                                   const int& number_uav){
    std::vector<geometry_msgs::Point> v(2);
    geometry_msgs::Point p1, p2, p_new;
    double norm;
    for(int i=0; i<(number_uav-1); i++){
        
        for(int j=i+1; j<number_uav; j++){    
            p1.x = current_poses[i].position.x;
            p1.y = current_poses[i].position.y;
            p1.z = current_poses[i].position.z;

            p2.x = current_poses[j].position.x;
            p2.y = current_poses[j].position.y;
            p2.z = current_poses[j].position.z;

            CalculNorm(p1, p2, norm);

            GiveTranslatedPoint(p1,p2,p_new,radius,norm);
            v[0]=p_new;
            GiveTranslatedPoint(p2,p1,p_new,radius,norm);
            v[1]=p_new;

            addLine(v,
                    i*100+j,
                    ns,
                    width);
        }
    }
}

void DERGVisualization::CalculNorm(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, double& norm){
    norm = sqrt(  (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z) );
}

void DERGVisualization::GiveTranslatedPoint(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, geometry_msgs::Point& new_p, const double& distance, double& norm){
    new_p.x = p1.x + distance * (p2.x-p1.x)/norm;
    new_p.y = p1.y + distance * (p2.y-p1.y)/norm;
    new_p.z = p1.z + distance * (p2.z-p1.z)/norm;
}
void DERGVisualization::addLine(std::vector<geometry_msgs::Point> v,
                               const int& id,
                               const std::string& ns,
                               const double& width){
    visualization_msgs::Marker marker_line;

    marker_line.header.frame_id = frame_id;
    marker_line.header.stamp = ros::Time::now();
    marker_line.ns = ns;
    marker_line.id = id;
    marker_line.color.r = color.r;
    marker_line.color.g = color.g;
    marker_line.color.b = color.b;
    marker_line.color.a = color.a;
    marker_line.type = visualization_msgs::Marker::LINE_STRIP; 
    marker_line.action = visualization_msgs::Marker::ADD;
    marker_line.scale.x = width; // width
    for(int i=0; i<v.size(); i++){
        marker_line.points.push_back(v[i]);
    }
    marker_array.markers.push_back(marker_line);
}
void DERGVisualization::ShortestDistanceLines(const std::vector<mrs_msgs::FutureTrajectory>& point,
                                             const int& id,
                                             const std::string& ns,
                                             const double& width,
                                             const double& radius,
                                             const int& number_uav,
                                             const std::vector<float> color_shortest_distance_lines,
                                             const std::vector<float> color_desired_ref_sphere){
    std::vector<geometry_msgs::Point> v;
    double norm_min;
    int ind=0;
    Eigen::Matrix<double, 3, 1> c;
    Eigen::Matrix<double, 1, 1> sphere_radii;
    geometry_msgs::Point p1, p2, p_new1, p_new2;
    
    for(int i=0; i<(number_uav-1); i++){

        for(int j=i+1; j<number_uav; j++){
            // calculate the minimal norm and return index
            CalculNormMin(point,i,j,norm_min,ind);
            p1.x = point[i].points[ind].x;
            p1.y = point[i].points[ind].y;
            p1.z = point[i].points[ind].z;
                
            p2.x = point[j].points[ind].x;
            p2.y = point[j].points[ind].y;
            p2.z = point[j].points[ind].z;
            
            
            // calculate the coordinates of the desired ref position translated by radius Ra
            GiveTranslatedPoint(p1,p2,p_new1,radius,norm_min);
            GiveTranslatedPoint(p2,p1,p_new2,radius,norm_min);

            changeMarkersColor(color_shortest_distance_lines[0], color_shortest_distance_lines[1], color_shortest_distance_lines[2], color_shortest_distance_lines[3]);
            v = {p_new1,p_new2};
            addLine(v,
                    i*100+j,
                    ns,
                    width);
            
            // spheres at desired reference pose
            c = {p1.x,p1.y,p1.z};
            sphere_radii(0) = radius;
            changeMarkersColor(color_desired_ref_sphere[0], color_desired_ref_sphere[1], color_desired_ref_sphere[2], color_desired_ref_sphere[3]);
            addOneSphere(c,
                         sphere_radii,
                         i*100+j,
                         "desired_ref_sphere");
            
            c = {p2.x,p2.y,p2.z};
            addOneSphere(c,
                         sphere_radii,
                         i*100+j+100000,
                         "desired_ref_sphere");
        }
    }
}

void DERGVisualization::CalculNormMin(const std::vector<mrs_msgs::FutureTrajectory>& point,
                                     const int& uav1, 
                                     const int& uav2, 
                                     double& norm_min, 
                                     int& ind){
    double norm;
    geometry_msgs::Point ptest1, ptest2;
    ptest1.x = point[uav1].points[0].x;
    ptest1.y = point[uav1].points[0].y;
    ptest1.z = point[uav1].points[0].z;
                
    ptest2.x = point[uav2].points[0].x;
    ptest2.y = point[uav2].points[0].y;
    ptest2.z = point[uav2].points[0].z;
    CalculNorm(ptest1, ptest2, norm_min);
    for(int j=1; j<point[uav1].points.size(); j++){
            
            ptest1.x = point[uav1].points[j].x;
            ptest1.y = point[uav1].points[j].y;
            ptest1.z = point[uav1].points[j].z;
                
            ptest2.x = point[uav2].points[j].x;
            ptest2.y = point[uav2].points[j].y;
            ptest2.z = point[uav2].points[j].z;
            //ROS_INFO_STREAM("p1 : " << ptest1 << " p2 : " << ptest2);
            CalculNorm(ptest1, ptest2, norm);

            if(norm < norm_min){
                norm_min = norm;
                ind = j;
            }
        }
}

void DERGVisualization::addTextLabel(const Eigen::Matrix<double, 3, 1>& text_position,
                                    const int& id,
                                    const std::string& ns,
                                    const std::string& text,
                                    const double& scalez){
    visualization_msgs::Marker marker_text;
    marker_text.header.frame_id = frame_id;
    marker_text.header.stamp = ros::Time::now();
    marker_text.ns = ns;
    marker_text.id = id;
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
    marker_text.action = visualization_msgs::Marker::ADD;
    marker_text.text = text;
    marker_text.pose.position.x = text_position(0,0); 
    marker_text.pose.position.y = text_position(1,0); 
    marker_text.pose.position.z = text_position(2,0);
    marker_text.scale.z = scalez;
    marker_text.color.r = color.r;
    marker_text.color.g = color.g;
    marker_text.color.b = color.b;
    marker_text.color.a = color.a;
    marker_text.lifetime = ros::Duration();
    marker_array.markers.push_back(marker_text); 
    
}

void DERGVisualization::changeMarkersColor(float r, float g, float b, float a){
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
}

void DERGVisualization::publishMarkers(ros::Publisher publisher){
    publisher.publish(marker_array);
    deleteAllMarkers();
}

DERGVisualization::DERGVisualization(std::string frame){
    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 0.8;
    frame_id = frame;
}

void DERGVisualization::deleteAllMarkers(){
    marker_array.markers.clear();
}