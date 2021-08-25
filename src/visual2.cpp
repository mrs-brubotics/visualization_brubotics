#include <visual.h>

void ERGVisualization::addOneSphere(const Eigen::Matrix<double, 3, 1>& sphere_center,
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

void ERGVisualization::addOneCylinder(const Eigen::Matrix<double, 6, 1>& cylinder_startendpoint,
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

void ERGVisualization::changeMarkersColor(float r, float g, float b, float a){
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
}

void ERGVisualization::publishMarkers(ros::Publisher publisher){
    publisher.publish(marker_array);
    deleteAllMarkers();
}

ERGVisualization::ERGVisualization(std::string frame){
    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 0.8;
    frame_id = frame;
}

void ERGVisualization::deleteAllMarkers(){
    marker_array.markers.clear();
}