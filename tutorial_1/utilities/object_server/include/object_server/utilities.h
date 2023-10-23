#ifndef object_server_UTILITIES_H_
#define object_server_UTILITIES_H_

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>

namespace object_server
{

  inline visualization_msgs::Marker create_mesh_marker(
    const std::string& mesh_file,
    int id,
    double x=0.0, double y=0.0, double z=0.0,
    double q_w=1.0, double q_x=0.0, double q_y=0.0, double q_z=0.0,
    double scale = 1.0,
    const std::string& ns = "ns")
  {
    visualization_msgs::Marker marker;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = q_x;
    marker.pose.orientation.y = q_y;
    marker.pose.orientation.z = q_z;
    marker.pose.orientation.w = q_w;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.mesh_resource = mesh_file;
    marker.mesh_use_embedded_materials = true;
    return marker;
  }

  inline visualization_msgs::InteractiveMarker create_interactive_marker(
    const std::string& name, 
    const visualization_msgs::Marker& marker,
    const geometry_msgs::Pose& pose)
  {
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.name = name;
    int_marker.description = "interactive_marker";

    visualization_msgs::InteractiveMarkerControl control;
    control.name = "planar";
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    control.markers.push_back(marker);
    control.markers.back().pose = pose;
    int_marker.controls.push_back(control);

    return int_marker;
  }

}

#endif
