#include <object_server/object_server.h>

namespace object_server
{

  ObjectServer::ObjectServer() : 
    server_("object_server"),
    object_count_(0)
  {
  }

  ObjectServer::~ObjectServer()
  {
  }

  bool ObjectServer::initialize(ros::NodeHandle &nh)
  {
    return true;
  }

  bool ObjectServer::add(
    const std::string& name,
    const visualization_msgs::Marker& marker,
    const geometry_msgs::Pose& pose)
  {
    // create and insert the marker to the server, bind its callback function
    visualization_msgs::InteractiveMarker int_marker = 
      object_server::create_interactive_marker(name, marker, pose);
    int_marker.header = marker.header;
    server_.insert(int_marker, boost::bind(&ObjectServer::interactiveMarkerFeedback, this, _1, object_count_));

    // update the inital pose
    server_.setPose(name, pose, marker.header);
    server_.applyChanges();
    poses_.push_back(pose);

    // store the id
    object_count_++;
    return true;
  }

  bool ObjectServer::clear()
  {
    object_count_ = 0;
    poses_.clear();
    server_.clear();
    return true;
  }

  void ObjectServer::interactiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg, 
    size_t obj_idx)
  {
    // update the pose of the object
    poses_[obj_idx] = msg->pose;

    // call the derived feedback funtion
    feedback(poses_, obj_idx);
  }

}