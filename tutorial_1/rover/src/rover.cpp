#include <rover/rover.h>

#include <object_server/utilities.h>

namespace rover
{

  Rover::Rover()
  {
  }

  Rover::~Rover()
  {
  }

  bool Rover::initialize(ros::NodeHandle& nh)
  {
    // ros connections
    command_sub_ = nh.subscribe("/key_vel", 1, &Rover::commandCallback, this);
    
    rover_marker_pub_ = nh.advertise<visualization_msgs::Marker>("rover", 10);
    
    // init member variables
    pos_b_w_.setZero();
    vel_b_.setZero();
    thrust_b_.setZero();

    // create the marker for visualizing the rover
    rover_marker_ = object_server::create_mesh_marker(
      "package://rover/launch/meshes/preserverance/perseverance.dae",
      1,
      0, 0, 0,
      0.7045553,0, 0, 0.7096491,
      0.3);
    rover_marker_.header.frame_id = "rover";
    rover_marker_.header.stamp = ros::Time::now();

    // get the parameters of this node (see config.yaml)
    std::vector<double> v = {0,0,0};

    if(!ros::param::get("mass", v))
      return false;
    M_ = Eigen::Vector3d(v[0], v[1], v[2]).asDiagonal();

    if(!ros::param::get("damping", v))
      return false;
    D_ = Eigen::Vector3d(v[0], v[1], v[2]).asDiagonal();

    if(!ros::param::get("force_gain", v))
      return false;
    K_ = Eigen::Vector3d(v[0], v[1], v[2]).asDiagonal();

    return true;
  }

  void Rover::update(ros::Time& time, ros::Duration& duration)
  {
    double dt = duration.toSec();

    // rotation of the rovers body {b} with respect to the world/map {w}
    Eigen::Matrix3d R_b_w = Eigen::Matrix3d::Identity();
    R_b_w.topLeftCorner(2,2) = Eigen::Rotation2Dd(pos_b_w_.z()).matrix();

    // obtain the cooresponding from the given thrust
    Eigen::Vector3d acc_b = dynamic(thrust_b_);

    // rotation matrix of rover frame relative to map frame
    Eigen::Matrix3d R_rover_map;
    R_rover_map = Eigen::AngleAxisd(pos_b_w_.z(), Eigen::Vector3d::UnitZ());

    // integrate velocity into position
    pos_b_w_ += R_rover_map*vel_b_*dt + 0.5*acc_b*dt*dt;

    // integrate acceleration into velocity
    vel_b_ += acc_b*dt;

    // update the tf transformation
    tf::StampedTransform T_rover_map;
    T_rover_map.setOrigin(tf::Vector3(pos_b_w_.x(), pos_b_w_.y(), 0.0));

    tf::Quaternion Q_rover_map;
    Q_rover_map.setRPY(0.0, 0.0, pos_b_w_.z());
    T_rover_map.setRotation(Q_rover_map);

    // broadcast the transformation
    T_rover_map.child_frame_id_ = "rover";
    T_rover_map.frame_id_ = "map";
    T_rover_map.stamp_ = time;
    broadcaster_.sendTransform(T_rover_map);

    // publish the visualization
    rover_marker_.header.stamp = time;
    rover_marker_pub_.publish(rover_marker_);
  }

  Eigen::Vector3d Rover::dynamic(const Eigen::Vector3d& thrust) const
  {
    return M_.inverse() * (K_ * thrust - D_ * vel_b_);
  }

  void Rover::commandCallback(const geometry_msgs::TwistConstPtr& msg)
  {
    thrust_b_ = Eigen::Vector3d(msg->linear.x, msg->linear.y, msg->angular.z);
  }
}