#ifndef ROVER_ROVER_H_
#define ROVER_ROVER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

// ros msgs
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

namespace rover
{

  class Rover
  {
  private:
    ros::Subscriber command_sub_;
    ros::Publisher rover_marker_pub_; 
    visualization_msgs::Marker rover_marker_;

    tf::TransformBroadcaster broadcaster_;

    Eigen::Vector3d thrust_b_;
    Eigen::Vector3d pos_b_w_, vel_b_;

    Eigen::Matrix3d M_, D_, K_;

  public:
    Rover();
    virtual ~Rover();

    bool initialize(ros::NodeHandle& nh);

    void update(ros::Time& time, ros::Duration& dt);

  private:
    Eigen::Vector3d dynamic(const Eigen::Vector3d& thrust) const;

  private:
    void commandCallback(const geometry_msgs::TwistConstPtr& msg);

  };

}

#endif