#ifndef ROVER_CONTROLLER_H_
#define ROVER_CONTROLLER_H_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <object_msgs/Objects.h>

#include <eigen3/Eigen/Dense>

#include <tf/transform_listener.h>

namespace rover_controller
{

  struct Obstacle
  {
    Eigen::Vector3d pos;
    double radius;
  };

  class Controller
  {
    private:
      std::string map_frame_;
      std::string rover_frame_;

      bool has_goal_;
      double rover_radius_;

      ros::Time prev_time_;

      // ros connections
      ros::Publisher command_pub_;
      ros::Publisher field_pub_;

      ros::Subscriber goal_sub_;
      ros::Subscriber obstacles_sub_;
      tf::TransformListener listener_;

      // gains
      Eigen::Matrix3d K_attractor_;
      double k_repulsive_;
      double lambda_repulsive_;
      double k_vortex_;
      double lambda_vortex_;

      // inputs
      Eigen::Vector3d goal_pos_w_;
      std::vector<Obstacle> obstacles_;

      std::vector<Eigen::Vector3d> field_points_w_;

    public:
      Controller();

      virtual ~Controller();

      bool initialize(ros::NodeHandle& nh);

      void update(const ros::Time& time, const ros::Duration durtation);

    private:
      Eigen::Vector3d computeForce(
        std::vector<Obstacle> obstacles, const Eigen::Vector3d& goal, 
        const Eigen::Vector3d& pos);

      Eigen::Vector3d goalAttractorForce(
        const Eigen::Vector3d& goal, const Eigen::Vector3d& pos);

      Eigen::Vector3d obstacleRepulsiveForce(
        std::vector<Obstacle> obstacles, const Eigen::Vector3d& pos);

      Eigen::Vector3d obstacleVortexForce(
        std::vector<Obstacle> obstacles, const Eigen::Vector3d& pos);

      Eigen::Vector3d roverPos();

      void publishCommand(const Eigen::Vector3d& thrust);

    private:
      void createVectorField(std::vector<Eigen::Vector3d>& points);

      void visualizerVectorField(
        std::vector<Obstacle> obstacles, 
        const Eigen::Vector3d& goal,
        const std::vector<Eigen::Vector3d>& points);

    private:
      void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

      void obstacleCallback(const object_msgs::ObjectsConstPtr& msg);
  };
}

#endif