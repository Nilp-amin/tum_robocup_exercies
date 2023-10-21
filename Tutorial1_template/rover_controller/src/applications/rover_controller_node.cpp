#include <ros/ros.h>

#include <rover_controller/controller.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "rover_controller_node");

  rover_controller::Controller controller;

  ros::NodeHandle nh;
  if(!controller.initialize(nh))
  {
    ROS_ERROR_STREAM("rover_controller::Controller failed to initialize");
    return -1;
  }

  ros::Rate rate(60);
  ros::Time time;
  ros::Time prev_time = ros::Time::now();
  ros::Duration duration;

  while(ros::ok())
  {
    time = ros::Time::now();
    duration = time - prev_time;
    prev_time = time;

    controller.update(time, duration);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};