#include <ros/ros.h>

#include <rover/rover.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "rover_node");

  rover::Rover rover;

  ros::NodeHandle nh;
  if (!rover.initialize(nh))
  {
    ros::shutdown();
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

    rover.update(time, duration);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};