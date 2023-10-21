#include <ros/ros.h>

#include <object_server/mars_environment.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "object_server");

  object_server::MarsEnvironment mars;

  ros::NodeHandle nh;
  mars.initialize(nh);

  ros::Rate rate(60);
  while(ros::ok())
  {
    mars.update(ros::Time::now());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};