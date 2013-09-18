#include <ros/ros.h>

int 
main(int argc, char** argv)
{ 
  ros::init(argc, argv, "stageros");
  ros::Rate rate(10);
  while(ros::ok())
  {
    rate.sleep();
  }
  return 0;
}