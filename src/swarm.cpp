#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>

#include <cmath> //for sqrt and atan2 functions
#include <tf/tf.h> //to transform quaternion into euler
#include <cstdlib> //for random number functions

//global variables to store the last received range readings and odom
sensor_msgs::RangeConstPtr lastF;
sensor_msgs::RangeConstPtr lastL;
sensor_msgs::RangeConstPtr lastR;
nav_msgs::OdometryConstPtr lastOdomReading;

//callback functions to process data from subscribed range and topics
void rangeF_received(sensor_msgs::RangeConstPtr msg){
  lastF = msg;
}
void rangeL_received(sensor_msgs::RangeConstPtr msg){
  lastL = msg;
}
void rangeR_received(sensor_msgs::RangeConstPtr msg){
  lastR = msg;
}
void odometryReceived(nav_msgs::OdometryConstPtr msg){
  lastOdomReading = msg;
}

//main function of the node
void swarmrobot(){
  //creating node handles for subscription/publishing and parameter server queries
  ros::NodeHandle node;
  ros::NodeHandle pnode("~"); //used to get private parameters ~x, ~y and ~th

  //subscribing to odometry and ranges, announcing published topics
  ros::Subscriber sub = node.subscribe<nav_msgs::Odometry>("odom",10, odometryReceived);
  ros::Subscriber subF = node.subscribe<sensor_msgs::Range>("range_f",10, rangeF_received);
  ros::Subscriber subL = node.subscribe<sensor_msgs::Range>("range_l",10, rangeL_received);
  ros::Subscriber subR = node.subscribe<sensor_msgs::Range>("range_r",10, rangeR_received);
  ros::Publisher pub = node.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 1, false);

  ros::Rate r(10); //an object to maintain specific frequency of a control loop - 10hz

  geometry_msgs::TwistStamped cmd; //command that will be sent to Stage (published)

  while(ros::ok())
  {
    //required in C++ clients to receive messages from subscribed topics
    //note: Python clients don't need this call
    ros::spinOnce();

    if (!lastOdomReading || !lastF || !lastL || !lastR ){//we cannot issue any commands until we have our position
      std::cout<<"waiting for ranges and odom to become available"<<std::endl;
      r.sleep();
      continue;
    }

    // default speeds,
    cmd.twist.linear.x = 0; // forward
    cmd.twist.linear.z = 0; // up/down
    cmd.twist.angular.z = 0; // left/right

    //BEHAVIORS CODE (BEGIN)
    // INPUT:
    //   Only the following ranges: lastL, lastR, lastF, and Up/Down ranges (needs additional subscribers, etc).
    //   You are also allowed to use depth(lastOdomReading->pose.pose.position.z) and compass
    //   (lastOdomReading->pose.pose.orientation, see Closed-loop tutorial how to extract heading from it)
    //
    // OUTPUT:
    //   Only Thrust(cmd.twist.linear.x), Buoyance(cmd.twist.linear.z) and Turning(cmd.twist.angular.z)

    //Obstacle Avoidance behavior
    double OAforward = fmax(0,lastF->range - 0.5);
    double OAupdown = 0;
    double OAleftright = lastL->range - lastR->range;
    //Random behavior
    double RNDforward = 0;
    double RNDupdown = 0;
    double RNDleftright = ((double)rand()/(double)RAND_MAX)*1.0 - 0.5; //random number in the range [-0.5, 0.5)

    //action coordination
    if (fabs(OAleftright)<=0.01 && fabs(OAforward) <= 0.01){
      std::cout<<"I hate to do nothing, choosing Random behavior"<<std::endl;
      cmd.twist.linear.x = RNDforward;
      cmd.twist.linear.z = RNDupdown;
      cmd.twist.angular.z = RNDleftright;
    }
    else {
      std::cout<<"Choosing Obstacle Avoidance behavior"<<std::endl;
      cmd.twist.linear.x = OAforward;
      cmd.twist.linear.z = OAupdown;
      cmd.twist.angular.z = OAleftright;
    }
    //BEHAVIORS CODE (END)

    if (lastOdomReading->pose.pose.position.z >= -0.1){
      std::cout<<"Robot should stay in the water"<<std::endl;
      cmd.twist.linear.z = -0.1;
    }
    //publishing command to a robot
    cmd.header.stamp = ros::Time::now();
    pub.publish(cmd);

    //sleeping so, that the loop won't run faster than r's frequency
    r.sleep();
  }
}

//entry point of the executable
int
main(int argc, char** argv)
{
  ros::init(argc, argv, "swarmrobot");
  swarmrobot();
  return 0;
}
