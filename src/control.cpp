#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <cmath> //for sqrt and atan2 functions
#include <tf/tf.h> //to transform quaternion into euler

//utility function to normalyze angles (-pi <= a <= pi)
double normalizeAngle( double a ){
    while( a < -M_PI ) a += 2.0*M_PI;
    while( a >  M_PI ) a -= 2.0*M_PI;
    return a;
};

//global variable to store the last received odometry reading
nav_msgs::OdometryConstPtr lastOdomReading;

//callback function to process data from subscribed Odometry topic
void odometryReceived(nav_msgs::OdometryConstPtr msg){
  lastOdomReading = msg;
}

//main function of the node
void control(){
  //creating node handles for subscription/publishing and parameter server queries
  ros::NodeHandle node;
  ros::NodeHandle pnode("~"); //used to get private parameters ~x, ~y and ~th

  //subscribing to odometry and announcing published topics
  ros::Subscriber sub = node.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",10, odometryReceived);
  ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);
  ros::Publisher pub_p = node.advertise<std_msgs::Float64>("p", 1, false);

  ros::Rate r(10); //an object to maintain specific frequency of a control loop - 10hz

  //declaring coefficients variables with default values
  double Kp = 0.3;
  double Ka = 0.8;
  double Kb = -0.15;
  //getting coefficients from Parameter Server
  node.getParam("Kp",Kp);
  node.getParam("Ka",Ka);
  node.getParam("Kb",Kb);
  std::stringstream ss;
  ss << "Coefficients: Kp="<< Kp << ", Ka="<< Ka <<", Kb="<< Kb;
  ROS_INFO("%s", ss.str().c_str());

  //goal pose: coordinates (x,y) in metres and orintation (th) in radians
  double goal_x = 0;
  double goal_y = 0;
  double goal_th = 0;
  //getting parameters from Parameter Server
  pnode.getParam("x",goal_x);
  pnode.getParam("y",goal_y);
  //pnode.getParam("th",goal_th);//th is 0 for now, since control equation has hardcoded final orientation of 0
  ROS_INFO("Goal: x=%f, y=%f, th=%f", goal_x, goal_y, goal_th);

  geometry_msgs::Twist cmd; //command that will be sent to Stage (published)

  while(ros::ok())
  {
    //required in C++ clients to receive messages from subscribed topics
    //note: Python clients don't need this call
    ros::spinOnce();

    if (!lastOdomReading){//we cannot issue any commands until we have our position
      std::cout<<"waiting for lastOdomReading to become available"<<std::endl;
      r.sleep();
      continue;
    }

    //current robot 2D coordinates and orientation
    geometry_msgs::Pose pose = lastOdomReading->pose.pose;//robots current pose (position and orientation)
    double x = pose.position.x;
    double y = pose.position.y;
    double th = tf::getYaw(pose.orientation);

    //deltas
    double dx = goal_x - x;
    double dy = goal_y - y;
    double dth = normalizeAngle(goal_th - th);

    std::cout<<x<<" "<<y<<" "<<th<<std::endl; //outputing current x, y anf th to standard output
    if (fabs(dx)<0.02 && fabs(dy)<0.02 && fabs(dth)<0.02){
      //if we are acceptably close to the goal, we can exit
      std::cout<<"Reached destination"<<std::endl;
      break;
    }

    //control equations (slide 25)
    double p = sqrt(dx*dx + dy*dy);
    double a = normalizeAngle(-th + atan2(dy,dx));
    double b = normalizeAngle(-th - a);
    //control equations (slide 26)
    double v = Kp*p; //translational speed in m/s
    double w = normalizeAngle(Ka*a + Kb*b); //rotational speed in rad/s
    //setting command fields
    cmd.linear.x = v;
    cmd.angular.z = w;
    //publishing command to a robot
    pub.publish(cmd);

    //publishing p for rqt_plot
    std_msgs::Float64 p_;
    p_.data = p;
    pub_p.publish(p_);

    //sleeping so, that the loop won't run faster than r's frequency
    r.sleep();
  }
}

//entry point of the executable
int
main(int argc, char** argv)
{
  ros::init(argc, argv, "control");
  control();
  return 0;
}
