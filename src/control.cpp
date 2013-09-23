#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <tf/tf.h> //to transform quaternion into euler


double normalizeAngle( double a ){
    while( a < -M_PI ) a += 2.0*M_PI;
    while( a >  M_PI ) a -= 2.0*M_PI;	 
    return a;
};

nav_msgs::OdometryConstPtr lastOdomReading;

void odometryReceived(nav_msgs::OdometryConstPtr msg){
  lastOdomReading = msg;
}

int 
main(int argc, char** argv)
{ 
  
  ros::init(argc, argv, "control");
  ros::NodeHandle node;
  ros::NodeHandle pnode("~");
  ros::Subscriber sub = node.subscribe<nav_msgs::Odometry>("odom",10, odometryReceived);  
  ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);
  ros::Publisher pub_p = node.advertise<std_msgs::Float64>("p", 1, false);
  ros::Rate r(10);
  
  geometry_msgs::Twist cmd; //command that will be sent to Stage (published)
  
  //getting coefficients from Parameter Server
  double Kp = 0.3;
  double Ka = 0.8;
  double Kb = -0.15;
  node.getParam("Kp",Kp);
  node.getParam("Ka",Ka);
  node.getParam("Kb",Kb);
  
  //goal pose: coordinates (x,y) in metres and orintation (th) in radians
  double goal_x = 0;
  double goal_y = 0;
  double goal_th = 0;
  pnode.getParam("x",goal_x);
  pnode.getParam("y",goal_y);
  //pnode.getParam("th",goal_th);//th is 0 for now, since control equation has hardcoded final orientation
  
  while(ros::ok())
  {
    //required in C++ clients to receive messages from subscribed topics
    //note: Python clients don't need this call
    ros::spinOnce();
    
    if (!lastOdomReading){
      //waiting for an Odometry reading
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
    
    std::cout<<x<<" "<<y<<" "<<th<<std::endl;
    if (fabs(dx)<0.01 && fabs(dy)<0.01 && fabs(dth)<0.01){
      std::cout<<"Reached destination"<<std::endl;
      break;
    }

    //control equations (slides 23? and 24?)
    double p = sqrt(dx*dx + dy*dy);
    double a = normalizeAngle(-th + atan2(dy,dx)); 
    double b = normalizeAngle(-th - a); 
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
    
    r.sleep();
  }
  return 0;
}
