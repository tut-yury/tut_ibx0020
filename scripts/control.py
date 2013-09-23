#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import tf   #for a function to transform quaternion into euler
import math #for sqrt and atan2 functions


def normalizeAngle(a){
    while a < -math.pi:
        a += 2.0*math.pi
    while a >  math.pi:
        a -= 2.0*math.pi	 
    return a

lastOdomReading = None

def odometryReceived(data):
    global lastOdomReading 
    lastOdomReading = data
    
def control():
    global lastOdomReading
    rospy.init_node('control')

    rospy.Subscriber("odom", Odometry, odometryReceived)
    pub = rospy.Publisher('cmd_vel', Twist)
    pub_p = rospy.Publisher('p', Float64)
    r = rospy.Rate(10) # 10hz
    
    #getting coefficients from Parameter Server
    Kp = rospy.get_param('Kp',0.3)
    Ka = rospy.get_param('Ka',0.8)
    Kb = rospy.get_param('Kb',-0.15)

    #goal pose: coordinates (x,y) in metres and orintation (th) in radians
    goal_x = rospy.get_param('~x',0)
    goal_y = rospy.get_param('~y',0)
    #goal_th = rospy.get_param('~th',0) #unused for now and equals to 0

    cmd = Twist() #command that will be sent to Stage (published)
    
    while not rospy.is_shutdown():
        if lastOdomReading == None:#we cannot issue any commands until we have our position
            print 'waiting for lastOdomReading to become available'
            r.sleep()
            continue 
        
        #current robot 2D coordinates and orientation
        pose = lastOdomReading.pose.pose #robots current pose (position and orientation)
        x = pose.position.x
        y = pose.position.y
        #euler_from_quaternion returns array of [pitch, roll, yaw], we need only yaw (rotation around Z axis)
        th = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]

        #deltas
        dx = goal_x - x
        dy = goal_y - y
        dth = goal_th - th

        print x, y, th
        if math.fabs(dx)<0.01 and math.fabs(dy)<0.01 and math.fabs(dth)<0.01:
            print 'Reached destination'
            break
        
        #control equations (slides 23 and 24)
        p = math.sqrt(dx*dx + dy*dy)
        a = normalizeAngle(-th + math.atan2(dy,dx)) 
        b = normalizeAngle(-th - a) 
        v = Kp*p #translational speed in m/s
        w = normalizeAngle(Ka*a + Kb*b) #rotational speed in rad/s
        #setting command fields
        cmd.linear.x = v 
        cmd.angular.z = w 
        #publishing command to a robot
        pub.publish(cmd);
    
        #publishing p for rqt_plot
        Float64 p_
        p_.data = p
        pub_p.publish(p_)
        
        r.sleep()
        

if __name__ == '__main__':
    control()
