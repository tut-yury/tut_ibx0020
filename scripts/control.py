#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf   #for a function to transform quaternion into euler
import math #for sqrt and atan2 functions

lastOdomReading = None

def callback(data):
    global lastOdomReading 
    lastOdomReading = data
    
def control():
    global lastOdomReading
    rospy.init_node('control')

    rospy.Subscriber("odom", Odometry, callback)
    pub = rospy.Publisher('cmd_vel', Twist)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	if lastOdomReading == None:#we cannot issue any commands until we have our position
            print 'waiting for lastOdomReading to become available'
            r.sleep()
            continue 

	#goal pose: coordinates (x,y) in metres and orintation (th) in radians
        goal_x = rospy.get_param('~x',0)
        goal_y = rospy.get_param('~y',0)
        goal_th = rospy.get_param('~th',0) #unused for now and equals to 0

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

        if math.fabs(dx)<0.01 and math.fabs(dy)<0.01 and math.fabs(dth)<0.01:
            print 'Reached destination'
            break
        else:
            print x, y, th
	
        #control equations (slides 23 and 24)
        p = math.sqrt(dx*dx + dy*dy)
        a = math.fmod(-th + math.atan2(dy,dx),2*math.pi) 
        if a>math.pi:
            a-=2*math.pi
        b = math.fmod(-th - a,2*math.pi) 
        if b>math.pi:
            b-=2*math.pi
        
        #getting coefficients from Parameter Server
        Kp = rospy.get_param('Kp',0.3)
        Ka = rospy.get_param('Ka',0.8)
        Kb = rospy.get_param('Kb',-0.15)

        #calculating speeds
        cmd = Twist() #command that will be sent to Stage (published)
        cmd.linear.x = Kp*p #v, translational speed in m/s
	cmd.angular.z = Ka*a + Kb*b #w - rotational speed in rad/s

        pub.publish(cmd)
        r.sleep()
        
if __name__ == '__main__':
    control()

