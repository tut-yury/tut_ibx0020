#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry  # Odometry for depth and compass sensors
from geometry_msgs.msg import TwistStamped  # uwsim uses twist with a timestamp
from sensor_msgs.msg import Range  # range sensor data type
import tf    # for a function to transform quaternion into euler
import math  # for sqrt and atan2 functions
import random  # for random behavior

# global variables to store the last received range readings and odom
lastF = None
lastL = None
lastR = None
lastOdomReading = None


# callback functions to process data from subscribed range and topics
def rangeF_received(data):
    global lastF
    lastF = data


def rangeL_received(data):
    global lastL
    lastL = data


def rangeR_received(data):
    global lastR
    lastR = data


def odometryReceived(data):
    global lastOdomReading
    lastOdomReading = data


# main function of the node
def swarmrobot():
    global lastF
    global lastL
    global lastR
    global lastOdomReading

    # subscribing to odometry and ranges, announcing published topics
    rospy.Subscriber("odom", Odometry, odometryReceived)
    rospy.Subscriber("range_f", Range, rangeF_received)
    rospy.Subscriber("range_l", Range, rangeL_received)
    rospy.Subscriber("range_r", Range, rangeR_received)
    pub = rospy.Publisher('cmd_vel_stamped', TwistStamped)

    r = rospy.Rate(10)  # an object to maintain specific frequency of a control loop - 10hz

    cmd = TwistStamped()  # command that will be sent to Stage (published)

    while not rospy.is_shutdown():
        if lastF is None or lastL is None or lastR is None or lastOdomReading is None:
            print 'waiting for ranges and odom to become available'
            r.sleep()
            continue
        #  default speeds,
        cmd.twist.linear.x = 0  # forward
        cmd.twist.linear.z = 0  # up/down
        cmd.twist.angular.z = 0  # left/right

        # BEHAVIORS CODE (BEGIN)
        # INPUT:
        #   Only the following ranges: lastL, lastR, lastF, and Up/Down ranges (needs additional subscribers, etc).
        #   You are also allowed to use depth(lastOdomReading.pose.pose.position.z) and compass
        #   (lastOdomReading.pose.pose.orientation, see Closed-loop tutorial how to extract heading from it)
        #
        # OUTPUT:
        #   Only Thrust(cmd.twist.linear.x), Buoyance(cmd.twist.linear.z) and Turning(cmd.twist.angular.z)

        # Obstacle Avoidance behavior
        OAforward = max(0, lastF.range - 0.5)
        OAupdown = 0
        OAleftright = lastL.range-lastR.range
        # Random behavior
        RNDforward = 0
        RNDupdown = 0
        RNDleftright = random.random()*1.0 - 0.5  # random number in the range [-0.5, 0.5)

        # action coordination
        if abs(OAleftright) <= 0.01 and abs(OAforward) <= 0.01:
            print "I hate to do nothing, choosing Random behavior"
            cmd.twist.linear.x = RNDforward
            cmd.twist.linear.z = RNDupdown
            cmd.twist.angular.z = RNDleftright
        else:
            print "Choosing Obstacle Avoidance behavior"
            cmd.twist.linear.x = OAforward
            cmd.twist.linear.z = OAupdown
            cmd.twist.angular.z = OAleftright
        # BEHAVIORS CODE (END)

        if lastOdomReading.pose.pose.position.z >= -0.1:
            print "Robot should stay in the water"
            cmd.twist.linear.z = -0.1

        # publishing command to a robot
        cmd.header.stamp = rospy.Time.now()
        pub.publish(cmd)
        # sleeping so, that the loop won't run faster than r's frequency
        r.sleep()
        # end of loop
    # end of function

# entry point of the executable
# calling the main node function of the node only if this .py file is executed directly, not imported
if __name__ == '__main__':
    rospy.init_node('swarmrobot')
    swarmrobot()
