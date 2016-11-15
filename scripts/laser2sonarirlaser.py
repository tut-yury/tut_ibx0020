#!/usr/bin/env python
import random
import math

import rospy
import tf
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range


class laser2sonarirlaser:
    def __init__(self, sensor_num=4, radius=0.1, rate=10):
        self.rate = rate
        self.sensor_num = sensor_num
        self.radius = radius

        rospy.init_node('laser2sonar')
        self.rng = Range()
        self.laserData = None
        self.pub_sonar = []
        self.pub_ir = []
        for idx in range(0, self.sensor_num):
            self.pub_sonar.append(rospy.Publisher('sonar_'+str(idx), Range, queue_size=self.sensor_num))
            self.pub_ir.append(rospy.Publisher('ir_'+str(idx), Range, queue_size=self.sensor_num))
        self.pub_laser = rospy.Publisher('laser', LaserScan, queue_size=self.sensor_num)
        self.pub_tf = tf.TransformBroadcaster()
        rospy.Subscriber("base_scan", LaserScan, self.callback_LaserScan)

    def callback_LaserScan(self, data):
        self.laserData = data
        total_angle = self.laserData.angle_max-self.laserData.angle_min
        ln = len(self.laserData.ranges)
        step = ln/self.sensor_num
        rng = self.rng
        rng.min_range = max(0, data.range_min-self.radius)
        rng.max_range = max(0, data.range_max-self.radius)
        rng.header.stamp = rospy.get_rostime()
        rng.header.seq += 1
        for idx in range(self.sensor_num):
            rng.header.frame_id = 'range_'+str(idx)
            rng.range = data.range_max
            for i in range(idx*step, (idx+1)*step):
                rng.range = min(rng.range, data.ranges[i])
            rng.range *= random.uniform(0.7, 1.3)
            rng.range = max(0, rng.range-self.radius)
            rng.radiation_type = Range.ULTRASOUND
            rng.field_of_view = total_angle/self.sensor_num
            self.pub_sonar[idx].publish(rng)

            rng.radiation_type = Range.INFRARED
            rng.field_of_view = data.angle_increment
            rng.range = data.ranges[idx*step + step/2]
            rng.range *= random.uniform(0.9, 1.1)
            rng.range = max(0, rng.range-self.radius)
            self.pub_ir[idx].publish(rng)

        ranges = data.ranges
        self.laserData.ranges = []
        for i in range(ln):
            self.laserData.ranges.append(ranges[i] * random.uniform(0.99, 1.01))

        self.pub_laser.publish(self.laserData)

        total_angle = self.laserData.angle_max-self.laserData.angle_min
        stepRad = total_angle/self.sensor_num
        idx = rng.header.seq % self.sensor_num
        frame_id = 'range_'+str(idx)
        print frame_id
        angle = self.laserData.angle_min+stepRad*(idx+0.5)
        self.pub_tf.sendTransform((math.cos(angle)*self.radius, math.sin(angle)*self.radius, 0.00),
                                  tf.transformations.quaternion_about_axis(angle, (0, 0, 1)),
                                  rospy.get_rostime(), frame_id, 'base_laser_link')

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        l2s = laser2sonarirlaser()
        l2s.run()
    except rospy.ROSInterruptException:
        pass
