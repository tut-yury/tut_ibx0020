#!/usr/bin/env python
import string
import time
import serial
import threading

import rospy
import tf

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg._Odometry import Odometry
from sensor_msgs.msg import Range


class intellibrainbot_serial (threading.Thread):
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200,
                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                 bytesize=serial.EIGHTBITS):
        threading.Thread.__init__(self)
        self.ser = serial.Serial(port=port, baudrate=baudrate, parity=parity,
                                 stopbits=stopbits, bytesize=bytesize)
        self.ser.open()
        self.ser.isOpen()
        self.threadLock_in = threading.Lock()
        self.threadLock_out = threading.Lock()
        self.serial_in = list()
        self.serial_out = list()
        self.continueRunning = True
        self.setDaemon(True)

    def addCommand(self, cmd):
        self.threadLock_in.acquire()
        inserted = False
        if cmd.startswith('D') or cmd.startswith('T'):  # looking for D or T command
            for i, val in enumerate(self.serial_in):
                if val.startswith('D') or val.startswith('T'):
                    self.serial_in.remove(val)
                    self.serial_in.insert(i, cmd)
                    inserted = True
                    break

        if not inserted:
            self.serial_in.append(cmd)

        self.threadLock_in.release()

    def getCommand(self):
        cmd = None
        self.threadLock_in.acquire()
        if len(self.serial_in) > 0:
            cmd = self.serial_in.pop(0)
        self.threadLock_in.release()
        return cmd

    def addData(self, data):
        self.threadLock_out.acquire()
        self.serial_out.append(data)
        self.threadLock_out.release()

    def getData(self):
        data = None
        self.threadLock_out.acquire()
        if len(self.serial_out) > 0:
            data = self.serial_out.pop(0)
        self.threadLock_out.release()
        return data

    def run(self):
        lastCommandSent = None
        while self.checkRunning():
            cmd = None
            if lastCommandSent is None or rospy.get_rostime().to_sec()-lastCommandSent > 0.2:
                cmd = self.getCommand()

            if cmd is not None:
                self.ser.write(cmd)
                self.ser.flush()
                lastCommandSent = rospy.get_rostime().to_sec()

            data = ''
            t = ''
            while t != '\n' and self.checkRunning():
                if self.ser.inWaiting() > 0:
                    t = self.ser.read(1)
                    if (t != '\r' and t != '\n'):
                        data += t
                elif len(data) == 0:
                    break
                else:
                    time.sleep(0.01)

            if len(data) > 0 and t == '\n' and self.checkRunning():
                self.addData(data)

            time.sleep(0.001)

    def checkRunning(self):
        return self.continueRunning and not rospy.is_shutdown()

    def destruct(self):
        print('Closing serial')
        try:
            self.ser.close()
        except:
            pass
        self.continueRunning = False
        try:
            self.join(2.0)
        except:
            pass


class intellibrainbot:
    def __init__(self, serial, rate=10,):
        self.rate = rate
        self.serial = serial

        rospy.init_node('intellibrainbot')

        # commands
        rospy.Subscriber("serial_in", String, self.callback_serial_in)
        rospy.Subscriber("twist", Twist, self.callback_twist)

        # sensors
        self.pub_serial_out = rospy.Publisher('serial_out', String)
        self.pub_L = rospy.Publisher('rangeL', Range)
        self.pub_R = rospy.Publisher('rangeR', Range)
        self.pub_C = rospy.Publisher('rangeC', Range)
        self.pub_O = rospy.Publisher('odom', Odometry)
        self.pub_tf = tf.TransformBroadcaster()

    def destruct(self):
        self.serial.destruct()

    def callback_serial_in(self, data):
        self.serial.addCommand(data.data)

    def callback_twist(self, data):
        self.serial.addCommand('T ' + str(int(round(data.linear.x*1000))) + ' ' + str(int(round(data.angular.z*1000))) + '\n')

    def run(self):

        L = Range()
        R = Range()
        L.min_range = 0.1
        L.max_range = 1.0
        L.radiation_type = Range.INFRARED
        L.field_of_view = 0.4  # ~22degrees
        L.header.frame_id = 'L'
        R.min_range = L.min_range
        R.max_range = L.max_range
        R.radiation_type = L.radiation_type
        R.field_of_view = L.field_of_view
        R.header.frame_id = 'R'

        C = Range()
        C.min_range = 0.1
        C.max_range = 3.0
        C.radiation_type = Range.ULTRASOUND
        C.field_of_view = 0.4  # ~22degrees
        C.header.frame_id = 'C'

        O = Odometry()
        O.header.frame_id = 'odom'
        O.child_frame_id = 'base_link'

        NaN = float('nan')

        r = rospy.Rate(self.rate)

        lastSensorCommandSent = None
        lastTfCommandSent = None

        while not rospy.is_shutdown():
            if lastSensorCommandSent is None or rospy.get_rostime().to_sec()-lastSensorCommandSent > 5:
                self.serial.addCommand('R L R C O\n')
                lastSensorCommandSent = rospy.get_rostime().to_sec()

            if lastTfCommandSent is None or rospy.get_rostime().to_sec()-lastTfCommandSent > 1:
                self.pub_tf.sendTransform((0.09, 0.00, 0.10), tf.transformations.quaternion_about_axis(0.0, (0, 0, 1)), rospy.get_rostime(), 'C', 'base_link')
                self.pub_tf.sendTransform((0.07, 0.05, 0.05), tf.transformations.quaternion_about_axis(0.6, (0, 0, 1)), rospy.get_rostime(), 'L', 'base_link')
                self.pub_tf.sendTransform((0.07, -0.05, 0.05), tf.transformations.quaternion_about_axis(-0.6, (0, 0, 1)), rospy.get_rostime(), 'R', 'base_link')
                lastTfCommandSent = rospy.get_rostime().to_sec()

            '''
            #code to dynamically enable/disable sensors depending on number of subscribers, disabled for now

            if lastSensorCommandSent==None or rospy.get_rostime().to_sec()-lastSensorCommandSent>5:
                cmdOn = 'R '
                cmdOff = 'r '
                if self.pub_C.get_num_connections()>0:
                    cmdOn+=' C'
                else:
                    cmdOff+=' C'
                if self.pub_L.get_num_connections()>0:
                    cmdOn+=' L'
                else:
                    cmdOff+=' L'
                if self.pub_R.get_num_connections()>0:
                    cmdOn+=' R'
                else:
                    cmdOff+=' R'
                if self.pub_O.get_num_connections()>0:
                    cmdOn+=' O'
                else:
                    cmdOff+=' O'


                if len(cmdOff)>2:
                    self.serial.addCommand(cmdOff)
                if len(cmdOn)>2:
                    self.serial.addCommand(cmdOn)
                lastSensorCommandSent = rospy.get_rostime().to_sec()
            '''

            data = self.serial.getData()

            if data is not None:
                self.pub_serial_out.publish(String(data))
                if data.startswith('R '):
                    datas = string.split(data, ' ')
                    for i, reading in enumerate(datas):
                        reading_arr = string.split(reading, ':')
                        if i > 0 and len(reading_arr) == 2:
                            rType = reading_arr[0]
                            rVal = reading_arr[1]
                            if rType == 'L':
                                L.header.stamp = rospy.get_rostime()
                                L.header.seq += 1
                                L.range = int(rVal)/1000.0 if int(rVal) > 0 else NaN
                                self.pub_L.publish(L)
                            elif rType == 'R':
                                R.header.stamp = rospy.get_rostime()
                                R.header.seq += 1
                                R.range = int(rVal)/1000.0 if int(rVal) > 0 else NaN
                                self.pub_R.publish(R)
                                pass
                            elif rType == 'C':
                                C.header.stamp = rospy.get_rostime()
                                C.header.seq += 1
                                C.range = int(rVal)/1000.0 if int(rVal) > 0 else NaN
                                self.pub_C.publish(C)
                            elif rType == 'O':
                                rVal_arr = string.split(rVal, ',')
                                if len(rVal_arr) == 5:
                                    O.header.stamp = rospy.get_rostime()
                                    O.pose.pose.position.x = int(rVal_arr[0])/1000.0
                                    O.pose.pose.position.y = int(rVal_arr[1])/1000.0
                                    q = tf.transformations.quaternion_about_axis(int(rVal_arr[2])/1000.0, (0, 0, 1))
                                    O.pose.pose.orientation.z = q[2]
                                    O.pose.pose.orientation.w = q[3]
                                    O.twist.twist.linear.x = int(rVal_arr[3])/1000.0
                                    O.twist.twist.angular.z = int(rVal_arr[4])/1000.0
                                    self.pub_O.publish(O)
                                    self.pub_tf.sendTransform(
                                                              (O.pose.pose.position.x, O.pose.pose.position.y, O.pose.pose.position.z),
                                                              (O.pose.pose.orientation.x, O.pose.pose.orientation.y, O.pose.pose.orientation.z, O.pose.pose.orientation.w),
                                                              rospy.get_rostime(), 'base_link', 'odom')

            r.sleep()

if __name__ == '__main__':
    try:
        serial = intellibrainbot_serial()
        bot = intellibrainbot(serial)
        serial.start()
        bot.run()
        bot.destruct()

    except rospy.ROSInterruptException:
        pass
