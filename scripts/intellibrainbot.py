#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial
import threading



class intellibrainbot:
    def callback_serial_in(self, data):
        rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)

    def addCommand(self, cmd):
        self.threadLock.acquire()
        self.commands.append(cmd)
        self.threadLock.release()
        #print ('Added command "'+ cmd + '"')
   
    def __init__(self, rate = 10, port='/dev/ttyUSB0', baudrate=115200, 
        parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS):
        self.ser = serial.Serial(port=port,baudrate=baudrate,parity=parity,stopbits=stopbits,bytesize=bytesize)
        self.ser.open()
        self.ser.isOpen()
        self.rate = rate
        self.threadLock = threading.Lock()
        self.commands=list()
        self.addCommand('D 0 0\n')
        self.addCommand('R L R C O\n')
        rospy.init_node('intellibrainbot')
        rospy.Subscriber("serial_in", String, self.callback_serial_in)
        
        
    def run(self):
        self.publish()
        
    def destruct(self):
        print ('Closing serial')
        self.ser.close()
        
    def publish(self):
        self.pub_serial_out = rospy.Publisher('serial_out', String)    

        data=''
        r = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            cmd = None    
            if len(self.commands)>0:
                self.threadLock.acquire()
                cmd = self.commands.pop(0)
                self.threadLock.release()
            
            if cmd!=None:
                #print ('Sending command "'+ cmd + '"')
                self.ser.write(cmd)
            
            data=''
            t = ''        
            while t!='\n' and not rospy.is_shutdown():
                #print ('Waiting '+ str(self.ser.inWaiting()) +' "' + data + '"' )
                if self.ser.inWaiting() > 0:
                    t = self.ser.read(1)
                    if (t!='\r' and t!='\n'):
                        data+=t
                elif len(data)==0:
                    break
                
            if len(data)>0:
                str_ = data + ' ' + str(self.ser.inWaiting())
                rospy.loginfo(str_)
                self.pub_serial_out.publish(String(str_))
                
            r.sleep()

if __name__ == '__main__':
    try:
    
        bot = intellibrainbot()
        bot.run()
        bot.destruct()
        
    except rospy.ROSInterruptException:
        pass
