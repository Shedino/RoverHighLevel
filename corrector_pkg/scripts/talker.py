#!/usr/bin/env python
import rospy
import csv
import sys
import math
#from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from donkey_rover.msg import Scanner_Command

def talker():

    pub = rospy.Publisher('scanner_commands', Scanner_Command, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	#print ('Waiting for the scanner command (gohome, start (p), stop (s), insert (i): ');	
	#name = raw_input()
	msg = Scanner_Command()
	msg.Scanner_Command = 'GoHome' #'Horizontal' #
	msg.Scanner_Ajustment_Angle = -100  #rad -100 for no change
 	msg.Scanner_Roll_Angle = 1.57       #rad -100 for no change
	msg.Scanner_Home_Angle = -100      #rad -100 for no change
	msg.Scanner_Period = 2          #sec -100 for no 
	
	#corrector2 = 1.2        
	#hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(msg)
        #pub2.publish(corrector2)	
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
        
 
 

