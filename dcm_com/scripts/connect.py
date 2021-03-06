#!/usr/bin/env python
import sys
sys.path.append('/home/sherpa/pySWM')
import math
import rospy
import tf
import swm

from custom_msgs.msg import positionEstimate
from custom_msgs.msg import orientationEstimate

agentName = "donkey"
currentRoll = 0
currentPitch = 0
currentYaw = 0
currentLat = 0
currentLon = 0
currentAlt = 0

def updateAttitude(data):
    global currentRoll, currentPitch, currentYaw
    currentRoll = data.roll * math.pi/180.0
    currentPitch = data.pitch * math.pi/180.0
    currentYaw = data.yaw * math.pi/180.0

def updatePosition(data):
    global currentLat, currentLon, currentAlt
    currentLat = data.latitude
    currentLon = data.longitude
    currentAlt = data.altitude

if __name__ == '__main__':
    try:
        rospy.init_node('dcm_com')
        print "[dcm_com:] initialising %s data on DCM database ..." % (agentName)
        swm.run('set donkey geopose 0 0 0 0 0 0 1')
        rospy.Subscriber("mti/filter/orientation",orientationEstimate,updateAttitude)
        rospy.Subscriber("mti/filter/position",positionEstimate,updatePosition)
        rate = rospy.Rate(0.2)
        while not rospy.is_shutdown():
            currentQuaternion = tf.transformations.quaternion_from_euler(currentRoll, currentPitch, currentYaw, "sxyz")
            swm.run('set donkey geopose %.6f %.6f %.2f %.2f %.2f %.2f %.2f' % (currentLat, currentLon, currentAlt, currentQuaternion[0], currentQuaternion[1], currentQuaternion[2], currentQuaternion[3]))
            print "[dcm_com:] the current geopose of the donkey in the DCM is %.6f,%.6f %.2f %.2f %.2f %.2f %.2f" % (currentLat, currentLon, currentAlt, currentQuaternion[0], currentQuaternion[1], currentQuaternion[2], currentQuaternion[3])
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
