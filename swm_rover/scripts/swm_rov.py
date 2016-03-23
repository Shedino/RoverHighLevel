#!/usr/bin/env python

#****************************************************************************
#
# Sherpa World Model Interface
#
# This software rover developed at:
# Casy Lab- Bologna
#
# Description:
#  Interface node with SWM
#	 - Input: name of the rover
#
# Authors:
# Mohsen <seyedmohs.mirhassani@unibo.it>
#
# Created in 05/02/2016.
#
#
# Copyright (C) 2015 PRISMA Lab. All rights reserved.
#****************************************************************************/


#---Ros lib
import rospy
import roslib
import actionlib
#---

import os.path
import ConfigParser
import zmq
import random
import sys
import time
import json

import sys
import time
import re
import os
import thread
import multiprocessing


from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Joy

from pyutil.fipa import *
from pyutil.wdbutil import *
from pyutil.consutil import *

#-Receive the position of # rover form SWM 				-> publish on Ros Topic
class swm_interface_rover( object ):

	def __init__( self, name ):
		self.rover_name = name

	def getNodeId(self, queryMessage):
		context = zmq.Context()
		socket = context.socket(zmq.REQ)
		socket.connect("tcp://localhost:22422")		
		socket.send_string(json.dumps(queryMessage))
		queryResult = socket.recv_json()

		if queryResult['ids']:
			return queryResult['ids'][0]
		else:
			return ""

	def getgenpose(self, genius_query):
		context = zmq.Context()
		socket = context.socket(zmq.REQ)
		socket.connect("tcp://localhost:22422")		
		socket.send_string(json.dumps(genius_query))
		queryResult = socket.recv_json()
		return queryResult


#        genius_query = ({ "@worldmodeltype": "RSGQuery", "query": "GET_NODES","attributes": [{"key": "sherpa:agent_name","value": "genius"},],})


        #### RESULT STRUCTURE?????

	def get_geopose(self, node_id):
		context = zmq.Context()
		socket = context.socket(zmq.REQ)
		socket.connect("tcp://localhost:22422")		

		p = Pose()
		attributes_query = ({ "@worldmodeltype": "RSGQuery", "query": "GET_NODE_ATTRIBUTES", "id":  node_id})

		socket.send_string(json.dumps(attributes_query))
		result = socket.recv_json()

		#position		
		p.position.x = float( result['attributes'][3]['value'])
		p.position.y = float(result['attributes'][4]['value'])
		p.position.z = float(result['attributes'][5]['value'])
		#orientation
		p.orientation.w = float(result['attributes'][6]['value'])
		p.orientation.x = float(result['attributes'][7]['value'])
		p.orientation.y = float(result['attributes'][8]['value'])
		p.orientation.z = float(result['attributes'][9]['value'])

		return p
		

	
	def run(self):

		rate = rospy.Rate(10) # 10hz
		#---Rover 0
		rover_pub = rospy.Publisher('/CREATE/swm/' + self.rover_name, Pose, queue_size=1)		
		rover_ready_pub = rospy.Publisher('/CREATE/swm/' + self.rover_name + '_ready', Bool, queue_size=1)
		#rover_pose = Pose()		 
		rover_id = ""
		rover_ready = Bool()
		print self.rover_name
		genius_query = ({ "@worldmodeltype": "RSGQuery", "query": "GET_NODES","attributes": [{"key": "sherpa:agent_name","value": "genius"},],})
		print self.getgenpose(genius_query)
		while not rospy.is_shutdown():
			rover_ready.data = True
			print self.rover_name
			rover_pub.publish( rover_pose )
			rover_ready_pub.publish( rover_ready )
			rate.sleep()
				

if __name__ == '__main__':

	rover_name = "rover_" #+ sys.argv[1]
	rospy.init_node('CREATE_SWM_' + rover_name)
	interface = swm_interface_rover( rover_name )
	interface.run()


