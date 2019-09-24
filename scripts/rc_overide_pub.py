#!/usr/bin/env python2
'''
Perception and Robotics Group
University of Maryland

Author:
Ashwin Varghese Kuruttukulam(ashwinvk94@gmail.com)
'''

import sys
import rospy
import math
from std_msgs.msg import Int16
from mavros_msgs.msg import OverrideRCIn,RCIn
import tf.transformations

class rc_pub:
	def __init__(self):

		#Initialize flags
		self.obstacleAvoidFlag = False
		self.objectFlag = False
		self.rcFlag = False

		# Rate init
		self.rate = rospy.Rate(10.0)  # MUST be more then 20Hz
		self.moveTime = 2
		self.safeDist = 2
		self.rollThresh = 200
		self.pitchThresh = 200
		self.Kp = 200
		self.distThresh = 2
		
		self.rollSig = 0
		self.pitchSig = 0

		#Initializing publishers
		rospy.Subscriber("/mavros/rc/in", RCIn, self.RCInCb)
		rospy.Subscriber("/cat_dog/lidar_distances", LidarDistances, self.lidarDistanceCb)
		self.rcPub = rospy.Publisher("/mavros/rc/override",OverrideRCIn, queue_size = 10)

		while not rospy.is_shutdown():
			self.rcMsg = OverrideRCIn()
			#Check if lidar dist has been published
			if self.objectFlag and self.rcFlag:
				if self.rollSig<=self.rollThresh and self.rollSig>=-self.rollThresh:
					rollThreshSig = self.rollSig
				elif self.rollSig>self.rollThresh:
					rollThreshSig = self.rollThresh
					print 'thershold reached!'
				elif self.rollSig<-self.rollThresh:
					rollThreshSig = self.rollThresh
					print 'thershold reached!'
				if self.pitchSig<=self.pitchThresh and self.pitchSig>=-self.pitchThresh:
					pitchThreshSig = self.pitchSig
				elif self.pitchSig>self.pitchThresh:
					pitchThreshSig = self.pitchThresh
					print 'thershold reached!'
				elif self.pitchSig<-self.pitchThresh:
					pitchThreshSig = -self.pitchThresh
					print 'thershold reached!'

				#print 'rollThreshSig,pitchThreshSig'
				#print rollThreshSig,pitchThreshSig

				self.rcMsg.channels = [rollThreshSig,pitchThreshSig,0,0,0,0,0,0]

			else:
				self.rcMsg.channels = [0,0,0,0,0,0,0,0]
			# Safety to ensure that usual commands are taken in if the override is
			# switched off using aux rc switch
			if not self.obstacleAvoidFlag:
				self.rcMsg.channels = [0,0,0,0,0,0,0,0]
			# print 'OBSTACLE AVOIDANCE DISABLED!'
			self.rcPub.publish(self.rcMsg)

			self.rate.sleep()
	'''
	RC In callback
	'''
	def RCInCb(self,data):
		channelsData = data.channels
		print channelsData
		obstacleChannelData = channelsData[5]

		# print obstacleChannelData
		if obstacleChannelData>1470:
			self.obstacleAvoidFlag = True
		else:
			self.obstacleAvoidFlag = False
		self.rcFlag = True
	'''
	Lidar Distance callback
	'''
	def lidarDistanceCb(self,state):
		distances =  state.data
		# First value of array shows whether the obstacle has been detected and the
		# second value shows which direction to move in
		moveRightSig = distances[0]
		if moveRightSig:
			self.rollSig
			self.lidarDistFlag = True
		else:
			self.lidarDistFlag = False
		rightMoveSig = rightMoveSig/self.distThresh
		frontMoveSig = frontMoveSig/self.distThresh
		#print 'rightMoveSig,frontMoveSig'
		#print rightMoveSig,frontMoveSig


def main(args):
	rospy.init_node('rc_pub', anonymous=True)
	ic = rc_pub()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
