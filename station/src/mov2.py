#! /usr/bin/env python
import rospy
import numpy as np 
import time
import math
from haversine import haversine, Unit
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import Float32

rospy.init_node('station2')
def startBoth(speed):
	rate = rospy.Rate(10)
	publisher1 = rospy.Publisher('/boat2/thrusters/right_thrust_cmd', Float32, queue_size=1)
	publisher2 = rospy.Publisher('/boat2/thrusters/left_thrust_cmd', Float32, queue_size=1)
	rate.sleep()
	publisher1.publish(speed)
	publisher2.publish(speed)
	rate.sleep()
def startBothi(left,right):
	rate = rospy.Rate(10)
	publisher1 = rospy.Publisher('/boat2/thrusters/right_thrust_cmd', Float32, queue_size=1)
	publisher2 = rospy.Publisher('/boat2/thrusters/left_thrust_cmd', Float32, queue_size=1)
	rate.sleep()
	publisher1.publish(right)
	publisher2.publish(left)
	rate.sleep()

def startRightEngine(speed):
	rate = rospy.Rate(10)
	publisher = rospy.Publisher('/boat2/thrusters/right_thrust_cmd', Float32, queue_size=1)
	rate.sleep()
	publisher.publish(speed)
	rate.sleep()

def startLeftEngine(speed):
	rate = rospy.Rate(10)
	publisher = rospy.Publisher('/boat2/thrusters/left_thrust_cmd', Float32, queue_size=1)
	rate.sleep()
	publisher.publish(speed)
	rate.sleep()

def startLateralEngine(speed):
	rate = rospy.Rate(10)
	publisher = rospy.Publisher('/boat2/thrusters/lateral_thrust_cmd', Float32, queue_size=1)
	rate.sleep()
	publisher.publish(speed)

def getGoal():
	rate = rospy.Rate(10)  
	def temp(data):
		global temp3,temp4,x1,y1,z1,w
		temp3=data.pose.position.latitude
		temp4=data.pose.position.longitude
		x1=data.pose.orientation.x
		y1=data.pose.orientation.y
		z1=data.pose.orientation.z
		w=data.pose.orientation.w
	rospy.Subscriber('/vrx/station_keeping/goal', GeoPoseStamped, temp)
	rate.sleep()
	r = R.from_quat([x1, y1, z1, w])
	(z,y,x)=r.as_euler('zyx', degrees=True)
	return (temp3,temp4,z)

def getorientation():
	rate = rospy.Rate(10)  
	def temp(data):
		global x2,y2,z2,w2
		x2=data.orientation.x
		y2=data.orientation.y
		z2=data.orientation.z
		w2=data.orientation.w
	rospy.Subscriber('/boat2/sensors/imu/imu/data', Imu, temp)
	rate.sleep()
	r = R.from_quat([x2, y2, z2, w2])
	(z,y,x)=r.as_euler('zyx', degrees=True)
	return z

def getLatAndLong():
	rate = rospy.Rate(10)  
	def temp(data):
		global temp1,temp2
		temp1=data.latitude
		temp2=data.longitude
	rospy.Subscriber('/boat2/sensors/gps/gps/fix', NavSatFix, temp)
	rate.sleep()
	return (temp1,temp2)

def TurnToOrientation(x):
	orientation=getorientation()
	print "Turning to",x,"degrees"
	goal=x
	if (goal-orientation>180):
		r=-2.0
		l=1.0
	elif (goal-orientation<-180):
		r=1.0
		l=-2.0
	elif (goal-orientation<0):
		r=-2.0
		l=1.0
	else:
		r=1.0
		l=-2.0
	while(abs(orientation-goal)>10):
		startRightEngine(r)
		startLeftEngine(l)
		orientation=getorientation()
	time.sleep(5)
	while(abs(orientation-goal)>3):
		startRightEngine(r)
		startLeftEngine(l)
		orientation=getorientation()
		time.sleep(3)
	print("Done")

def computewheretoturn(a,b):
	a=int(a*100000)
	b=int(b*100000)
	(lat,lon)=getLatAndLong()
	lat=int(lat*100000)
	lon=int(lon*100000)
	if(lat==a and lon==b):
		return -999
	elif(lat==a):
		if(b>lon):
			return 0
		else:
			return 180
	elif(lon==b):
		if(a>lat):
			return 90
		else:
			return -90
	else:
		if(lat>a):
			kappa=180
		else:
			kappa=0
		klish=float(a-lat)/float(b-lon)
		radian=math.atan(klish)
		degrees=math.degrees(radian)
		return int(degrees-kappa)

def movingto(a,b):
	maxdistance=math.sqrt( ((0.0002)**2)+((0.0002)**2) )
	p=computewheretoturn(a,b)
	if p==-999:
		print("Already in position")
	else:	
		TurnToOrientation(p)
		(lat,lon)=getLatAndLong()
		p1=[lon,lat]
		p2=[b,a]
		distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
		while(distance>maxdistance):
			startBoth(5)
			(lat,lon)=getLatAndLong()
			p1=[lon,lat]
			distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
		print("Moved to "+str(a)+" and "+str(b))


#x=[(23.637133,37.944351),(23.636666,37.942716),(23.636351,37.941799),(23.636883,37.941816),(23.636966,37.941983),(23.63695,37.942116),(23.636766,37.942316),(23.636583,37.942416),(23.636283,37.942466),(23.635383,37.942133),(23.633783,37.941033)]
while(1):
	#startBothi(1.1,1)
	startBoth(2)
#movingto(glat,glong)

