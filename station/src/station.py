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


rospy.init_node('station')
def startBoth(speed):
	rate = rospy.Rate(10)
	publisher1 = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=1)
	publisher2 = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=1)
	rate.sleep()
	publisher1.publish(speed)
	publisher2.publish(speed)
	rate.sleep()

def startRightEngine(speed):
	rate = rospy.Rate(10)
	publisher = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=1)
	rate.sleep()
	publisher.publish(speed)
	rate.sleep()

def startLeftEngine(speed):
	rate = rospy.Rate(10)
	publisher = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=1)
	rate.sleep()
	publisher.publish(speed)
	rate.sleep()

def startLateralEngine(speed):
	rate = rospy.Rate(10)
	publisher = rospy.Publisher('/wamv/thrusters/lateral_thrust_cmd', Float32, queue_size=1)
	rate.sleep()
	publisher.publish(speed)

def getGoal():
	rate = rospy.Rate(10)  
	def temp(data):
		global temp1,temp2,x1,y1,z1,w
		temp1=data.pose.position.latitude
		temp2=data.pose.position.longitude
		x1=data.pose.orientation.x
		y1=data.pose.orientation.y
		z1=data.pose.orientation.z
		w=data.pose.orientation.w
	rospy.Subscriber('/vrx/station_keeping/goal', GeoPoseStamped, temp)
	rate.sleep()
	r = R.from_quat([x1, y1, z1, w])
	(z,y,x)=r.as_euler('zyx', degrees=True)
	return (temp1,temp2,z)

def getorientation():
	rate = rospy.Rate(10)  
	def temp(data):
		global x1,y1,z1,w
		x1=data.orientation.x
		y1=data.orientation.y
		z1=data.orientation.z
		w=data.orientation.w
	rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, temp)
	rate.sleep()
	r = R.from_quat([x1, y1, z1, w])
	(z,y,x)=r.as_euler('zyx', degrees=True)
	return z

def getLatAndLong():
	rate = rospy.Rate(10)  
	def temp(data):
		global temp1,temp2
		temp1=data.latitude
		temp2=data.longitude
	rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, temp)
	rate.sleep()
	return (temp1,temp2)

def TurnToOrientation(x):
	orientation=getorientation()
	print "Turning to",x,"degrees"
	goal=x
	if (goal-orientation>180):
		s=-4
		k=2
	else:
		s=2
		k=-4 
	while(abs(abs(orientation)-goal)>80):
		startRightEngine(s)
		startLeftEngine(k)
		orientation=getorientation()
	while(abs(abs(orientation)-abs(goal))>10):
		startRightEngine(s/2)
		startLeftEngine(k/2)
		orientation=getorientation()
	time.sleep(5)
	while(abs(abs(orientation)-abs(goal))>3):
		startRightEngine(s/2)
		startLeftEngine(k/2)
		orientation=getorientation()
		time.sleep(3)
	print("Done")

(glat,glongi,gor)=getGoal()
if(gor>-45 and gor<=45):
	fixorientation=0
elif(gor<=-45 and gor>-135):
	fixorientation=-90
elif(gor>45 and gor<=135):
	fixorientation=90
else:
	fixorientation=180
(lat,longi)=getLatAndLong()
x=glat-lat
y=glongi-longi
if(y<=0):
	TurnToOrientation(180)
else:
	TurnToOrientation(0)
(lat,longi)=getLatAndLong()
x=glat-lat
y=glongi-longi
if(y>=0 and x>0):
	k=1
	l=1
elif(y>=0 and x<=0):
	k=1
	l=-2
elif(y<0 and x>0):
	k=1
	l=-2
elif(y<0 and x<=0):
	k=1
	l=1
flag=0
print "Moving to goal"
while(1):
	if(abs(abs(longi)-abs(glongi))>=0.00001 and abs(lat-glat)>=0.00001 and flag==0):
		startBoth(k)
		startLateralEngine(l)
		(lat,longi)=getLatAndLong()
		continue
	elif(abs(longi-glongi)>=0.00001):
		flag=1
		startBoth(k)
		(lat,longi)=getLatAndLong()
	elif(abs(lat-glat)>=0.00001):
		flag=1
		startLateralEngine(l)
		(lat,longi)=getLatAndLong()
	else:
		break
print "Reached goal"
print "Getting Orientation"
TurnToOrientation(fixorientation)
print "Fixing position again"
(lat,longi)=getLatAndLong()
x=glat-lat
y=glongi-longi
if(fixorientation==0):
	while(lat-glat>0.00001):
		startLateralEngine(-2)
		(lat,longi)=getLatAndLong()
	while(lat-glat<-0.00001):
		startLateralEngine(1)
		(lat,longi)=getLatAndLong()
	while(longi-glongi>0.00001):
		startBoth(-2)
		(lat,longi)=getLatAndLong()
	while(longi-glongi<-0.00001):
		startBoth(1)
		(lat,longi)=getLatAndLong()
if(fixorientation==180):
	while(lat-glat>0.00001):
		startLateralEngine(1)
		(lat,longi)=getLatAndLong()
	while(lat-glat<-0.00001):
		startLateralEngine(-2)
		(lat,longi)=getLatAndLong()
	while(longi-glongi>0.00001):
		startBoth(1)
		(lat,longi)=getLatAndLong()
	while(longi-glongi<-0.00001):
		startBoth(-2)
		(lat,longi)=getLatAndLong()
if(fixorientation==90):
	while(lat-glat>0.00001):
		startBoth(-2)
		(lat,longi)=getLatAndLong()
	while(lat-glat<-0.00001):
		startBoth(1)
		(lat,longi)=getLatAndLong()
	while(longi-glongi>0.00001):
		startLateralEngine(1)
		(lat,longi)=getLatAndLong()
	while(longi-glongi<-0.00001):
		startLateralEngine(-2)
		(lat,longi)=getLatAndLong()
if(fixorientation==-90):
	while(lat-glat>0.00001):
		startBoth(1)
		(lat,longi)=getLatAndLong()
	while(lat-glat<-0.00001):
		startBoth(-2)
		(lat,longi)=getLatAndLong()
	while(longi-glongi>0.00001):
		startLateralEngine(-2)
		(lat,longi)=getLatAndLong()
	while(longi-glongi<-0.00001):
		startLateralEngine(1)
		(lat,longi)=getLatAndLong()

print "Getting to final orientation"
TurnToOrientation(int(gor))
print "End of task"
