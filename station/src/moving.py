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
from lidar import *

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
	rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, temp)
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
	rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, temp)
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

def movingto_old(a,b):
	maxdistance=math.sqrt( ((0.00002)**2)+((0.00002)**2) )
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
			startBoth(1)
			(lat,lon)=getLatAndLong()
			p1=[lon,lat]
			distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
		print("Moved to "+str(a)+" and "+str(b))

def avoidcol():
	if(getorientation()<-135):
		TurnToOrientation(180-45-(-180-getorientation()))
	else: 
		TurnToOrientation(getorientation()-45)
	t_end = time.time() + 20
	while (time.time() < t_end):
		startBoth(2)
	print('nd')

#(glat,glong,gori)=getGoal()
#movingto_old(glat,glong)
#TurnToOrientation(gori)

#case=lidarcam()
def movingto(a,b):
	maxdistance=math.sqrt( ((0.00002)**2)+((0.00002)**2) )
	p=computewheretoturn(a,b)
	if p==-999:
		print("Already in position")
	else:	
		TurnToOrientation(p)
		(lat,lon)=getLatAndLong()
		p1=[lon,lat]
		p2=[b,a]
		distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
		tend=time.time() + 100
		while(distance>maxdistance):
			if(time.time()>=tend):
				return 0
			case=lidarcam()
			if case==0:
				startBoth(2)
				(lat,lon)=getLatAndLong()
				p1=[lon,lat]
				distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
			elif case==1:
				t_endo = time.time() + 20
				while(time.time()<t_endo):
					pass
			elif case==2:
				avoidcol()
				print("Fixing route again")
				return 0
		print("Moved to "+str(a)+" and "+str(b))
		return 1


glat=37.94014
glong=23.63955
while movingto(glat,glong)!=1:
	pass