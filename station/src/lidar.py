#! /usr/bin/env python
import rospy
import ros_numpy
import math
import numpy as np 
import pandas as pd
import random as rd
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import KMeans
from std_msgs.msg import String
from darknet_ros_msgs.msg import ObjectCount



#rospy.init_node('station')
def lidarcam():

	case=0
	a=rospy.wait_for_message('/darknet_ros/countobj', String, timeout=None)
	kappa=int(a.data)
	#print(kappa)
	if(kappa!=0):
		b=rospy.wait_for_message('/darknet_ros/boxes', String, timeout=None)
		boxes=b.data
		#print(boxes)
		x=boxes.split("/")
		x.pop()
		box=[]
		for i in x:
			i=i.split()
			box.append([i[0],int(i[1]), int(i[3])])
		sorted_box=sorted(box, key=lambda x: x[1])

		
		x=rospy.wait_for_message('/cloud_in', PointCloud2, timeout=None)

		#tupl=[]
		tapl=[]

		for p in pc2.read_points(x, field_names = ("x", "y", "z"), skip_nans=True):
			if (abs(p[0])<0.8 and abs(p[1])<0.5):
				continue
			if (p[0]<0):
				continue
			#print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
			#tupl.append((p[0],p[1],p[2]))
			tapl.append([p[0],-p[1]])
			#plt.scatter(-p[1],p[0])

		#plt.show()

		kmeans = KMeans(n_clusters=kappa).fit(tapl)

		p=kmeans.cluster_centers_
		#print(p)
		final=[]
		i=0
		while i<kappa:
			klish=-p[i][1]/p[i][0]
			radian=math.atan(klish)
			degrees=math.degrees(radian)
			
			"""
			if(degrees>40 or degrees<-40):
				i=i+1
				continue
			"""
			final.append([round(math.sqrt(p[i][0]**2+p[i][1]**2),2),round(degrees,2)])
			i=i+1
		#print(final)
		lidar_final=sorted(final, key=lambda x:x[1], reverse=True)

		final= [a+b for a,b in zip(sorted_box,lidar_final)]

		#print(final)
		a=rospy.wait_for_message('/darknet_ros/countobj', String, timeout=None)
		kappa=int(a.data)
		if(kappa!=0):
			rospy.sleep(1.)
			b=rospy.wait_for_message('/darknet_ros/boxes', String, timeout=None)
			boxes=b.data
			#print(boxes)
			x=boxes.split("/")
			x.pop()
			box=[]
			for i in x:
				i=i.split()
				box.append([i[0],int(i[1]),int(i[3])])
				
			sorted_box=sorted(box, key=lambda x: x[1])

			
			x=rospy.wait_for_message('/cloud_in', PointCloud2, timeout=None)

			#tupl=[]
			tapl=[]

			for p in pc2.read_points(x, field_names = ("x", "y", "z"), skip_nans=True):
				if (abs(p[0])<0.8 and abs(p[1])<0.5):
					continue
				if (p[0]<0):
					continue
				#print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
				#tupl.append((p[0],p[1],p[2]))
				tapl.append([p[0],-p[1]])
				#plt.scatter(-p[1],p[0])

			#plt.show()

			kmeans = KMeans(n_clusters=kappa).fit(tapl)

			p=kmeans.cluster_centers_
			#print(p)
			finale=[]
			i=0
			while i<kappa:
				klish=-p[i][1]/p[i][0]
				radian=math.atan(klish)
				degrees=math.degrees(radian)
				
				"""
				if(degrees>40 or degrees<-40):
					i=i+1
					continue
				"""
				finale.append([round(math.sqrt(p[i][0]**2+p[i][1]**2),2),round(degrees,2)])
				i=i+1
			#print(final)
			lidar_final=sorted(finale, key=lambda x:x[1], reverse=True)

			finale= [a+b for a,b in zip(sorted_box,lidar_final)]

			for i in finale:
				if i[0]=='boat':
					for j in final:
						print(j) 
						if j[0]=='boat':
							if (int(i[1])-int(j[1])>5 and i[4]<j[4]):
								print("boat moving right")
								if(int(i[2])<500):
									case=0 #synexizw
									#print("#einai aristera moy kai kineitai pros ta deksia") #einai aristera moy kai kineitai pros ta deksia
								else:
									case=1 #stamataw
									#print("einai mprosta moy kai paei deksia")

							elif (int(j[1])-int(i[1])>5 and j[4]<i[4]):
								print("boat moving left")
								if(int(i[2])<500):
									case=0
									#print("#einai aristera moy kai kineitai pros ta aristera") #einai aristera moy kai kineitai pros ta aristera
								else:
									case=2 #prepei na kanw deksia prosperasi
									#print("einai deksia mou kai paei aristera")
							else:
								print("boat not moving")
								if(int(i[2])<500):
									case=0 #synexizw
								else:
									case=2
							break
					break
	return case
#print(lidarcam())