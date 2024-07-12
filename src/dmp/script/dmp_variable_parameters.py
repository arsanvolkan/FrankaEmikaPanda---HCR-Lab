#!/usr/bin/env python
#-*- coding:utf-8 -*-
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d  # Fonction pour la 3D
from dmp.srv import *
from dmp.msg import *
import os 
import yaml

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print ("Starting LfD...")
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)
    print ("LfD done")    
            
    return resp;


#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print ("Starting DMP planning...")
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)
    print ("DMP planning done")   
            
    return resp;


def trajectory(X,Y,Z):
    trajectory=[]
    for i in range (len(X)):
        trajectory.append([X[i],Y[i],Z[i]])
    return trajectory

def readFile2(file_name,X,Y,Z,T):

	with open(file_name, "r") as f:
		data = f.readlines()
	
	
	for line in data:
		
		x,y,z,w,wx,wy,wz,t = line.split()
		X.append(float(x))
		Y.append(float(y))
		Z.append(float(z))
		T.append(float(t))
				
			
	return [X,Y,Z,T]


def readVel(file_name,X,Y,Z,T):

	with open(file_name, "r") as f:
		data = f.readlines()
	
	
	for line in data:
		
		x,y,z,t = line.split()
		X.append(float(x))
		Y.append(float(y))
		Z.append(float(z))
		T.append(float(t))
				
			
	return [X,Y,Z,T]

def writeData(fileName, plan, dims, time,dt):
	file_path = os.path.join(os.path.expanduser("~"),"frankaemikapanda", "src",
    "dmp","tasks", fileName)
	# if file with the same name exists, delete it 
	if os.path.isfile(file_path):
		os.remove(file_path)
	with open(file_path, 'w+') as f:
		i = 0
		while i < len(plan.plan.points):
				j = 0
				while j < 3:
					f.write(str(plan.plan.points[i].positions[j]))
					f.write(" ")
					j+= 1
				if i<len(time):
					f.write(str(time[i]))
				else:
					f.write(str(time[-1]+i*dt))
				f.write("\n")
				i +=1

def get_object_goal():
	object_file = rospy.get_param("goal_object")
	object_path = os.path.join(os.path.expanduser("~"),"frankaemikapanda", "src",
    "yolo5_ros","objects", object_file )
	with open(object_path, 'r') as f:
		content = yaml.load(f)
		COM = content["data"]
		print(COM)
	return COM

def get_demo_trajectory_path():
	demo_trajectory_file = rospy.get_param("demo_trajectory_file")
	demo_trajectory_path = os.path.join(os.path.expanduser("~"),"frankaemikapanda", "src",
    "dmp","tasks", demo_trajectory_file )
	return demo_trajectory_path
	


if __name__ == '__main__':
	rospy.init_node('dmp_tutorial_node')

	#Create a DMP from a 3-D trajectory
	dims = 3
	dt = 1.0                
	K = 10000                 
	D = 2.0 * np.sqrt(K)      
	num_bases = 4
	X1=[]
	Y1=[]
	Z1=[]
	T1=[]
	VX1=[]
	VY1=[]
	VZ1=[]
	trajectory_path = get_demo_trajectory_path()
	[X1,Y1,Z1,T1]=readFile2(trajectory_path,X1,Y1,Z1,T1)
	#[VX1,VY1,VZ1,T1]=readVel("/home/dhrikarl/catkin_ws/joint_impedance_position_testdemoforPERSEO_touchFruit_velocity.txt",VX1,VY1,VZ1,T1)
	traj=trajectory(X1,Y1,Z1)
	resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

	#Set it as the active DMP
	makeSetActiveRequest(resp.dmp_list)

	#Now, generate a plan
	x_0 = [X1[0],Y1[0],Z1[0]]       #Plan starting at a different point than demo
	x_dot_0 = [0.0,0.0,0.0]
	t_0 = 0 
	# HERE CHANGE GOAL TO GET PARAMETERS               
	goal = get_object_goal()         #Plan to a different goal than demo
	goal_thresh = [0.05,0.05,0.05]
	seg_length = -1          #Plan until convergence to goal
	tau = 1 * resp.tau       #Desired plan should take twice as long as demo
	dt = 1.0
	integrate_iter = 5      #dt is rather large, so this is > 1  
	plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
							seg_length, tau, dt, integrate_iter)
	writeData("dmp_new_generated_trajectory.txt", plan, dims,T1,dt)