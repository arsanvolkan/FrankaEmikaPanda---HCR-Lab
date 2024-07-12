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

def orientationTrajectory(W,Wx,Wy,Wz):
    trajectory=[]
    for i in range (len(X)):
        trajectory.append([W[i],Wx[i],Wy[i],Wz[i]])
    return trajectory

def readFile1(file_name,X,Y,Z,T):

	with open(file_name, "r") as f:
		data = f.readlines()
	for line in data:
		x,y,z,t = line.split()
		X.append(float(x))
		Y.append(float(y))
		Z.append(float(z))
		T.append(float(t))
		
			
	return [X,Y,Z,T]
def readFile2(file_name,X,Y,Z,W,Wx,Wy,Wz,T):

	with open(file_name, "r") as f:
		data = f.readlines()
	
	
	for line in data:
		
		x,y,z,w,wx,wy,wz,t = line.split()
		X.append(float(x))
		Y.append(float(y))
		Z.append(float(z))
		W.append(float(w))
		Wx.append(float(wx))
		Wy.append(float(wy))
		Wz.append(float(wz))
		T.append(float(t))
			
			
			
	return [X,Y,Z,W,Wx,Wy,Wz,T]

def writeData(fileName, plan, dims):
	with open(fileName, 'w') as f:
		i = 0
		while i < len(plan.plan.points):
				j = 0
				while j < 3:
					f.write(str(plan.plan.points[i].positions[j]))
					f.write(" ")
					j+= 1
				f.write("\n")
				i +=1


def printPlan(file_name):
	X=[]
	Y=[]
	Z=[]
	with open(file_name, "r") as f:
		data = f.readlines()
	
	for line in data:
		x,y,z = line.split()
		X.append(float(x))
		Y.append(float(y))
		Z.append(float(z))
	trajectory=[X,Y,Z]
	x = trajectory[0]
	y = trajectory[1]
	z = trajectory[2]

	# Affichage des données
	print(x, '\n', y, '\n', z)

	fig = plt.figure()
	ax = fig.gca(projection='3d')  # Affichage en 3D
	ax.plot(x, y, z, label='Courbe')  # Tracé de la courbe 3D
	plt.title("Generated trajectory")
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	plt.tight_layout()
	plt.show()

def printTraj(trajectory):
	
	x = trajectory[0]
	y = trajectory[1]
	z = trajectory[2]

	# Affichage des données
	print(x, '\n', y, '\n', z)

	fig = plt.figure()
	ax = fig.gca(projection='3d')  # Affichage en 3D
	ax.plot(x, y, z)  # Tracé de la courbe 3D
	plt.title("Base trajectory")
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	plt.tight_layout()
	plt.show()

if __name__ == '__main__':
    rospy.init_node('dmp_tutorial_node')

    #Create a DMP from a 3-D trajectory
    dims = 7
    dt = 1.0                
    K = 100                 
    D = 2.0 * np.sqrt(K)      
    num_bases = 4
    X1=[]
    Y1=[]
    Z1=[]
    W1=[]
    Wx1=[]
    Wy1=[]
    Wz1=[]
    T1=[]

    [X1,Y1,Z1,W1,Wx1,Wy1,Wz1,T1]=readFile2("/home/dhrikarl/catkin_ws/joint_impedance_position_test_forDemo.txt",X1,Y1,Z1,W1,Wx1,Wy1,Wz1,T1)
    traj=trajectory(X1,Y1,Z1)
    orientation_traj=orientationTrajectory(W1,Wx1,Wy1,Wz1)
    #traj = [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
    printTraj([X1,Y1,Z1])
    print(traj)
    #traj = [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
    resp = makeLFDRequest(dims, traj, orientation_traj, dt, K, D, num_bases)

    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = [0.28,0.3,0.5]       #Plan starting at a different point than demo
    x_dot_0 = [0.0,0.0,0.0]
    t_0 = 0                
    goal = [0.28,0.3,0.4]         #Plan to a different goal than demo
    goal_thresh = [0.05,0.05,0.05]
    seg_length = -1          #Plan until convergence to goal
    tau = 1 * resp.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)
    writeData("planDMP_test_forDemo.txt", plan, dims)
    printPlan("planDMP_test_forDemo.txt")
    print (plan)
