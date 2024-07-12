#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
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

def readData(file_name):
    trajectory = []
    # Lecture des en-têtes des données avec comme délimiteur le point-virgule
    head = np.loadtxt(file_name, delimiter=',', max_rows=1, dtype=np.str)
    # Lecture des données au format str
    data = np.loadtxt(file_name, delimiter=',', skiprows=1, dtype=np.str)
    # data = np.char.replace(data, ',', '.')

    # Affichage des en-têtes
    print(head)

    # Sélections des données en fonction de l'en-tête et conversion en flottant
    X = np.asarray(data[:, np.where(head == 'X_value')],
                   dtype=np.float, order='C').flatten()
    Y = np.asarray(
        data[:, np.where(head == 'Y_value')], dtype=np.float, order='C').flatten()
    Z = np.asarray(
        data[:, np.where(head == 'Z_value')], dtype=np.float, order='C').flatten()
    for i in range (len(X)):
        trajectory.append([X[i],Y[i],Z[i]])
    return trajectory


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

def printTraj(file_name):
    f=open("plan.txt","r")


if __name__ == '__main__':
    rospy.init_node('dmp_tutorial_node')

    #Create a DMP from a 2-D trajectory
    dims = 3
    dt = 1.0                
    K = 100                 
    D = 2.0 * np.sqrt(K)      
    num_bases = 4
    traj = readData("/home/dhrikarl/catkin_ws/data2.txt")
    print(traj)
    #traj = [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = [0.3,0.0,0.5]          #Plan starting at a different point than demo
    x_dot_0 = [0.0,0.0,0.0]
    t_0 = 0                
    goal = [0.5,0.0,0.5]         #Plan to a different goal than demo
    goal_thresh = [0.2,0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * resp.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)
    writeData("plan2.txt", plan, dims)
    printTraj("plan2.txt")
    print (plan)
