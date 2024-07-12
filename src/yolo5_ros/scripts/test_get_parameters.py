#!/usr/bin/env python
import rospy
import os
import yaml



def get_object_goal():
	object_file = rospy.get_param("goal_object")
	object_path = os.path.join(os.path.expanduser("~"),"frankaemikapanda", "src",
    "yolo5_ros","objects", object_file )
	with open(object_path, 'r') as f:
		content = yaml.load(f)
		COM = content["data"]
		print(COM)

def get_demo_trajectory_path():
	demo_trajectory_file = rospy.get_param("demo_trajectory_file")
	demo_trajectory_path = os.path.join(os.path.expanduser("~"),"frankaemikapanda", "src",
    "dmp","tasks", demo_trajectory_file )
	return demo_trajectory_path

if __name__ == '__main__':
	rospy.init_node('test_parameter_node')
	print(get_object_goal())
	#get_object_goal()