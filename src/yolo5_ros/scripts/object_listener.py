#!/usr/bin/env python3.7
import rospy
import numpy as np
import os
from rospy.numpy_msg import numpy_msg
from yolo5_ros.msg import int_arrays, string_array, int_array, float_array, class_com


def classes_callback(data):
    # print log info 
    rospy.loginfo(data.labels)
    # Get path where objects text files will be stored
    objects_path = os.path.join(os.path.expanduser("~"),"frankaemikapanda", "src",
    "yolo5_ros","objects")
    # Delete all old objects
    for f in os.listdir(objects_path):
       os.remove(os.path.join(objects_path, f))
    #Create object.text files named after the labels
    # print("helloooo")
    for i in range(len(data.labels)):
        object_path = os.path.join(objects_path, data.labels[i]+'.txt')
        with open(object_path, 'w+') as f:
            f.write(str(data.coordinates.data[i]))

def coordinates_callback(data):
    pass


def classes_listener():
    rospy.Subscriber("/classes_coordinates", class_com, classes_callback)

if __name__ == '__main__':
    rospy.init_node('object_listener', anonymous=True)
    rate =  rospy.Rate(1/30)
    while not rospy.is_shutdown():
        rospy.loginfo("now listening")
        classes_listener()
        rate.sleep()