# Parameterized tasks taught via kinesthetic teaching using vision

The aim of this project is to control the robot FrankaEmika by giving it an object as a goal. It consists of two different working spaces, one for the GUI created using QT and the other one for the ros catkin workspace for all the backend calculations. 

# Requirements
## Hardware
1. FrankaEmika robot
2. Realsense Depth Camera
## Software
1. Ubuntu 18 
2. Ros Melodic
3. libfranka installed from source. Check: [FrankaemikaFCI](https://frankaemika.github.io/docs/installation_linux.html#building-from-source) for how to to install it. 
3. Python3.7 
4. Requirements for YOLOv5. The requirement file can be found under catkin_ws/src/yolo5_ros. It that can be installed using: 
``` 
python3.7 -m pip install -r requirements.txt
```
5. Pyrealsense package. It can be installed using: 
``` 
python3.7 -m pip  install pyrealsense2
```
# catkin_ws
## Building catkin_ws
1. Clone the catkin_ws in your home directory.
2. Inside the catkin_ws run: 
```
catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build  --continue-on-failure -s
```
Make sure to change the ```PATH``` value to the path of the build folder inside libfranka on your computer. 
3. Source the setup.bash: 
```
source devel/setup.bash
```


#### Note: 
The workspace has a lot of packages and it can happen that it won't build completely by the first try. If this happens, navigate to the package that failed to build and run: 
```
catkin build --this
```
If there are some dependencies missing, please install them and try to build the package again.

## Packages of the catkin_ws
### yolo5_ros
This ros-package allows using YOLOv5 with a realsense for detecting and classifiying objects in realtime. To use it independently from the Qt_ws run:
``` 
roslaunch yolo5_ros yolo5_publisher_listener.launch
```
For this, the Realsense camera must be connected to the computer. When you run this launch file, yolov5 is run to identify real-time image data. Based on the prediction results, the following Ropstopic is published: 
``` 
yolo5_ros/class_com
``` 
It publishes a message with type class_com, which is composed of a list of arrays containing the labels and an array of float arrays containing the x,y, and z coordinates of each detected object in the base frame. 

### DMP
 This package is used for generating a new trajctory given a demo and an updated goal . It was cloned from [dmp](https://github.com/sniekum/dmp) and extented to be able to accept variable arguments for both the trajectory and the goal. To use it independently from the Qt_ws run:
```
roslaunch dmp dmp_generate_trajectoy.launch demo_trajectory_file:=demo.txt goal_object:=Object.yaml
```
#### Note
Here we assume that the dmp source file has a folder called tasks, where the trajectories are stored and a folder under yolo5_ros where the objects to be found. 

# Qt_ws
