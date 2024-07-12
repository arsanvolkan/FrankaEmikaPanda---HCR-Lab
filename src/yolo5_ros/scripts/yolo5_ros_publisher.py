#!/usr/bin/env python3.7
from traceback import print_tb
import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int16
from rospy.numpy_msg import numpy_msg
from yolo5_ros.msg import int_arrays, string_array, float_array, class_com, float_arrays
import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import math
import os

def convert_pixel_to_m(x, y, depth):
    # Source 1: https://medium.com/@yasuhirachiba/converting-2d-image-coordinates-to-3d-coordinates-using-ros-intel-realsense-d435-kinect-88621e8e733a
    # source 2: https://github.com/ravijo/ros_openpose/blob/b9cbca7f31965af6b5304504e75b9cec1df0f747/include/ros_openpose/cameraReader.hpp#L110-L145
    # Values were drawn from /camera/depth/camera_info and hardcoded here
    # get instrincs parameters
    _intrinsics = rs.intrinsics()
    _intrinsics.width = 640
    _intrinsics.height = 480
    _intrinsics.ppx = 323.4368896484375
    _intrinsics.ppy = 236.62672424316406
    _intrinsics.fx = 379.8893127441406
    _intrinsics.fy = 379.8893127441406
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = rs.distortion.none     
    D = [0.0, 0.0, 0.0, 0.0, 0.0] 
    _intrinsics.coeffs = [i for i in D]
    result = rs.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
    return result[0], result[1], result[2]

def convert_from_cam_to_to_base(x, y ,z ):
    # Gripper Home position
    x_gripper , y_gripper, z_gripper = 0.37, 0.0 ,0.486
    # Camera Offset
    x_offset, y_offset, z_offset = -0.17, -0.0485, -0.9390
    x_base = x + x_gripper + x_offset
    y_base = y + y_gripper + y_offset
    z_base = z + z_gripper + z_offset
    return x_base, y_base, z_base


def box_publisher(bbox):
    # this function takes as input a numpy.narray containing the bounding boxes 
    # of the detected objects and publish them as a rostopic
    pub = rospy.Publisher("bbox", numpy_msg(int_arrays), queue_size=1)
    rate = rospy.Rate(10) # 10hz
    bbox_pub = int_arrays(bbox)
    #rospy.loginfo(bbox_pub)
    pub.publish(bbox_pub)

def coordinates_publisher(bbox, depth):
    coordinates_array = np.ndarray(shape=(len(bbox),3), dtype=float)
    #one pixel image size is 1 micro meter, we convert x and y to meter
    #source: https://en.wikipedia.org/wiki/Intel_RealSense#cite_note-OmniVision_OV2740-33
    resolution = (10**(-6)) 
    for i in range(len(bbox)):
        bbox_x = bbox[i][0]
        bbox_y = bbox[i][1]
        bbox_width = bbox[i][2]
        bbox_height = bbox[i][3]
        # x, y, z in physical imager frame in m
        x, y, z = convert_pixel_to_m((bbox_x+bbox_width)/2.0, (bbox_y+bbox_height)/2.0, depth[i])
        coordinates_array[i][0]= ((bbox_x+bbox_width)/2.0) * resolution
        coordinates_array[i][1]= (bbox_y+bbox_height)/2.0 * resolution
        coordinates_array[i][2]=  depth[i]
    pub = rospy.Publisher("coordinates_list", numpy_msg(float_arrays), queue_size=1)
    rate = rospy.Rate(10)
    coordinates_list_pub = float_arrays(coordinates_array)
    #rospy.loginfo(coordinates_list_pub)
    pub.publish(coordinates_list_pub)
        
def object_classes_publisher(object_classes):
    # this function takes as input a a list of strings containing the object classes 
    # of the detected objects and publish them as a rostopic
    pub = rospy.Publisher("object_classes", string_array , queue_size=1)
    rate = rospy.Rate(10) # 10hz
    object_classes_pub = string_array(object_classes)
    #rospy.loginfo(object_classes_pub)
    pub.publish(object_classes_pub)

def classes_com_publihser(bbox, depth, object_classes):
    coordinates_arrays = float_arrays()
    #one pixel image size is 1 micro meter, we convert x and y to meter
    #source: https://en.wikipedia.org/wiki/Intel_RealSense#cite_note-OmniVision_OV2740-33
    resolution = (10**(-6)) 
    for i in range(len(bbox)):
        bbox_x = bbox[i][0]
        bbox_y = bbox[i][1]
        bbox_width = bbox[i][2]
        bbox_height = bbox[i][3]
        # x, y, z in physical imager frame in m
        x, y, z = convert_pixel_to_m((bbox_x+(bbox_width/2.0)), (bbox_y+(bbox_height/2.0)), depth[i])
        x_base, y_base, z_base = convert_from_cam_to_to_base(x, y, z)
        z = depth[i]
        coordinates_array = float_array()
        coordinates_array.data = [x_base, y_base, z_base]
        coordinates_arrays.data.append(coordinates_array)
    pub = rospy.Publisher("classes_coordinates", numpy_msg(class_com), queue_size=1)
    classes_coordinates_pub = class_com(coordinates = float_arrays(),
        labels= [])
    classes_coordinates_pub.coordinates = coordinates_arrays
    classes_coordinates_pub.labels = object_classes
    # rospy.loginfo(classes_coordinates_pub.coordinates.data)
    pub.publish(classes_coordinates_pub)



# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

W= 640
H = 480
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)

if device_product_line == 'L500':
    W= 960
    H = 540
else:
    W= 640
    H = 480
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

aligned_stream = rs.align(rs.stream.color) # alignment between color and depth
point_cloud = rs.pointcloud()


### Prepare the YOLOv5 object Detector module
print("loading the model")
#model = torch.hub.load('ultralytics/yolov5', 'yolov5l')  # or yolov5m, yolov5l, yolov5x, custom
weights_path = os.path.join(os.path.expanduser("~"), "frankaemikapanda", "src", "yolo5_ros", "models","yolo5_custom", "best_07.07.pt")
model_path = os.path.join(os.path.expanduser("~"), "frankaemikapanda", "src", "yolo5_ros", "models","yolov5")
model = torch.hub.load(model_path, 'custom', path= weights_path,  source='local', force_reload=True) 

names = model.names
COLORS = np.random.uniform(0, 255, size=(len(names), 3))

def getResults(output, detection_threshold = 0.2):
    detection_results = output.pred[0].cpu()
    pred_classes = np.array(detection_results[:, -1])
    # get score for all the predicted objects
    pred_scores = np.array(detection_results[:, -2])
    # get all the predicted bounding boxes
    pred_bboxes = np.array(detection_results[:, :4])
    # get boxes above the threshold score
    boxes = pred_bboxes[pred_scores >= detection_threshold].astype(np.int32)
    classes_ids = detection_results[:, -1][pred_scores >= detection_threshold]
    classes = [names[int(id)] for id in classes_ids]
    scores = pred_scores[pred_scores >= detection_threshold]
    return boxes, classes, scores

def draw_boxes(image, boxes, classes, heights=None, depths = None):
    # read the image with OpenCV
    #image = cv2.cvtColor(np.asarray(image), cv2.COLOR_BGR2RGB)
    for i, box in enumerate(boxes):
        color = COLORS[names.index(classes[i])]
        cv2.rectangle(
            image,
            (int(box[0]), int(box[1])),
            (int(box[2]), int(box[3])),
            color, 2
        )
        if heights is None:
            if depths is None:
                label = ""
            else:
                label =  " : D.: {:.2f} m.".format(depths[i])
        else:
            if depths is None:
                label =  " : H. :{:.2f} m.".format(heights[i])
            else:
                label =  " : H.: {:.2f} m. | D.: {:.2f} m. ".format(heights[i], depths[i])

        cv2.putText(image, "{}{}".format(classes[i],label), (int(box[0]), int(box[1]-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2,
                    lineType=cv2.LINE_AA)
    return image

def getHeight(boxes):
    def processBox(bbox):
        obj_points = verts[int(bbox[1]):int(bbox[1] + bbox[3]), int(bbox[0]):int(bbox[0] + bbox[2])].reshape(-1, 3)
        zs = obj_points[:, 2]
        ys = obj_points[:, 1]
        z = np.median(zs)
        ys = np.delete(ys, np.where((zs < z - 1) | (zs > z + 1)))
        my = np.amin(ys, initial=1)
        My = np.amax(ys, initial=-1)
        height = (My - my)
        return height
    heights = []
    for b in boxes:
        heights.append(processBox(b))
    return heights

def getDepth(boxes):
    def processBox(bbox):
        x, y = int((bbox[0] + bbox[2]) /2), int((bbox[1] + bbox[3])/2)
        depth = depth_frame.get_distance(x, y)
        dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
        distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
        return distance
    depths_objects = []
    for b in boxes:
        depths_objects.append(processBox(b))
    return depths_objects
# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        frames = aligned_stream.process(frames) ## NEW

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        points = point_cloud.calculate(depth_frame)
        verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, W, 3)  # xyz

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        output = model(color_image)
        boxes, classes, scores = getResults(output, detection_threshold=0.2)
        heights = getHeight(boxes)
        depths_objects = getDepth(boxes)


        #color_image = draw_boxes(color_image, boxes, classes, heights=heights, depths=depths_objects)
        color_image = draw_boxes(color_image, boxes, classes, depths=depths_objects)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        rospy.init_node('yolo5_ros_publisher', anonymous=True)
        ## to publish only the bbox uncomment the following
        # box_publisher(boxes)
        ## to publish only the labels uncomment the following
        #object_classes_publisher(classes)
        #coordinates_publisher(boxes, depths_objects)
        ## to publish classes with center of mass uncomment the following
        classes_com_publihser(boxes, depths_objects, classes)

        #camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo)

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)
        

finally:

    # Stop streaming
    pipeline.stop()
