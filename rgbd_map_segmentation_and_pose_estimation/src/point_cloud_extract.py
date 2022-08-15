#!/usr/bin/env python

#import Rospy
import rospy
#import math lib
from math import *
#import Cmd_Velocity Message
from geometry_msgs.msg import Point32
#import Lazer msg
from nav_msgs.msg import Odometry
#import array message for Occupancy grid
# import the camera data message
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from detection_msgs.msg import BoundingBox, BoundingBoxes

#numpy import
import numpy as np
#import time to count during debugging
import time
#import OpenCV
import cv2
# import the transfrom module
import tf
import os
from plyfile import PlyData, PlyElement

from tf.transformations import *
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import imutils
# for int to bytes conversion
import struct
from sklearn.cluster import DBSCAN
#Global variable for the map
# get the yolo network

# --- DEBUGFGING VARIABKES
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

# import pcl
# --- END OF DEBUGFGING VARIABKES

class P3D():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0 

class Points_extract():
    
    def __init__(self):
        global LaserRanges
        global attemp
        #intialize the node
        rospy.init_node('point_clouds_extract')
        rospy.loginfo('Starting Node : point_clouds_extract...')

        #define a speed rate of 10Hz
        rate = rospy.Rate(10)
        #variable to view the number of map upadate
        attemp = 0

        # yolo data
        self.yolo_image_topic = rospy.get_param("~yolo_image_out_topic")
        self.yolo_boxes_topic = rospy.get_param("~yolo_boxes_topic")

        # transfor listnmer
        
        self.listener = tf.TransformListener()

        # Camera variables
        self.camera_infos = CameraInfo()
        self.depth_shape = (0, 0)
        # yolo parameters
        self.object_classes = rospy.get_param("~objects_classes").split(';')
        self.rotation_degrees = int(rospy.get_param("~rotation_degrees"))
        self.probability_threshold = float(rospy.get_param("~probability_threshold"))
        # bouding box messages
        self.bounding_boxes = []
        self.boxes_time = 0
        self.depth_colormap = []
        self.queue_depth = []
        
        # Points CLoud Message
        self.new_points_msgs = [ PointCloud() for k in self.object_classes ]
        self.total_points = [ 0 for k in self.object_classes ]
        self.file_number = 0
        self.point_clouds = [ [] for k in self.object_classes ]
        self.points_tresh = 2 #pointcloud threshold in m
        self.pcd_transform = None
        self.pcd_started = False
        self.fps_start_time = time.time()
        self.fps_counter, self.fps = 0, 0

        self.pcd_extracted = 0
        self.pcd_published = 0
        # start a time counter
        self.time_counter = time.time()

        # get the parameters from launch file
        self.camera_rgb_info_topic = rospy.get_param("~camera_rgb_info_topic")
        # self.camera_depth_info_topic = '/spot/depth/frontright/camera_info'
        self.camera_depth_topic = rospy.get_param("~camera_depth_topic")
        self.camera_frame_id = rospy.get_param("~camera_frame_id")
        self.map_frame_id = rospy.get_param("~map_frame_id")
        self.log_detection = rospy.get_param("~log_detection")
        # self.odom_topic = '/spot/odometry'
        
        self.rgb_infos = rospy.Subscriber(self.camera_rgb_info_topic, CameraInfo, callback=self.get_camera_info, queue_size=1)
        # define publisher arras
        self.publisher_namespaces = rospy.get_param("~publisher_namespaces")
        self.publish_depth = []
        # depth publisher
        for ObJ in self.object_classes:
            self.publish_depth.append(rospy.Publisher(self.publisher_namespaces + ObJ.replace(' ','_').lower(), PointCloud, queue_size=1))
        # Go to the main loop
        self.main_loop()

    def callback(self, depth_image, boxes_detect, yolo_image):
        # check if somthing as been detected
        if boxes_detect.bounding_boxes != []:
            # Depth header for publising
            depth_head = depth_image.header
            # get the transforma te the time
            transform = 0
            if transform != None:
                # display starting message
                if not self.pcd_started:
                    self.pcd_started = True
                    rospy.loginfo('Starting object clouds extraction...')
                # get the transorm for the map frameframe_id
                self.extract_object(
                    self.convert_depth_image(depth_image),
                    self.get_box_list(boxes_detect.bounding_boxes),
                    yolo_image
                )
            #
            self.publish_point_cloud(depth_head)

    def publish_point_cloud(self, depth_head):
        new_pcd_msg = []
        # lop throught point clouds
        for pcd in self.point_clouds:
            
            points_c_msg = PointCloud()
            points_c_msg.header = depth_head
            points_c_msg.header.frame_id = self.camera_frame_id
            points_c_msg.points = pcd
            # check if the pcd is not empty
            if points_c_msg.points == []:
                points_c_msg.header.frame_id = self.map_frame_id
                new_pcd_msg.append(points_c_msg)
                continue
            # debuggin variables
            self.pcd_extracted += 1
            self.pcd_published += 1
            #
            if self.listener.canTransform(self.camera_frame_id, self.map_frame_id, points_c_msg.header.stamp):
                points_c_msg = self.listener.transformPointCloud(self.map_frame_id.replace("/",""), points_c_msg)
            else:
                if self.log_detection:
                    rospy.loginfo("Can't transform point cloud at requested time")
                self.pcd_published -= 1
                
                points_c_msg.points = []
            new_pcd_msg.append(points_c_msg)
        # publish the point_cloud
        self.new_points_msgs = new_pcd_msg

    def extract_object(self, depth_proceed, boxes_coord, yolo_image):
        # get the camera intrinsix
        camera_intrinsic = self.camera_infos.K
        # create the point clouds list
        local_point_clouds, local_points_list = [ [] for k in self.object_classes], []
        # get rot and trans mat from frames
        # Create the list of transforms
        transform_arr = [] #[[Tr_map, Rt_map]]#, [Tr_od, Rt_od]]
        # loop throught all the available points
        for boxes_points in boxes_coord:
            # debbug Object
            if self.log_detection:
                rospy.loginfo('Detected ' + str(boxes_points[0][0]) + ' probability ' + str(round(boxes_points[0][1],2)))
            # check if the image as been rotate during the detection
            x_min_2d, y_min_2d, x_max_2d, y_max_2d = boxes_points[1]
            # 
            if self.rotation_degrees != 0:
                # rotate the image for new reference
                bb_boxes = self.rotate_boxes(boxes_points[1], depth_proceed)
                x_min_2d, y_min_2d, x_max_2d, y_max_2d = bb_boxes
            # create the local object list
            point_object = []
            for x_point in range(x_min_2d, x_max_2d):
                for y_point in range(y_min_2d, y_max_2d):
                    d_2d = depth_proceed[y_point][x_point]
                    if d_2d != 0 and d_2d < self.points_tresh * 1000:
                        # debuggin
                        points = self.get3D_point(x_point, y_point, d_2d, camera_intrinsic, transform_arr)
                        local_point_clouds[self.object_classes.index(boxes_points[0][0])].append(points)
            # get the index
            index = self.object_classes.index(boxes_points[0][0])
            # save the point cloud for debuggin
            # self.save_pcd_plus_image(depth_proceed, local_point_clouds[index], yolo_image,boxes=boxes_coord, index=index)
        # test if an object exist
        for obj_pcd in local_point_clouds:
            if obj_pcd != []:
                self.point_clouds = local_point_clouds #pcd.voxel_down_sample(point_clouds, voxel_size=0.02)
                break
           
    def add_transform(self, Points, transforms):
        # Get the transform atrices
        Tr, Rt = transforms
        #
        points_rot = np.matmul(Points,Rt)
        #
        points_trans = np.add(points_rot, Tr)

        # Init point
        point_x = points_trans[0]
        point_y = points_trans[1]
        point_z = points_trans[2]

        return point_x, point_y, point_z


    def get3D_point(self, x_2D, y_2D, d_2D, intrinsic, transforms_arr, ratioSize=1000):
        
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        # get intrinsic params
        fx_d, fy_d, cx_d, cy_d = intrinsic[0], intrinsic[4], intrinsic[2], intrinsic[5]
        # Calculate point coordfinates in camera frame
        point_x = (x_2D - cx_d) * d_2D / ratioSize / fx_d
        point_y = (y_2D - cy_d) * d_2D/ ratioSize / fy_d
        point_z = d_2D / ratioSize
        #
        for transf in transforms_arr:
            point_x, point_y, point_z = self.add_transform(
                                        [point_x, point_y, point_z],
                                        transf
                                        )
            
        # Matrices
        Points = [point_x, point_y, point_z]
        # Init point
        point = Point32()
        point.x = Points[0]
        point.y = Points[1]
        point.z = Points[2]
        # convert to int 
        # point.x, point.y, point.z = int(point.x), int(point.y), int(point.z)
        return  point #, Points

    def get_camera_info(self, camera_info_msg):
        # get the camera info messages
        self.camera_infos = camera_info_msg

    def convert_depth_image(self, ros_image, get_colored=False):
        # https://stackoverflow.com/questions/64590101/process-depth-image-message-from-ros-with-opencv
        cv_bridge = CvBridge()
        try:
            depth_image = cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        except CvBridgeError, e:
            print e
        depth_array = np.array(depth_image, dtype=np.float32)
        if get_colored:
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_array = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        return depth_array


    def rotate_bound(self, image, angle):
        # https://pyimagesearch.com/2017/01/02/rotate-images-correctly-with-opencv-and-python/

        # grab the dimensions of the image and then determine the
        # center
        (h, w) = image.shape[:2]
        (cX, cY) = (w // 2, h // 2)
        # grab the rotation matrix (applying the negative of the
        # angle to rotate clockwise), then grab the sine and cosine
        # (i.e., the rotation components of the matripublish_depthx)
        M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)
        cos = np.abs(M[0, 0])
        sin = np.abs(M[0, 1])
        # compute the new bounding dimensions of the image
        nW = int((h * sin) + (w * cos))
        nH = int((h * cos) + (w * sin))
        # adjust the rotation matrix to take into account translation
        M[0, 2] += (nW / 2) - cX
        M[1, 2] += (nH / 2) - cY
        # perform the actual rotation and return the image
        return cv2.warpAffine(image, M, (nW, nH))  

    def rotate_boxes(self, bb, original_image):
        # https://stackoverflow.com/questions/61718596/how-to-rotate-bounding-box-in-open-cv-and-crop-it-python
        # get the angle
        angle = self.rotation_degrees
        img = self.rotate_bound(original_image.copy(), angle)
        # convert the boudingBox
        # get the center of the image
        center = [img.shape[0]/2, img.shape[1]/2]
        center_inv = [center[1], center[0]]
        rad_angle = math.radians(-angle)
        min_pt, max_pt = bb[:2], bb[2:]
        # get the rotation matrice
        RotMat = [[cos(rad_angle), -sin(rad_angle)],
                 [sin(rad_angle), cos(rad_angle) ]]
        # Get points
        new_min_pt = np.add(np.matmul(RotMat,np.add(min_pt, np.dot(-1, center_inv))), center)
        new_max_pt = np.add(np.matmul(RotMat,np.add(max_pt, np.dot(-1, center_inv))), center)
        # new_min_pt = np.matmul(RotMat,np.add(min_pt, np.dot(-1, center))) 
        # new_max_pt = np.matmul(RotMat,np.add(max_pt, np.dot(-1, center)))
        # 
        point = [new_min_pt[0], new_max_pt[1], new_max_pt[0], new_min_pt[1]]
        new_bb = np.array(point, dtype=np.int32)
        new_bb_draw = [new_bb]
        return new_bb

    def get_box_list(self, bounding_boxes):
        # check if the message is not empty
        boxes_arr = []
        if bounding_boxes != []:
            for boxes in bounding_boxes:
                # box of shape [[ c, p ], [xmin, ymin, xmax, ymax] ]frame_id
                if boxes.probability >= self.probability_threshold:
                    boxes_arr.append([[boxes.Class,boxes.probability],[boxes.xmin, boxes.ymin, boxes.xmax, boxes.ymax]])
        return boxes_arr

    def draw_boxes(self, depth_colormap, bounding_boxes):
        
        # img = depth_colormap
        # check if the message is not empty

        img = cv2.applyColorMap(cv2.convertScaleAbs(depth_colormap, alpha=0.03), cv2.COLORMAP_JET)
        if bounding_boxes != []:
            for boxes in bounding_boxes:
                l, t, r, b = np.array(boxes[1], dtype=np.int32)
                # l, t, r, b = boxes.xmin, boxes.ymin, boxes.xmax, boxes.ymax
                # Taken from https://github.com/pjreddie/darknet/blob/810d7f797bdb2f021dbe65d2524c2ff6b8ab5c8b/src/image.c#L283-L291
                # via https://stackoverflow.com/questions/44544471/how-to-get-the-coordinates-of-the-bounding-box-in-yolo-object-detection#comment102178409_44592380

                cv2.rectangle(img, (l, t), (r, b), (0, 0, 255), 3)
        #######
        # Publish tiem as box detection time and not anymore as depth franme (2 are arguable I think)
        #######
        return img

    def get_boxes(self, boxes_msg):
        # self.bounding_boxes = boxes_msg.bounding_boxes
        return boxes_msg.bounding_boxes


    ## =================================================================================================================
    # Debugging functions
    # main saving function
    def save_pcd_plus_image(self, depth_image, pcd_points, yolo_image, index=0, boxes=[]):
        # get the lenrgth of the point cloud
        length = len(pcd_points)
        # check if the number of points as changed since the last iteration
        if length != self.total_points[index] and length != 0:
            # increment the file number
            self.file_number += 1
            # get the objects class
            object_class = self.object_classes[index]
            # get the master path with os module
            master_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),'extraction_room_02') 
            # if master path does not exist create it
            if not os.path.exists(master_path):
                os.makedirs(master_path)
            # define the name of the file
            file_pcd_name = os.path.join(master_path,'pcd_' + str(self.file_number) + '_' + object_class + '.ply')
            # if the file exist, increment the file_number and try again
            while os.path.isfile(file_pcd_name):
                self.file_number += 1
                file_pcd_name = os.path.join(master_path,'pcd_' + str(self.file_number) + '_' + object_class + '.ply')
            # define the image file name
            file_depth_name = os.path.join(master_path,'dep_image_' + str(self.file_number) + '_' + object_class + '.png')
            file_image_name = os.path.join(master_path,'rgb_image_' + str(self.file_number) + '_' + object_class + '.png')
            # save the pcd
            self.save_plyfile(pcd_points, file_pcd_name)
            # print the saved file name
            rospy.loginfo('Saved file: ' + object_class + '_n'+ str(self.file_number))
            self.total_points[index] = length

    # save the depth image
    def save_depth_image(self, depth_image, FILE_NAME, boxes=[]):
        # create the depth image to colormap
        # colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # depth + bouding box
        colormap = self.draw_boxes(depth_image, boxes)
        # Saving the image
        cv2.imwrite(FILE_NAME, colormap)

    # save the pcd file
    def save_plyfile(self, pcd, FILE_NAME):
        # convert pcd pointcloud to an array of points
        pts = np.array([[point.x, point.y, point.z] for point in pcd], dtype=object)
        write_text=''
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        pts = list(zip(x, y, z))

        # the vertex are required to a 1-d list
        vertex = np.array(pts, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

        el = PlyElement.describe(vertex, 'vertex')
        PlyData([el], text=write_text).write(FILE_NAME)

    ## =================================================================================================================

    def main_loop(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            # self.debugg()
            depth_sub = message_filters.Subscriber(self.camera_depth_topic, Image)
            yolo_sub = message_filters.Subscriber(self.yolo_image_topic, Image)
            boxes_sub = message_filters.Subscriber(self.yolo_boxes_topic, BoundingBoxes)
            # odom_sub = message_filters.Subscriber(self.odom_topic, Odometry)
            # info_sub = message_filters.Subscriber('ceamera_info', CameraInfo)

            ts = message_filters.TimeSynchronizer([depth_sub, boxes_sub,yolo_sub], 1)
            ts.registerCallback(self.callback)
            # publish points cloud for test

            for Idx, publishers in enumerate(self.publish_depth):
                # publish the corresponding message
                publishers.publish(self.new_points_msgs[Idx])

            # check if 5 seconds have passed
            # if time.time() - self.time_counter > 10:
            #     # reset the counter
            #     self.time_counter = time.time()
            #     # print the pcd_infos
            #     print 'Total of extracted pcd :', self.pcd_extracted
            #     print 'Total of published pcd :', self.pcd_published

            # rospy Spin
            rate.sleep()  
    
if __name__ == '__main__':
    try:
        points_extract = Points_extract()
        points_extract.main_loop()
        # map_stitching.matchMap()
    except rospy.ROSInterruptException:
        # print(rospy.ROSInterruptException)
        pass
