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

# pointcloud imports
# import open3d as o3d
#numpy import
import numpy as np
#import time to count during debugging
import time
#import OpenCV
import cv2
# import the transfrom moduel
# from py_listner_tf import py3_listner
import tf2_ros
# import tf

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
        self.yolo_image_topic = '/yolov5/image_out' #rospy.get_param("~left_camera")
        self.yolo_boxes_topic = '/yolov5/detections' #rospy.get_param("~right_camera")

        # transfor listnmer
        
        # self.listener = tf.TransformListener()
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        # USUALS PARAM
        self.front_frame = []
        self.anoted_front_img = Image()
        self.anoted_front_img_data = []
        self.frame_id_name = 'image_anotated'
        self.fH, self.fW = 0,0




        # Camera variables
        self.camera_infos = CameraInfo()
        self.depth_shape = (0, 0)
        # yolo parameters
        self.object_classes = ['Door', 'Stairs', 'Door handle']
        self.rotation_degrees = 90
        # bouding box messages
        self.bounding_boxes = []
        self.boxes_time = 0
        self.depth_colormap = []
        self.queue_depth = []
        
        # Points CLoud Message
        self.new_points_msgs = [ PointCloud() for k in self.object_classes ]
        self.point_clouds = [ [] for k in self.object_classes ]
        self.points_tresh = 0.4 #tf2_listner pointcloud threshold in m
        self.pcd_transform = None
        self.pcd_started = False
        self.fps_start_time = time.time()
        self.fps_counter, self.fps = 0, 0
        # world fram coordinates
        self.worldH, self.worldW = 0,0
        #   define the topic to publish the map in (Using occupancy grid message)
        # self.map_stitched = rospy.Publisher(self.map_stitched_name, OccupancyGrid, queue_size=1)
        #   Subscribe to Odometry and LaserScan to fill the map
        self.camera_rgb_info_topic = '/spot/camera/frontright/camera_info'#rospy.get_param("~right_camera")
        self.camera_depth_info_topic = '/spot/depth/frontright/camera_info'
        self.camera_depth_topic = '/spot/depth/frontright/depth_in_visual'#rospy.get_param("~right_camera")
        self.odom_topic = '/spot/odometry'
        
        # 
        # self.boxes_sub = rospy.Subscriber(self.yolo_boxes_topic, BoundingBoxes, callback=self.get_boxes, queue_size=10)

        self.rgb_infos = rospy.Subscriber(self.camera_rgb_info_topic, CameraInfo, callback=self.get_camera_info, queue_size=1)
        self.depth_infos = rospy.Subscriber(self.camera_depth_info_topic, CameraInfo, callback=self.get_depth_info, queue_size=1)
        # self.frame_anot = rospy.Publisher('/camera/rgb/image_anotated', Image, queue_size=1)
        # define publisher arras
        self.publish_depth = []
        # depth publisher
        for ObJ in self.object_classes:
            self.publish_depth.append(rospy.Publisher('/point_extract/objects/'+ ObJ.replace(' ','_').lower(), PointCloud, queue_size=1))
        # Go to the main loop
        self.main_loop()

    def callback(self, depth_image, boxes_detect):
        # check if somthing as been detected
        if boxes_detect.bounding_boxes != []:
            # Depth header for publising
            depth_head = depth_image.header
            # get the transforma te the time
            # transform = self.tf2_listner('map','frontright_fisheye',  time=depth_head.stamp)
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
                    transform
                )
            #
            self.publish_point_cloud(depth_head)

    def publish_point_cloud(self, depth_head):
        new_pcd_msg = []
        # lop throught point clouds
        for pcd in self.point_clouds:
            
            points_c_msg = PointCloud()
            points_c_msg.header =  depth_head
            points_c_msg.header.frame_id =  'frontright_fisheye'
            points_c_msg.points = pcd
            new_pcd_msg.append(points_c_msg)
        # publish the point_cloud
        self.new_points_msgs = new_pcd_msg # work in progress

    def extract_object(self, depth_proceed, boxes_coord, trans):
        # get the camera intrinsix
        camera_intrinsic = self.camera_infos.K
        # create the point clouds list
        local_point_clouds, local_points_list = [ [] for k in self.object_classes], []
        # get rot and trans mat from frames
        # Tr_map, Rt_map = self.get_matrices(trans.transform.translation,
        #                                    trans.transform.rotation)
        # get rot and trans mat for odom
        # Tr_od, Rt_od = self.get_matrices(odom.pose.pose.position, 
                                        # odom.pose.pose.orientation)
        # Create the list of transforms
        transform_arr = [] #[[Tr_map, Rt_map]]#, [Tr_od, Rt_od]]
        # loop throught all the available points
        for boxes_points in boxes_coord:
            # debbug Object
            rospy.loginfo('Detected ' + str(boxes_points[0][0]) + ' probability ' + str(boxes_points[0][1]))
            # check if the image as been rotate during the detection
            x_min_2d, y_min_2d, x_max_2d, y_max_2d = boxes_points[1]
            # 
            if self.rotation_degrees != 0:
                # rotate the image for new reference
                bb_boxes = self.rotate_boxes(boxes_points[1], depth_proceed)
                x_min_2d, y_min_2d, x_max_2d, y_max_2d = bb_boxes
            # get if the point is not outside the range of stereo camera
            # w_d, w_c = np.amin(self.depth_shape), np.amax() # TO-DO
            # get the bouding boxes points
            # x_min_2d, y_min_2d = boxes_points[1][0], boxes_points[1][1]
            # x_max_2d, y_max_2d = boxes_points[1][2], boxes_points[1][3]
            # create the local object list
            point_object = []
            for x_point in range(x_min_2d, x_max_2d):
                for y_point in range(y_min_2d, y_max_2d):
                    d_2d = depth_proceed[y_point][x_point]
                    if d_2d != 0:# and d_2d > self.points_tresh * 1000:
                        # debuggin
                        points = self.get3D_point(x_point, y_point, d_2d, camera_intrinsic, transform_arr)
                        local_point_clouds[self.object_classes.index(boxes_points[0][0])].append(points)
                        # local_points_list.append(point_arr)
            # Points cloud at the current time
            # point_clouds.append(point_object)
        # test if an object exist
        for obj_pcd in local_point_clouds:
            if obj_pcd != []:
                # self.filter_pcd(local_points_list)
                self.point_clouds = local_point_clouds #pcd.voxel_down_sample(point_clouds, voxel_size=0.02)
                break

    def filter_pcd(self,local_pcd):
        # 
        clustering = DBSCAN(eps=2, min_samples=2, metric='euclidean').fit(local_pcd)
        # clustering = DBSCAN(eps=0.5, *, min_samples=1, metric='euclidean', metric_params=None, algorithm='auto', leaf_size=30, p=None, n_jobs=None).fit(local_pcd)
        # debbug
        print np.unique(clustering.labels_)
    def get_matrices(self, trans, rot):
        # conveert quat to Radians
        rot_mat = [rot.x, rot.y, rot.z, rot.w]
        rot.x, rot.y, rot.z = euler_from_quaternion(rot_mat)
        # Create the rotations matrices
        Rx = [[1,     0,     0],
              [0, cos(rot.x), -sin(rot.x)],
              [0, sin(rot.x), cos(rot.x)]]
              #
        Ry = [[cos(rot.y), 0, sin(rot.y)],
              [0,     1,     0],
              [-sin(rot.y),0, cos(rot.y)]]
              #
        Rz = [[cos(rot.z),-sin(rot.z), 0],
              [sin(rot.z), cos(rot.z), 0],
              [0,    0,     1]]
        Rt = np.matmul(Rx,Ry)
        Rt = np.matmul(Rt,Rz)
        Tr = [trans.x, trans.y, trans.z]

        return Tr, Rt

    def pcd_filtering(self,pcd_list):
        # x, y, z = pcd_list[:,0], pcd_list[:,1], pcd_list[:,2]
        # pcd=np.column_stack((x,y,z))
        # mask=z>np.mean(z)
        # spatial_query=pcd[z>np.mean(z)]
        # pcd.shape==spatial_query.shape 
        #plotting the results 3D

        # Processing using PCL
        cloud = pcl.PointCloud()
        cloud_projected = pcl.PointCloud()

        # Points
        cloud.from_array(pcd_list)

        proj = cloud.make_ProjectInliers()
        proj.set_model_type(pcl.SACMODEL_PLANE)

        # Process to filetring
        cloud_projected = proj.filter()
        #
        points_filtered = []
        # Rebuild the points object
        for i in range(0, cloud_projected.size):
            # Create the point object
            point = Point32()
            # Fill the point object
            point.x = cloud_projected[i][0]
            point.y = cloud_projected[i][1]
            point.z = cloud_projected[i][2]
            # fill the main object
            points_filtered.append(point)
        # return the point cloud
        return points_filtered
        
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

    def get_depth_info(self, camera_info_msg):
        self.depth_shape = (camera_info_msg.width, camera_info_msg.height)

    def convert_depth_image(self, ros_image, get_colored=False):
        # https://stackoverflow.com/questions/64590101/process-depth-image-message-from-ros-with-opencv
        cv_bridge = CvBridge()
        try:
            depth_image = cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
            # depth_image = self.rotate_image(depth_image, self.rotation_degrees)
        except (CvBridgeError, e):
            print(e)
        depth_array = np.array(depth_image, dtype=np.float32)
        if get_colored:
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_array = cv2.applyColorrotation_degreesMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        return depth_array



    def get_rgb_frame(self, camera_msg):
        im = np.frombuffer(camera_msg.data, dtype=np.uint8).reshape(camera_msg.height, camera_msg.width, -1)
        front_frame = im

    def get_depth_frame(self, depth_msg):
        rot = tf_fr.transform.rotation
        rot_mat = [rot.x, rot.y, rot.z, rot.w]
        r, p, y = euler_from_quaternion(rot_mat)
        roll, pitch, yaw = math.degrees(r), math.degr
        # self.rosbot_2_map = rospy.Subscriber('/{}2/map'.format(self.robots_namespaces), OccupancyGrid, callback=self.map2Get, queue_size=1)ees(p), math.degrees(y)
        print('R, p, Y = ',roll, pitch, yaw)

    def rotate_image(self,image_original, angle):
        image = image_original.copy()
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result

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
        # debuggin
        # img = self.draw_boxes(original_image, new_bb_draw)
        # cv2.imshow(str(0), img) #self.rotate_bound(img, angle))
        # cv2.waitKey(1)  # 1 millisecond
        return new_bb


    def get_box_list(self, bounding_boxes):
        # check if the message is not empty
        boxes_arr = []
        if bounding_boxes != []:
            for boxes in bounding_boxes:
                # box of shape [[ c, p ], [xmin, ymin, xmax, ymax] ]frame_id
                boxes_arr.append([[boxes.Class,boxes.probability],[boxes.xmin, boxes.ymin, boxes.xmax, boxes.ymax]])
        return boxes_arr

    def draw_boxes(self,depth_colormap,bounding_boxes):
        
        # img = depth_colormap
        # check if the message is not empty

        img = cv2.applyColorMap(cv2.convertScaleAbs(depth_colormap, alpha=0.03), cv2.COLORMAP_JET)
        if bounding_boxes != []:
            for boxes in bounding_boxes:
                l, t, r, b = np.array(boxes, dtype=np.int32)
                # l, t, r, b = boxes.xmin, boxes.ymin, boxes.xmax, boxes.ymax
                # Taken from https://github.com/pjreddie/darknet/blob/810d7f797bdb2f021dbe65d2524c2ff6b8ab5c8b/src/image.c#L283-L291
                # via https://stackoverflow.com/questions/44544471/how-to-get-the-coordinates-of-the-bounding-box-in-yolo-object-detection#comment102178409_44592380

                cv2.rectangle(img, (l, t), (r, b), (0, 0, 255), 3)
        #######
        # Publish tiem as box detection time and not anymore as depth franme (2 are arguable I think)
        #######
        return img

    def depth_visual_object():
        pass

        # self.rosbot_2_map = rospy.Subscriber('/{}2/map'.format(self.robots_namespaces), OccupancyGrid, callback=self.map2Get, queue_size=1)
    def debugg(self):
        # print('')
        # gripper_color_camera_z_forward
        if get_trans != None:
            # print(get_trans)
            pass
        # print(self.tf_listener('head','body'))

    def get_frame(self, camera_msg):
        fH, fW = camera_msg.height, camera_msg.width
        im = np.frombuffer(camera_msg.data, dtype=np.uint8).reshape(camera_msg.height, camera_msg.width, -1)
        front_frame = im

    def get_boxes(self, boxes_msg):
        # self.bounding_boxes = boxes_msg.bounding_boxes
        return boxes_msg.bounding_boxes
        # self.boxes_time = float(str(boxes_msg.header.stamp.secs) + '.' + str(boxes_msg.header.stamp.nsecs))
        
        # message of type list of dict :
            # Class, prob xmin, ymin, xmax, ymax

    def pub_anotated_frame(self):

        # check if map published
        self.anoted_front_img.header.frame_id = "camera_depth_frame"
        self.anoted_front_img.header.stamp = rospy.Time.now()
        #
        self.anoted_front_img.height, self.anoted_front_img.width = self.fH, self.fW

        final_flatten = np.array(self.anoted_front_img_data).flatten()  

        self.anoted_front_img.encoding = "rgb8"
        self.anoted_front_img.data = final_flatten.tobytes() 
        self.frame_anot.publish(self.anoted_front_img) 
    
    def tf2_listner(self, topic_a, topic_b, time=rospy.Time(0)):
        try:
            trans = self.tfBuffer.lookup_transform(topic_a, topic_b, time)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # print('No Transform at requested time')
            trans = self.pcd_transform
        return trans


    def tf_listner(self, topic_a, topic_b, time=rospy.Time(0)):
        try:
            (trans,rot) = self.listener.lookupTransform(topic_a, topic_b, time)
            # self.pcd_transform = trans, rot
            print topic_a
            print trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException)
            # trans, rot = self.pcd_transform 
            pass


    def main_loop(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            # self.debugg()
            depth_sub = message_filters.Subscriber(self.camera_depth_topic, Image)
            boxes_sub = message_filters.Subscriber(self.yolo_boxes_topic, BoundingBoxes)
            # odom_sub = message_filters.Subscriber(self.odom_topic, Odometry)
            # info_sub = message_filters.Subscriber('ceamera_info', CameraInfo)

            ts = message_filters.TimeSynchronizer([depth_sub, boxes_sub], 1)
            ts.registerCallback(self.callback)
            # publish points cloud for test

            for Idx, publishers in enumerate(self.publish_depth):
                # publish the corresponding message
                publishers.publish(self.new_points_msgs[Idx])

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




    # def old_convert_depth_image(self, ros_image):
    #     # https://stackoverflow.com/questions/64590101/process-depth-image-message-from-ros-with-opencv
    #     cv_bridge = CvBridge()
    #     try:
    #         # get the time stamp
    #         time_stamp_secs = float(str(ros_image.header.stamp.secs) + '.' + str(ros_image.header.stamp.nsecs))
    #         if self.queue_depth != []:
    #             self.queue_depth.append([ros_image, time_stamp_secs])
    #             ros_image = self.queue_depth[0][0]
    #             time_stamp_secs = self.queue_depth[0][1]

    #         time_stamp = ros_image.header.stamp
    #         depth_image = cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
    #         depth_image = self.rotate_image(depth_image, self.rotation_degrees)
    #     except (CvBridgeError, e):
    #         print(e)
    #     depth_array = np.array(depth_image, dtype=np.float32)
    #     # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    #     depth_colormap = []
    #     depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    #     #
    #     try:
    #         new_depth, box_stamp = self.draw_boxes(depth_colormap)
    #         # print(round(time_stamp_secs - box_stamp,3))
    #         # check thebest time 
    #         minStp = []
    #         for timesStp in self.queue_depth:
    #             minStp.append(abs(timesStp[1] - box_stamp))
    #         iDx_queue = 0
    #         if minStp !=  []:
    #             iDx_queue = np.argmin(minStp)
            
    #         if time_stamp_secs - box_stamp > 1:
    #             self.queue_depth.append(ros_image)
    #             new_depth = depth_colormap
    #         # if time_stamp_secs - box_stamp > 0.6:
    #         if iDx_queue == len(self.queue_depth):
    #             iDx_queue -= 1
    #         self.queue_depth = self.queue_depth[iDx_queue+1:]

    #         depth_msg = cv_bridge.cv2_to_imgmsg(new_depth, encoding='passthrough')
    #         depth_msg.header.stamp = time_stamp
    #         self.publish_depth.publish(depth_msg)
    #     except (CvBridgeError, e):
    #         print(e)
