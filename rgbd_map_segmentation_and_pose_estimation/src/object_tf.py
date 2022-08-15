#!/usr/bin/env python

#import Rospy
import rospy
#import math lib
from math import *
#import Cmd_Velocity Message
from geometry_msgs.msg import Point32
#import array message for Occupancy grid
# import the camera data message
from sensor_msgs.msg import Image, CameraInfo, PointCloud, ChannelFloat32

# pointcloud imports
# import open3d as o3d
#numpy import
import numpy as np
#import OpenCV
import cv2
# import the transfrom moduel
# from py_listner_tf import py3_listner
import os

import rosgraph
import rostopic



# import the clustering library of sklearn
from sklearn import cluster
import tf

#Global variable for the map
# get the yolo network


# import pcl
# --- END OF DEBUGFGING VARIABKES

class object_tf():
    
    def __init__(self):
        #intialize the node
        rospy.init_node('object_tf')
        rospy.loginfo('Starting Node : object_tf...')

        #define a speed rate of 10Hz
        rate = rospy.Rate(10)
        # transfor broadcaster
        self.br = tf.TransformBroadcaster()
        # Points CLoud Message
        self.pcd_topic_namespace = rospy.get_param("~publishers_endName") # get as an argument
        #   define the topic to publish the map in (Using occupancy grid message)
        # self.map_stitched = rospy.Publisher(self.map_stitched_name, OccupancyGrid, queue_size=1)
        #   Subscribe to Odometry and LaserScan to fill the map
        # not sure if any better or different that master=None
        # master = rosgraph.Master('/rostopic')
        # pubs, subs = rostopic.get_topic_list(master=master)
        topic_list = []

        rospy.loginfo('Waiting for filtered point cloud to be published...')
        # get the topic list while not published
        while topic_list == []:
            rospy.sleep(0.2)
            master = rosgraph.Master('/rostopic')
            pubs, subs = rostopic.get_topic_list(master=master)
            for topic in pubs:
                name = topic[0]
                if self.pcd_topic_namespace in name:
                    topic_list.append(name)
            
        self.point_clouds_data = [ PointCloud() for k in topic_list]
        self.saved_pts_length = [0 for k in topic_list]
        # create a loop tha generate list of 24bits color code in range k
        self.file_number = 0
        # define the minimum number of points in a cluster to be considered an object
        self.min_clust_points = int(rospy.get_param("~min_clust_points"))
        self.map_frame_id = rospy.get_param("~map_frame_id")  # get as an argument
        # publishs infos about the topic
        # show topicsd name
        topic_names = ''
        # create an object name's list
        self.objects_name = []
        # display all topic and create object name list
        for iDx, topic in enumerate(topic_list):
            # remove the filtering name at the topic
            new_topic_name = topic.replace(self.pcd_topic_namespace, '') 
            # split the topic at the '/'
            new_topic_list = new_topic_name.split('/')
            # get the object's name and add it to the list
            self.objects_name.append(new_topic_list[-1])
            # display the topic name
            topic_names += '\n' + topic
            if iDx == len(topic_list)-1:
                rospy.loginfo('Founded '+ str(len(topic_list)) +  ' topic(s)' + topic_names)
        # define a list of subscribers and publishers
        self.point_clouds_pubs = []
        self.point_clouds_subs = []
        self.tf_broadcasters = [[] for k in topic_list]
        # subscribe to the listed topics
        for IdX, pcd_names_topic in enumerate(topic_list):
            self.point_clouds_subs.append(rospy.Subscriber(pcd_names_topic, PointCloud, self.callback_point_clouds, (IdX)))
        # Go to the main loop
        # self.allObjects = rospy.Publisher(self.pcd_totimepic_namespace + 'all_objects', PointCloud, queue_size=1)
        # # Go to the main loop
        self.main_loop()

   


    def callback_point_clouds(self, point_clouds_msg, index):
        pcd_data = point_clouds_msg.points
        if pcd_data != []:
            self.data_time_stmp = point_clouds_msg.header.stamp
            # check if there is new point cloud data
            if self.saved_pts_length[index] != len(pcd_data):
                # cluster objects
                self.cluster_objects(pcd_data, index=index)
                # save the new lenght of the point cloud data
                self.saved_pts_length[index] = len(pcd_data)

    def cluster_objects(self, point_cloud, index=0):
        # segment the pcd into probable objects
        segmented_pcd = self.sklearn_DBSCAN(point_cloud)
        # get the centroid of the cluster
        centroid_list = self.find_cluster_centroid(segmented_pcd)
        # broadcast a transform to the centroid of the object
        tf_list = []
        for iDx, centroid in enumerate(centroid_list):
            tf_list.append([
                self.objects_name[index] + '_' + str(iDx),
                centroid, 
            ])
        self.tf_broadcasters[index] = tf_list
            
    def ptsArr2numpyArr(self, pts_arr):
        # convert pcd pointcloud to an array of points
        np_arr = np.array([[point.x, point.y, point.z] for point in pts_arr], dtype=object)
        # return the numpy array
        return np_arr

    def sklearn_DBSCAN(self, pcd):
        # convert pointcloud into numpy array
        data = self.ptsArr2numpyArr(pcd)
        # apply the DBSCAN algorithm of sklearn
        dbscan = cluster.DBSCAN(eps=0.1, min_samples=self.min_clust_points).fit(data)
        # get the labels of the data
        labels = dbscan.labels_
        # get the value of the max label
        max_label = np.amax(np.unique(labels))
        # check labels consistency
        if max_label == -1:

            return np.array([])

        segmentation = [[] for k in range(max_label + 1)]
        # llop throught label to create segmentation
        for i in range(len(dbscan.labels_)):
            segmentation[labels[i]].append(np.array(data[i]))
        # return the point cloud segmented
        return np.array(segmentation, dtype=object)

    def find_cluster_centroid(self, segmentation):
        # create an empty list to store the centroid of the cluster
        centroid_list = []
        # loop throught the segmentation
        for clusters in segmentation:
            clusters = np.array(clusters, dtype=object)
            # get the centroid of the cluster
            centroid = [np.mean(clusters[:,0]), np.mean(clusters[:,1]), np.mean(clusters[:,2])]
            # add the centroid to the list
            centroid_list.append(centroid)
        return centroid_list

    def publish_tfs(self,tf_msgs):
        # get the tf objects
        object_name, object_coordinates = tf_msgs
        # get the opbject coordinates
        x, y, z = object_coordinates
        # tf_broadcasters
        self.br.sendTransform((x, y, z),
                            (0.0, 0.0, 0.0, 1.0),
                            self.data_time_stmp,    # rospy.Time.now(),
                            object_name,
                            self.map_frame_id.replace('/', ''))
        # rospy.loginfo('Published tf from ' + object_name + ' to map')

    def main_loop(self):
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            # self.debugg()
            # odom_sub = message_filters.Subscriber(self.odom_topic, Odometry)
            # info_sub = message_filters.Subscriber('ceamera_info', CameraInfo)
            for tf_broad_obj in self.tf_broadcasters:
                for tf_msgs in tf_broad_obj:
                    if tf_msgs != []:
                        self.publish_tfs(tf_msgs)
                # publish points cloud for test
                # rospy Spin
                rate.sleep()  
    
if __name__ == '__main__':
    try:
        object_tf_01 = object_tf()
        object_tf_01.main_loop()
        # map_stitching.matchMap()
    except rospy.ROSInterruptException:
        # print(rospy.ROSInterruptException)
        pass


