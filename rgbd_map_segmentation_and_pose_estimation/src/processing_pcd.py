#!/usr/bin/env python

#import Rospy
import rospy
#import math lib
from math import *
#import Cmd_Velocity Message
from geometry_msgs.msg import Point32
# import the camera data message
from sensor_msgs.msg import PointCloud, ChannelFloat32

#numpy import
import numpy as np
import os

import rosgraph
import rostopic

import open3d as o3d

# import matplot as plt
import matplotlib.pyplot as plt

class Points_processing():
    
    def __init__(self):
        #intialize the node
        rospy.init_node('points_cloud_processing')
        rospy.loginfo('Starting Node : points_cloud_processing...')

        #define a speed rate of 10Hz
        rate = rospy.Rate(10)
        # transfor listnmer
        
        # Initialize the point cloud data
        self.data_time_stmp = rospy.Time.now()
        # save pcd to file
        self.save_ply = False
        # Get arguments from the launch file
        self.pcd_topic_namespace = rospy.get_param("~pcd_namespaces")  # get as an argument
        self.map_frame_id = rospy.get_param("~map_frame_id")  # get as an argument
        # Ibit the topic namespaces list
        topic_list = []
        rospy.loginfo('Waiting for point cloud to be published...')
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
        self.total_points = [[] for k in topic_list]

        # create a loop tha generate list of 24bits color code in range k
        self.pcd_color_list = [0x66FF00, 0xFF, 0x0022FF, 0xFFF700, 0xFF00F7, 0x00FFC4, 0x4D00FF, 0xB300FF]
        self.file_number = 0
        # publishs infos about the topic
        # show topicsd name
        topic_names = ''
        for iDx, topic in enumerate(topic_list):
            topic_names += '\n' + topic
            if iDx == len(topic_list)-1:
                rospy.loginfo('Founded '+ str(len(topic_list)) +  ' topic(s)' + topic_names)
        # define a list of subscribers and publishers
        self.point_clouds_pubs = []
        self.point_clouds_subs = []
        # define the published end_name
        self.end_name = rospy.get_param("~output_endName") 
        # subscribe to the listed topics
        for IdX, pcd_names_topic in enumerate(topic_list):
            self.point_clouds_subs.append(rospy.Subscriber(pcd_names_topic, PointCloud, self.callback_point_clouds, (IdX)))
            self.point_clouds_pubs.append(rospy.Publisher(pcd_names_topic + self.end_name, PointCloud, queue_size=1))
        # # Go to the main loop
        rospy.loginfo('Starting to process point cloud')
        self.main_loop()

    def callback_point_clouds(self, point_clouds_msg, index):
        pcd_data = point_clouds_msg.points
        if pcd_data != []:
            self.data_time_stmp = point_clouds_msg.header.stamp
            points_converted = self.convert_points(point_clouds_msg.points)
            # get the existings points from the map
            existing_points_msg =  self.point_clouds_data[index]
            # filter the point cloud and get the new message
            filtered_points = self.filter_points_clouds(points_converted, existing_points_msg, index=index)
            self.point_clouds_pubs[index].publish(filtered_points)
            # save point cloud in a file
            if self.save_ply:
                # convert pcd object to array
                pcd_array = self.convert_points(filtered_points.points)
                self.save_pcd_in_a_file(pcd_array, index)
            self.point_clouds_data[index] = filtered_points

    def convert_points(self, points):
        # define an array of points
        point_array = []
        # loop throught all points
        for point in points:
            # convert thepoint into the array
            point_array.append([point.x,
                                point.y,
                                point.z])
        # retunr the numpy array
        return np.array(point_array)

    def filter_points_clouds(self, array_points, existing_points_msg, index=0):
        # create a new open 3D point clud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(array_points)
        # downsample the point cloud
        voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.05)
        # extract inliers from the point cloud
        inliers_pcd = self.get_inlier(voxel_down_pcd)
        # process to clustering and extract the most populated cluster
        clustered_pcd = self.DBSCAN(inliers_pcd)
        # second instance of inliers extraction
        inliers_2_pcd = self.get_inlier(clustered_pcd)
        # publish the new point cloud
        return self.create_pcd_message(inliers_2_pcd, existing_points_msg, index=index)
    
    def display_inlier_outlier(self, cloud, ind):
        inlier_cloud = cloud.select_by_index(ind)
        outlier_cloud = cloud.select_by_index(ind, invert=True)

        # print("Showing outliers (red) and inliers (gray): ")
        outlier_cloud.paint_uniform_color([1, 0, 0])
        inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
        # o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
        return inlier_cloud

    # create a function that save point clouds in a .pcd file
    def save_pcd_in_a_file(self, point_clouds, index=0):
        # check if the number of points as changed since the last iteration
        if len(point_clouds) != self.total_points[index]:
            # increment the file number
            self.file_number += 1
            # get the master path with os module
            master_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),'pcd_maps')   
            # define file prefix
            file_prefix = 'pcd_map_'   
            # define the name of the file
            file_name = os.path.join(master_path,file_prefix + str(index) + '_n'+ str(self.file_number) + '.ply')
            # if the file exist, increment the file_number and try again
            while os.path.isfile(file_name):
                self.file_number += 1
                file_name = os.path.join(master_path,file_prefix + str(index) + '_n'+ str(self.file_number) + '.ply')
            # convert points array into open3d point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(point_clouds)
            # save the file
            o3d.io.write_point_cloud(file_name, pcd, write_ascii=True)
            # print the saved file name
            rospy.loginfo('Saved file: ' + file_prefix + str(index) + '_n'+ str(self.file_number) + '.ply')
            self.total_points[index] = len(point_clouds)

    # create a function that integrate point list and pointcloud into as new message
    def create_pcd_message(self, pcd_to_add, existing_points_msg, index=0):
        pcd_to_check = np.asarray(pcd_to_add.points)  
        # create a new point cloud
        points_c_msg = PointCloud()
        points_c_msg.points = existing_points_msg.points
        # points_c_msg.header.stamp =  rospy.Time(0)
        points_c_msg.header.stamp =  self.data_time_stmp
        points_c_msg.header.frame_id = self.map_frame_id.replace('/','')
        # convert the array Point32 array into a numpy array of points
        points_c_np = np.array([[p.x, p.y, p.z] for p in points_c_msg.points])
        # check if one of the 2 numpy array is empty
        if points_c_np.size == 0:
            # if empty, add the new point cloud
            points_c_np = pcd_to_check
        # remove all the duplicates points from the array
        points_uniques = np.unique(np.concatenate((points_c_np, pcd_to_check)), axis=0)
        # Convert pointcloud with the converted and add it to the message
        points_c_msg.points = [Point32(x=p[0], y=p[1], z=p[2]) for p in points_uniques]
        # ge the number of points in the message
        nb_of_points = len(points_c_msg.points)
        # Change all the points channel['rgb'] color in the message to red
        points_c_msg.channels = [ChannelFloat32("rgb", [self.pcd_color_list[index] for i in range(nb_of_points)])]
        # return the pcd messsage
        return points_c_msg

    def get_inlier(self, pcd):
        # https://towardsdatascience.com/how-to-automate-3d-point-cloud-segmentation-and-clustering-with-python-343c9039e4f5
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=10,
                                                    std_ratio=1.1)
        _, ind = pcd.remove_statistical_outlier(nb_neighbors=10,
                                            std_ratio=2)
        inlier_cloud = pcd.select_by_index(ind)
        outlier_cloud = pcd.select_by_index(ind, invert=True)

        # print("Showing outliers (red) and inliers (gray): ")
        outlier_cloud.paint_uniform_color([1, 0, 0])
        inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
        inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
        # o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
        return inlier_cloud

    # create a DBSCAN clustering
    def DBSCAN(self, pcd):
        # https://towardsdatascience.com/how-to-automate-3d-point-cloud-segmentation-and-clustering-with-python-343c9039e4f5
        # create a label list
        labels = np.array(pcd.cluster_dbscan(eps=0.1, min_points=10))
        max_label = labels.max()
        # convert old pointcloud into numpy array
        data = np.asarray(pcd.points)
        # create an empty segmentation array
        segmentation = [[] for k in range(max_label + 1)]
        
        # check labels consistency
        if max_label == -1:
            # create a new pointcloud
            new_pcd = o3d.geometry.PointCloud()
            # # add the biggest cluster to the new pointcloud
            new_pcd.points = o3d.utility.Vector3dVector([])
            return new_pcd
        # llop throught label to create segmentation
        for iDx, label in enumerate(labels):
            segmentation[label].append(data[iDx])    # IndexError: list index out of range

        # convert numpy array into open3d point cloud
        seg_size = [len(seg) for seg in segmentation]
        # get the index with the max size
        max_index = seg_size.index(max(seg_size))
        # create a new pointcloud
        new_pcd = o3d.geometry.PointCloud()
        # # add the biggest cluster to the new pointcloud
        new_pcd.points = o3d.utility.Vector3dVector(segmentation[max_index])
        # create new pointcloud with only the first cluster        
        return new_pcd

    def main_loop(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            # rospy Spin
            rate.sleep()  
    
if __name__ == '__main__':
    try:
        points_process = Points_processing()
        points_process.main_loop()
        # map_stitching.matchMap()
    except rospy.ROSInterruptException:
        # print(rospy.ROSInterruptException)
        pass


point_clouds