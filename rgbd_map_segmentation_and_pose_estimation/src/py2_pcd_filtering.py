#!/usr/bin/env python

# import pointcloud object from sensor msg
from sensor_msgs.msg import PointCloud
#import Point Message Message
from geometry_msgs.msg import Point32

# import Convex Hull form scipy
from scipy.spatial import ConvexHull

# import clsuter from sklearn
from sklearn import cluster
# import numpy
import numpy as np

# create a class py2PcdFilters
class py2PcdFilters():
    # create the init function
    def __init__(self, pcd_arr=[], pcd_obj=PointCloud()):
        # create the variables
        # pcd array value
        self.pcd_array = pcd_arr
        # pcd object value
        self.pcd_object = pcd_obj
        # create a list of the main segment
        self.main_segment = []
        # create list of segments for the object
        self.surface = []
        self.convex_hull_edges = []
        self.density = []
        self.segments_2D = []
        # densiest points
        self.densiest_points = []
        # largest points
        self.largest_points = []
        # initialize the densiest and largest index to -1
        self.densiest_index = -1    # must be compute before used
        self.largest_index = -1     # must be compute before used
    
    # define a function that process DBSCAN clustering using sklearn
    def DBSCAN(self, pcd, convert_from_o3d=False, convert_from_ptsList=False):
        data = pcd
        # check if point come from open3d or not
        if convert_from_o3d:
            # convert pointcloud into numpy array
            data = np.array(pcd.points)
        elif convert_from_ptsList:
            # convert points List into numpy array
            data = self.ptsArr2numpyArr(pcd)
        # apply the DBSCAN algorithm of sklearn
        dbscan = cluster.DBSCAN(eps=0.1, min_samples=10).fit(data)
        # get the labels of the data
        labels = dbscan.labels_
        # get the value of the max label
        max_label = np.amax(np.unique(labels))
        segmentation = [[] for k in range(max_label + 1)]
        # loop throught label to create segmentation
        for i in range(len(dbscan.labels_)):
            segmentation[labels[i]].append(data[i])
        # save the segmentation in the class
        self.main_segment = segmentation
        # return the point cloud segmented
        return segmentation
    
    # function that return the cluster with the more points in a segment
    def getBiggestSegment(self, subSegment):
        # create an array of the number of points in each segment
        n_points = [len(segment) for segment in subSegment]
        # get the biggest segment of the array
        biggest_segment = subSegment[np.argmax(n_points)]
        # return the largest segment
        return biggest_segment
    # convert Point32 list to numpy array
    def ptsArr2numpyArr(self, pts_arr):
        # convert pcd pointcloud to an array of points
        np_arr = np.array([[point.x, point.y, point.z] for point in pts_arr], dtype=object)
        # return the numpy array
        return np_arr
    
    # convert numpy array to Point32 list
    def numpyArr2ptsArr(self, np_arr):
        # convert numpy array to pointcloud
        pts_arr = [Point32(x=point[0], y=point[1], z=point[2]) for point in np_arr]
        # return the pointcloud
        return pts_arr

    # calculate the area of the convex hull in counterclockwise order
    def getConvex(self, points):
        # create a convex hull object
        hull = ConvexHull(points) 
        # get the vertices of the convex hull
        vertices = np.stack((points[hull.vertices,0], points[hull.vertices,1]), axis=1)
        # return convex
        return vertices

    # create a function that calculate the percentage of the points that are in the segmnent
    def perc2dSegment(self, segment, pcd):
        # ge the index of the densiest sefgment
        # get the number of points in the segment
        n_points = len(segment)
        # get the number of points in the point cloud
        n_points_cloud = len(pcd)
        # calculate the percentage of the points in the segment
        percentage = round(n_points / n_points_cloud,2)
        return percentage

    # function that calculatet the area in a convecx hull of a points list in clockwise order
    def areaInClockiwiseOrder(self, vertices):
        # Define the area variable
        area = 0
        # Store the lenth of the vertices as a variable
        lenth_vertices = len(vertices)  
        # loop throught all the edges of the polygone
        #(The edges has to ordered in clockwise order)
        for i, vertex in enumerate(vertices):
            # get the 2 init vectors
            va = vertex
            vb = vertices[(i+1) % lenth_vertices] # check dat modulo is weired
            
            # get the width  
            width = vb[0] - va[0]
            # get the height       # #   # #  #       # #   #   #

            height = (vb[1] + va[1]) / 2
            
            # add the total to the area
            area += width * height 
        return area

    # define a function that caclute the cluster areas and density
    def calc2dClustMetrics(self, segmentation=[]):
        # check if the segmentation is empty
        if segmentation == [] and self.main_segment != []:
            # if empty use the main segment
            segmentation = self.main_segment
        # create a list of the surface of each cluster
        surface, convex_hull_edges, density, segments_2D = self.surface, self.convex_hull_edges, self.density, self.segments_2D
        # calculate the convex hull of the biggest of every segments
        for seg in segmentation:
            # convert segment to numpy array
            seg = np.array(seg)
            # extrcact the x and y coordinates in a new array
            xy_seg = np.stack((seg[:, 0], seg[:, 1]), axis=1)
            # segs_2D array
            segments_2D.append(xy_seg)
            # caclulate the convex hull of the segment
            convex_hull = self.getConvex(xy_seg)
            # add the convex hull to the list
            convex_hull_edges.append(convex_hull)
            # calculate the surface of the convex hull in clockwise order (abs of counterclockwise)
            surface.append(abs(round(self.areaInClockiwiseOrder(convex_hull),4)))
            # calculate the density of the segment
            density.append(len(seg) / surface[-1])
            # check if the array is empty
        # keep the 4 largest cluster
        while len(surface) > 4:
            # get the index of the smalest cluster surface
            min_index = np.argmin(surface)
            # remove the biggest array from the list of arrays
            del segments_2D[min_index]
            # remove the biggest array from the list of arrays
            del surface[min_index]
            # remove the biggest array from the list of arrays
            del convex_hull_edges[min_index]
            # remove the biggest array from the list of arrays
            del density[min_index]
            # remive the biggest array from the list of arrays
            del segmentation[min_index]
        # save the index of the densiest cluster and largest cluster
        self.densiest_index = np.argmax(density)
        self.largest_index = np.argmax(surface)
        # get the final main objects
        self.densiest_points = segmentation[self.densiest_index]
        self.largest_points = segmentation[self.largest_index]
        # save all the arrays in the class
        self.surface = surface
        self.convex_hull_edges = convex_hull_edges
        self.density = density
        self.segments_2D = segments_2D
        # return new_pcd, larg_clust
        return self.densiest_points