#! /usr/bin/env python2.7 

#import Rospy
import rospy
#import math lib
from math import *
#import Cmd_Velocity Message
import geometry_msgs.msg 
#import Lazer msg
from nav_msgs.msg import Odometry
#import array message for Occupancy grid
from nav_msgs.msg import OccupancyGrid, MapMetaData
# import the camera data message
from sensor_msgs.msg import Image
#numpy import
import numpy as np
#import Lazer msg
from sensor_msgs.msg import LaserScan
#import numpy as 
import numpy as np
#import time to count during debugging
import time
#import OpenCV
import cv2
# import the transfrom moduel
import tf
import imutils

class py3_listner():
    
    def __init__(self):
        
        #   define the topic to publish the map in (Using occupancy grid message)
        # self.frame_anot = rospy.Publisher('/camera/rgb/image_anotated', Image, queue_size=1)
        # self.rosbot_1_map = rospy.Subscriber('/{}1/map'.format(self.robots_namespaces), OccupancyGrid, callback=self.map1Get, queue_size=1)
        # self.rosbot_2_map = rospy.Subscriber('/{}2/map'.format(self.robots_namespaces), OccupancyGrid, callback=self.map2Get, queue_size=1)
        # Go to the main loop
        self.test = 0
    
    def tf_listener(self, topic_a, topic_b):
        listener = tf.TransformListener()
        try:
            (trans,rot) = listener.lookupTransform(topic_a, topic_b, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
    def from_static_image_detection(self,frame):
        # For static images:

        mp_drawing = mp.solutions.drawing_utils
        mp_objectron = mp.solutions.objectron
        x=2
        with mp_objectron.Objectron(static_image_mode=True,
                                max_num_objects=5,
                                min_detection_confidence=0.5,
                                model_name='Chair') as objectron:

            image = frame.copy()
            # Convert the BGR image to RGB and process it with MediaPipe Objectron.
            results = objectron.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            self.fps_counter+=1
            if (time.time() - self.fps_start_time) > x :
                self.fps = self.fps_counter / (time.time() - self.fps_start_time)
                rospy.loginfo("3D object detection running at {} {}".format(round(self.fps,2), "fps"))
                self.fps_counter = 0
                self.fps_start_time = time.time()
            # Draw box landmarks.
            if not results.detected_objects:
                # print(f'No box landmarks detected on the camera frame')
                return frame
            print(f'Box landmarks on the camera frame')
            annotated_image = image.copy()
            for detected_object in results.detected_objects:
                mp_drawing.draw_landmarks(
                    annotated_image, detected_object.landmarks_2d, mp_objectron.BOX_CONNECTIONS)
                mp_drawing.draw_axis(annotated_image, detected_object.rotation,
                                detected_object.translation)
            
        return annotated_image

