#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import random
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
import numpy as np
import math
from tf import TransformListener
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from obstacle_detector.msg import Obstacles
import sys
import math

class MyRobot():
    def __init__(self, bot_name="NoName"):
        # bot names
        self.name = bot_name
        # move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # mode
        self.mode = 0
        # robotname
        self.robot_name = rospy.get_param('~robot_name', '')
        # red side or blue side
        my_color = rospy.get_param('rside')
        if my_color == 'r':
            self.side = 1
        else:
            self.side = -1
        # target
        self.target = self.calcGoal()
        self.next_target = 0
        self.max_target_num = 12
        # tf
        self.tf_broadcaster  = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        # topic
        # self.sub_obstacles   = rospy.Subscriber('obstacles', Obstacles, self.obstacles_callback) # obstacle
    
    def calcGoal(self):
        if self.side == 1:
            th = 0
        else:
            th = math.pi
        TARGET = [
            [-0.5 * self.side, 0, th],                      # bottom
            [-0.5 * self.side, 0, th + (math.pi * 0.25)], 
            [0, 0.5 * self.side, th + (math.pi * 1.0)],     # left
            [0, 0.5 * self.side, th - (math.pi * 0.5)],
            [0, 0.5 * self.side, th],
            [0, 0.5 * self.side, th - (math.pi * 0.25)],
            [0.5 * self.side, 0, th - (math.pi * 0.5)],     # top
            [0.5 * self.side, 0, th + (math.pi * 1.0)],
            [0.5 * self.side, 0, th + (math.pi * 1.25)],
            [0, -0.5 * self.side, th],                      # right
            [0, -0.5 * self.side, th + (math.pi * 0.5)],
            [0, -0.5 * self.side, th + (math.pi * 1.0)]
            
        ]
        return TARGET

    # RESPECT
    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def basic_move(self):
        self.setGoal(self.target[self.next_target][0], self.target[self.next_target][1], self.target[self.next_target][2])
        self.next_target += 1
        if self.next_target >= self.max_target_num:
            self.next_target = 0

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps
        while not rospy.is_shutdown():
            if self.mode == 0:
                self.basic_move()
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('myrobot')
    bot = MyRobot()
    bot.strategy()

