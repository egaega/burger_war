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
        # topic
        self.tf_broadcaster  = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener() # tf
        self.sub_obstacles   = rospy.Subscriber('obstacles', Obstacles, self.obstacles_callback) # obstacle
    
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
        self.setGoal(-0.5,0,0)
        self.setGoal(-0.5,0,3.1415*0.25)
        
        self.setGoal(0,0.5,0)
        self.setGoal(0,0.5,3.1415*1.5)
        self.setGoal(0,0.5,3.1415)
        self.setGoal(0,0.5,3.1415*1.75)
        
        self.setGoal(0.5,0,3.1415*1.5)
        self.setGoal(0.5,0,3.1415)
        self.setGoal(0.5,0,3.1415*1.25)
        
        self.setGoal(0,-0.5,3.1415)
        self.setGoal(0,-0.5,3.1415*0.5)
        self.setGoal(0,-0.5,0)

    def obstacles_callback(self, msg):

        closest_enemy_len = sys.float_info.max
        closest_enemy_x   = 0
        closest_enemy_y   = 0

        for num in range(len(msg.circles)):

            temp_x = msg.circles[num].center.x
            temp_y = msg.circles[num].center.y

            #フィールド内のオブジェクトであればパス
            if self.is_point_emnemy(temp_x, temp_y) == False:
                continue

            #敵の座標をTFでbroadcast
            enemy_frame_name = self.robot_name + '/enemy_' + str(num)
            map_frame_name   = self.robot_name + "/map"
            self.tf_broadcaster.sendTransform((temp_x,temp_y,0), (0,0,0,1), rospy.Time.now(), enemy_frame_name, map_frame_name)

            #ロボットから敵までの距離を計算
            try:
                target_frame_name = self.robot_name + '/enemy_' + str(num)
                source_frame_name = self.robot_name + "/base_footprint"
                (trans,rot) = self.tf_listener.lookupTransform(source_frame_name, target_frame_name, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            len_robot2enemy = math.sqrt(pow(trans[0],2) + pow(trans[1],2))

            if closest_enemy_len > len_robot2enemy:
                closest_enemy_len = len_robot2enemy
                closest_enemy_x   = temp_x
                closest_enemy_y   = temp_y

        #敵を検出している場合、その座標と距離を出力
        if closest_enemy_len < sys.float_info.max:

            map_frame_name   = self.robot_name + "/map"
            enemy_frame_name = self.robot_name + "/enemy_closest"
            self.tf_broadcaster.sendTransform((closest_enemy_x,closest_enemy_y,0), (0,0,0,1), rospy.Time.now(), enemy_frame_name, map_frame_name)

            #ロボットから敵までの距離をpublish
            #self.pub_robot2enemy.publish(closest_enemy_len)

    def is_point_emnemy(self, point_x, point_y):
        #フィールド内の物体でない、敵と判定する閾値（半径）
        thresh_corner = 0.180
        thresh_center = 0.280

        #フィールド内かチェック
        if   point_y > (-point_x + 1.55):
            return False
        elif point_y < (-point_x - 1.55):
            return False
        elif point_y > ( point_x + 1.55):
            return False
        elif point_y < ( point_x - 1.55):
            return False

        #フィールド内の物体でないかチェック
        len_p1 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y - 0.53), 2))
        len_p2 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y + 0.53), 2))
        len_p3 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y - 0.53), 2))
        len_p4 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y + 0.53), 2))
        len_p5 = math.sqrt(pow(point_x         , 2) + pow(point_y         , 2))

        if len_p1 < thresh_corner or len_p2 < thresh_corner or len_p3 < thresh_corner or len_p4 < thresh_corner or len_p5 < thresh_center:
            return False
        else:
            return True

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

