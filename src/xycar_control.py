#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import numpy as np
import cv2, rospy, time, math, os
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
import time
from sensor_msgs.msg import LaserScan
import math
from collections import deque

# lane_msgs 디렉토리 안의 msg 디렉토리 안에 있는 LaneInfo.msg 파일.
from track_drive.msg import LaneInfo, mode_info, reset

START = 0
AR_DRIVE = 1
LANE_DRIVE_FIRST = 2
RUBBER_DRIVE = 3
LANE_DRIVE_SECOND = 4
OBSTACLE_DRIVE = 5

TRUE = 1
FALSE = 0



class XycarController:
    def __init__(self):
        self.mode = LANE_DRIVE_FIRST #AR Driving
        self.mode_msg = reset()
        self.motor_msg = xycar_motor()
        self.rubber_detected = FALSE
        self.obstacle_detected = FALSE

        self.flag_init = rospy.get_time()
        self.flag_rubber = None
        self.flag_obstacle = None
        self.list_hangul = ["스타트", "1111AR 주행모드", "222라qk 전 트랙", "333라바콘 주행", "444회피 전 트랙", "555회피주행"]

        # print("DEBUGGGGGGGGGGGGG")
        #Motor Controssl Publisher
        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.mode_pub = rospy.Publisher('mode', reset, queue_size=1)
        # self.rate = rospy.Rate(10)
        #Mode Detect Subscriber

        rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)   #일부러 큐사이즈뺌"?
        rospy.Subscriber("lane", mode_info, self.lane_callback, queue_size=1)
        rospy.Subscriber("rubber", mode_info, self.rubber_callback, queue_size=1)
        rospy.Subscriber("obstacle", mode_info, self.obstacle_callback, queue_size=1)
          
    def lane_callback(self, data):
        print("mode :", self.list_hangul[self.mode])
        if (self.mode == LANE_DRIVE_FIRST or self.mode == LANE_DRIVE_SECOND) and (self.rubber_detected == FALSE and self.obstacle_detected == FALSE):
            self.drive(data.angle, data.speed)
        elif self.mode == LANE_DRIVE_FIRST and self.flag_obstacle != None and rospy.get_time() - self.flag_obstacle < 13.0:
            self.drive(data.angle, data.speed)
        elif self.mode == LANE_DRIVE_FIRST and self.flag_init != None and rospy.get_time() - self.flag_init < 6.0:
            self.drive(data.angle, data.speed)
        elif self.mode == LANE_DRIVE_SECOND and self.flag_rubber != None and rospy.get_time() - self.flag_rubber < 10.0:    
            self.drive(data.angle, data.speed)
        elif self.mode == LANE_DRIVE_FIRST and self.rubber_detected == TRUE:
            self.mode = RUBBER_DRIVE
        elif self.mode == LANE_DRIVE_SECOND and self.obstacle_detected == TRUE:
            self.mode = OBSTACLE_DRIVE

        self.send_mode()
        
    def rubber_callback(self, data):
        if self.mode == RUBBER_DRIVE and data.mode == TRUE:
            self.drive(data.angle, data.speed)
        elif self.mode == RUBBER_DRIVE and data.mode == FALSE:
            self.move_forward(0, 10, 0.1)
            self.mode = LANE_DRIVE_SECOND
            self.flag_rubber = rospy.get_time()
            # self.send_mode()
            
    def obstacle_callback(self, data):
        if self.mode == OBSTACLE_DRIVE and data.mode == TRUE:
            self.drive(data.angle, data.speed)
        elif self.mode == OBSTACLE_DRIVE and data.mode == FALSE:
            self.move_forward(0, 10, 0.1)
            self.mode = LANE_DRIVE_FIRST
            self.flag_obstacle = rospy.get_time()
            # self.send_mode()
            
    def lidar_callback(self, data):
        self.data = data.ranges
        self.lidar_points = list(self.data[40:140])
        self.filtered_list = [(index, value) for index, value in enumerate(self.lidar_points) if value <= 0.6]
        if len(self.filtered_list) > 10:
            self.rubber_detected = TRUE
        else:
            self.rubber_detected = FALSE
        
        self.lidar_points = list(self.data[60:120])
        self.filtered_list = [(index, value) for index, value in enumerate(self.lidar_points) if value <= 0.7]
        if len(self.filtered_list) > 10:
            self.obstacle_detected = TRUE
        else:
            self.obstacle_detected = FALSE
            
    def drive(self, angle, speed):
        self.motor_msg.angle = int(angle)   # 소수점 이하 부분 날아간다는 우려점
        self.motor_msg.speed = int(speed)    # 소수점 이하 부분 날아간다는 우려점
        print("Angle :", self.motor_msg.angle)
        print("Speed :", self.motor_msg.speed)
        print("Mode :", self.mode)
        self.motor_pub.publish(self.motor_msg)
    
    def move_forward(self, angle, speed, time):
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < time: 
            self.drive(angle, speed)

    def send_mode(self):
        # print("Publish mode")
        self.mode_msg.mode = self.mode
        self.mode_pub.publish(self.mode_msg)

def main():
    rospy.init_node('xycar_control')
    if not rospy.is_shutdown():
        xycar_move = XycarController()
        # print("Mode", xycar_move.mode)     #??????????????
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass