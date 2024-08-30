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
        self.mode = AR_DRIVE #AR Driving
        self.mode_msg = reset()
        self.motor_msg = xycar_motor()
        self.rubber_detected = FALSE
        self.obstacle_detected = FALSE

        self.flag_ar = None
        self.flag_rubber = None
        self.flag_obstacle = None
        self.list_hangul = ["스타트", "1111AR 주행모드", "222라qk 전 트랙", "333라바콘 주행", "444회피 전 트랙", "555회피주행"]

        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.mode_pub = rospy.Publisher('mode', reset, queue_size=1)

        rospy.Subscriber("ar", mode_info, self.ar_callback, queue_size=1)

    def ar_callback(self, data):
        if self.mode == AR_DRIVE and data.mode == TRUE:
            self.drive(data.angle, data.speed)
        elif self.mode == AR_DRIVE and data.mode == FALSE:
            self.move_forward(-5, 3, 2.5)
            self.mode = LANE_DRIVE_FIRST
            self.flag_ar = rospy.get_time()
            
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