#!/usr/bin/env python3
#-*- coding:utf-8 -*-

"""
라이다 주행 시 인지, 제어 코드가 필요 없음. track_drive 코드처럼 미션별로 모드 번호를 줘서
각 모드 번호에 맞는 주행을 수행할 수 있도록 설정.
"""

import numpy as np
import cv2, rospy, time, math, os
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
from track_drive.msg import mode_info

import math
from collections import deque

# # lane_msgs 디렉토리 안의 msg 디렉토리 안에 있는 LaneInfo.msg 파일.
# from track_drive.msg import LaneInfo

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
Fix_Speed = 10 # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
lidar_points = None  # 라이다 데이터를 담을 변수
heading = 0

LIDAR_DIST = 0.6
TOLERANCE_DIST = 0.35

#=============================================
# 프로그램에서 사용할 이동평균필터 클래스
#=============================================

class Lidar_rubber:

    #라이다 스펙
    """
    frame_id: "laser_frame"

    angle_min: -3.1241390705108643(rad)
    angle_max: 3.1415927410125732(rad)

    range_min: 0.15000000596046448(m)
    range_max: 12.0(m)

    가끔 값이 튀어서 inf(무한대)값 나옴 => inf값 제외하고 평균값 구하기
    """
    def __init__(self):
        self.range_min = 0.0
        self.range_max = 0.0
        self.filtered_list = []
        self.lidar_points = []
        self.speed = 10
        self.mode_msg = mode_info() 
        
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.mode_pub = rospy.Publisher('rubber', mode_info, queue_size=1)
        rospy.wait_for_message("/scan", LaserScan)
    
    def lidar_callback(self, data):
        self.data = data.ranges
        self.lidar_points = list(self.data[:181])
        self.filtered_list = [(index, value) for index, value in enumerate(self.lidar_points) if value <= LIDAR_DIST]
        self.tracking(self.filtered_list)
#=============================================
# 차량을 정차시키는 함수  
# 입력으로 지속시간을 받아 그 시간동안 속도=0 토픽을 모터로 보냄.
# 지속시간은 0.1초 단위임. 만약 15이면 1.5초가 됨.
#=============================================
# def stop_car(duration):
#     for i in range(int(duration)): 
#         drive(angle=0, speed=0)
#         time.sleep(0.1)

    def normalize_steering(self, angle, max_steer = 50):
        if angle >= max_steer:
            angle = max_steer
        elif angle <= -max_steer:
            angle = -max_steer
        return angle

    def tracking(self, lidar_data):
        k = 0.05
        right_data = []
        left_data = []
        heading = 0
        if len(lidar_data) > 10:
            self.speed = 6
            previous_index = None
            for item in lidar_data:
                if item[0] > 60:
                    right_data.append(0.6)
                    break
                if previous_index is None or item[0] == previous_index + 1:
                    right_data.append(item[1])
                    previous_index = item[0]
            previous_index = None
            for item in reversed(lidar_data):
                if item[0] < 130:
                    left_data.append(0.6)
                    break
                if previous_index is None or item[0] == previous_index - 1:
                    left_data.append(item[1])
                    previous_index = item[0]
                else:
                    break        
            right_mean = np.mean(right_data)
            left_mean = np.mean(left_data)
        else:
            self.drive(0, 10, 0)
            return

        error = abs(left_mean - TOLERANCE_DIST)   
        if left_mean > TOLERANCE_DIST or right_mean < TOLERANCE_DIST:
            if error < 0.03:
                heading = -10
            elif error < 0.07:
                heading = -30
            else:
                heading = -50
        else:
            if error < 0.03:
                heading = 10
            elif error < 0.07:
                heading = 30
            else:
                heading = 50
        
        if left_mean > 0.45:
            heading = -50
        heading = self.normalize_steering(heading)
        self.drive(heading, self.speed, 1)

        print("LEFTTTTT :", left_mean)
        print("RIGHTTTT :", right_mean)
        print("Heading :", heading)

    def drive(self, angle, speed, mode):
        self.mode_msg.angle = int(angle)   # 소수점 이하 부분 날아간다는 우려점
        self.mode_msg.speed = int(speed)    # 소수점 이하 부분 날아간다는 우려점
        self.mode_msg.mode = int(mode)
        self.mode_pub.publish(self.mode_msg)

def main():
    rospy.init_node('sensor_node', anonymous=True)
    if not rospy.is_shutdown():
        lidar_rubber = Lidar_rubber()
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
	    pass