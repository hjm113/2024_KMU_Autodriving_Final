#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import numpy as np
import cv2, rospy, time, math, os
from std_msgs.msg import Int32MultiArray
import time

import math
from collections import deque

# lane_msgs 디렉토리 안의 msg 디렉토리 안에 있는 LaneInfo.msg 파일.
from track_drive.msg import LaneInfo
from track_drive.msg import mode_info

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수
# Fix_Speed = 8  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
# new_speed = Fix_Speed  # 모터 속도 초기값
start_time = time.time()

#=============================================
# 프로그램에서 사용할 이동평균필터 함수
#=============================================
def moving_filter(self, input_value, queue):
    queue.append(input_value)
    weights = [70/85, 9/85, 3/85, 2/85, 1/85]
    #gain 값을 바꿔 보자
    weighted_sum = sum([a*b for a, b in zip(queue, weights)])
    
    return weighted_sum
        
#=============================================
# 프로그램에서 사용할 PID 클래스
#=============================================  
class PID:

    def __init__(self, kp, ki, kd):
        # PID 게인 초기화
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        # 이전 오차 초기화
        self.cte_prev = 0.0
        # 각 오차 초기화
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0
        # 적분오차 제한값 설정
        self.i_min = -10
        self.i_max = 10

    def pid_control(self, cte):
        # 미분오차 계산
        self.d_error = cte - self.cte_prev
        # 비례오차 계산
        self.p_error = cte
        # 적분오차 계산 및 제한 적용
        self.i_error += cte
        self.i_error = max(min(self.i_error, self.i_max), self.i_min)
        # 이전 오차 업데이트
        self.cte_prev = cte

        # PID 제어 출력 계산
        return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error

class Driving():
        
    def __init__(self): 
        rospy.init_node('driver', anonymous=True)

        # 모터 토픽
        self.mode_pub = rospy.Publisher('lane', mode_info, queue_size=1)
        
        # 속도 고정 값 
        self.Fix_Speed = 10 #5, 7 r
        # 최대 조향각
        self.max_steer = 40 # 50 -> 40
        
        # 큐 초기화
        self.queue_lat_err = deque(maxlen=5)
        self.queue_head_err = deque(maxlen=5)

        # PID 초기화
        self.pid = PID(kp=1.0, ki=0.1, kd=0.01)
        
        # 구독자 설정
        rospy.Subscriber('lane_information', LaneInfo, self.lane_callback, queue_size=1)
        
        # 라이다 위치
        self.lidar_pos = (300,240)

        # 모터 메시지 초기화
        self.mode_msg = mode_info()
        self.lane_info = LaneInfo()

    def lane_callback(self, data):
        self.lane_info = data
        target_point = [self.lane_info.left, self.lane_info.mid, self.lane_info.right]
        new_angle = self.stanley_controller(target_point, data.angle)
        new_angle = self.pid.pid_control(new_angle)
        self.drive(new_angle, self.Fix_Speed)

    def moving_filter(self, input_value, queue):
        queue.append(input_value)
        weights = [70/85, 9/85, 3/85, 2/85, 1/85]
        weighted_sum = sum([a*b for a, b in zip(queue, weights)])
        return weighted_sum

    def normalize_angle(self, angle):
        if angle >= self.max_steer:
            angle = self.max_steer
        elif angle <= -self.max_steer:
            angle = -self.max_steer
        return angle

    def stanley_controller(self, road_points, target_angle):
        if abs(target_angle) < 8:
            k = 0.01
        else:
            k = 0  # Stanley 상수 0.001
        k_angle = 0.85 # 1.2,2 speed 5,6 1.5 is 10

        left = road_points[0]
        mid = road_points[1]
        right = road_points[2]

        lateral = mid - self.lidar_pos[0]
        lateral_error = math.degrees(math.atan(k * lateral / self.Fix_Speed))
        heading_error = math.degrees(target_angle) * k_angle

        lateral_error_filtered = self.moving_filter(lateral_error, self.queue_lat_err)
        heading_error_filtered = self.moving_filter(heading_error, self.queue_head_err)

        steering = lateral_error_filtered + heading_error_filtered
        print("lateral :", lateral_error_filtered)
        print("heading :", heading_error_filtered)
        return steering

    def drive(self, angle, speed):
        global start_time
        self.mode_msg.angle = int(self.normalize_angle(angle))  # 소수점 이하 포함X
        self.mode_msg.speed = int(speed)  # 소수점 이하 포함X
        self.mode_msg.mode = 0
        self.mode_pub.publish(self.mode_msg)

    def stop_car(self, duration):
        for i in range(int(duration)): 
            self.drive(angle=0, speed=0)
            time.sleep(0.1)

def main():
     if not rospy.is_shutdown():
        lane_follower = Driving()
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass