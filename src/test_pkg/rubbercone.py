#!/usr/bin/env python3
#-*- coding:utf-8 -*-

"""
라이다 주행 시 인지, 제어 코드가 필요 없음. track_drive 코드처럼 미션별로 모드 번호를 줘서
각 모드 번호에 맞는 주행을 수행할 수 있도록 설정.
"""

import numpy as np
import cv2, rospy, time, math, os
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan

import math
from collections import deque

# # lane_msgs 디렉토리 안의 msg 디렉토리 안에 있는 LaneInfo.msg 파일.
# from track_drive.msg import LaneInfo

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수
Fix_Speed = 5  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값

lidar_points = None  # 라이다 데이터를 담을 변수

motor_msg = xycar_motor()  # 모터 토픽 메시지


#=============================================
# 프로그램에서 사용할 이동평균필터 클래스
#=============================================
class MovingAverage:

    # 클래스 생성과 초기화 함수 (데이터의 개수를 지정)
    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    # 새로운 샘플 데이터를 추가하는 함수
    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data.pop(0)  # 가장 오래된 샘플 제거
            self.data.append(new_sample)

    # 저장된 샘플 데이터의 갯수를 구하는 함수
    def get_sample_count(self):
        return len(self.data)

    # 이동평균값을 구하는 함수
    def get_mavg(self):
        if not self.data:
            return 0.0
        return float(sum(self.data)) / len(self.data)

    # 중앙값을 사용해서 이동평균값을 구하는 함수
    def get_mmed(self):
        if not self.data:
            return 0.0
        return float(np.median(self.data))

    # 가중치를 적용하여 이동평균값을 구하는 함수        
    def get_wmavg(self):
        if not self.data:
            return 0.0
        s = sum(x * w for x, w in zip(self.data, self.weights[:len(self.data)]))
        return float(s) / sum(self.weights[:len(self.data)])

    # 샘플 데이터 중에서 제일 작은 값을 반환하는 함수
    def get_min(self):
        if not self.data:
            return 0.0
        return float(min(self.data))
    
    # 샘플 데이터 중에서 제일 큰 값을 반환하는 함수
    def get_max(self):
        if not self.data:
            return 0.0
        return float(max(self.data))


def filtering(inf_start, inf_end, ranges):
    for k in range(inf_start,inf_end+1):
        ranges[k] = ranges[inf_start-1] + ((ranges[inf_end+1]-ranges[inf_start-1]) / (inf_end - inf_start + 2)) * (k - inf_start + 1)

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
        self.lidar_points = []
        self.previous_lidar_points = [] # 이전 데이터
        self.range_min = 0.0
        self.range_max = 0.0

        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

    def lidar_callback(self, data):
        if data and data.ranges:
            self.data = data.ranges
            self.lidar_points = list(self.data[:181])
            self.previous_lidar_points = self.lidar_points
            self.range_min = data.range_min
            self.range_max = data.range_max
        else:
            rospy.logwarn("라이다 데이터 수신 실패")
            self.lidar_points = self.previous_lidar_points
    
    def filtering(self):
        # 거리 데이터, 측정 가능 거리 임계값
        dist_thresh = 0.55
        filtered_list = [[index, value] for index, value in enumerate(self.lidar_points) if value <= dist_thresh]
        return filtered_list

#=============================================
# 모터 토픽을 발행하는 함수 
#=============================================
def drive(angle, speed):
    motor_msg.angle = int(angle)   # 소수점 이하 부분 날아간다는 우려점
    motor_msg.speed = int(speed)    # 소수점 이하 부분 날아간다는 우려점
    motor.publish(motor_msg)
    
#=============================================
# 차량을 정차시키는 함수  
# 입력으로 지속시간을 받아 그 시간동안 속도=0 토픽을 모터로 보냄.
# 지속시간은 0.1초 단위임. 만약 15이면 1.5초가 됨.
#=============================================
def stop_car(duration):
    for i in range(int(duration)): 
        drive(angle=0, speed=0)
        time.sleep(0.1)

max_steer = 50

def normalize_steering(angle):
    if angle >= max_steer:
        angle = max_steer
    elif angle <= -max_steer:
        angle = -max_steer
    return angle

def tracking(lidar_data, prev_lateral_err=0, prev_heading_err=0):
    k = 0.1

    angle_thresh = 60
    angle_increment = 0.01745329238474369
    left_data_x = []
    left_data_y = []

    right_data_x = []
    right_data_y = []


    # 0~180
    for i in range(len(lidar_data)):
        # 0도 ~ 30도
        if lidar_data[i][0] <= angle_thresh:
            right_data_x.append(lidar_data[i][1] * np.cos(angle_increment * lidar_data[i][0]))
            right_data_y.append(lidar_data[i][1] * np.sin(angle_increment * lidar_data[i][0]))

        # 150도 ~ 180도
        elif lidar_data[i][0] >= 180 - angle_thresh:
            left_data_x.append(lidar_data[i][1] * np.cos(angle_increment * lidar_data[i][0]))
            left_data_y.append(lidar_data[i][1] * np.sin(angle_increment * lidar_data[i][0]))

        ######

    # left big => lat -
    # right big => lat +
    
    # left
    if len(left_data_x) != 0 and len(right_data_x) == 0:
        left_x = left_data_x[-1]
        lateral_err = left_x / 2
        left_coefficients = np.polyfit(left_data_x,left_data_y, 2)

        left_heading_slope = 2 * left_coefficients[2] * left_x + left_coefficients[1]

        left_heading = math.degrees(math.atan(left_heading_slope))

        if left_heading < 0:
            heading_err = -math.pi/2 - left_heading
        else:
            heading_err = math.pi/2 - left_heading

    # right
    elif len(left_data_x) == 0 and len(right_data_x) != 0:
        right_x = right_data_x[0]
        lateral_err = right_x / 2

        right_coefficients = np.polyfit(right_data_x, right_data_y, 2)

        right_heading_slope = 2 * right_coefficients[2] * right_x + right_coefficients[1]

        right_heading = math.degrees(math.atan(right_heading_slope))

        if right_heading < 0:
            heading_err = -math.pi/2 - right_heading
        else:
            heading_err = math.pi/2 - right_heading
        

    # both
    elif len(left_data_x) != 0 and len(right_data_x) != 0:
        left_x = left_data_x[-1]
        right_x = right_data_x[0]

        mid = (left_x + right_x) / 2
        lateral_err = mid - left_x

        left_coefficients = np.polyfit(left_data_x,left_data_y, 2)
        right_coefficients = np.polyfit(right_data_x, right_data_y, 2)

    
        left_heading_slope = 2 * left_coefficients[2] * left_x + left_coefficients[1]
        right_heading_slope = 2 * right_coefficients[2] * right_x + right_coefficients[1]

        left_heading = math.degrees(math.atan(left_heading_slope)) # -pi/2 부터 pi/2까지.
        right_heading = math.degrees(math.atan(right_heading_slope)) # -pi/2부터 pi/2까지.

        if left_heading < 0:
            left_heading += 180
        
        if right_heading < 0:
            right_heading += 180

        heading_err = math.pi / 2 - (left_heading + right_heading)/2

    else:
        print("망했어요")

        # left big => lat -
        # right big => lat +

        if prev_lateral_err < 0:
            heading_err = -20
            lateral_err = 0
        else:
            heading_err = 20
            lateral_err = 0


    delta = heading_err+ math.degrees(math.atan(k*100*lateral_err/(Fix_Speed/10)))

    delta = normalize_steering(delta)

    return delta, lateral_err, heading_err

def main():
    global motor
    rospy.init_node('sensor_node', anonymous=True)
    motor = rospy.Publisher('xycar_motor',xycar_motor, queue_size=1)
    lidar_rubber = Lidar_rubber()
    rate = rospy.Rate(10)  # 10 Hz

    # 이전 lateral_err와 heading_err 초기값 설정
    prev_lateral_err = 0
    prev_heading_err = 0

    while not rospy.is_shutdown():
        # rospy.loginfo("전방 라이다 정보: {}".format(lidar_rubber.data))
        # rospy.loginfo("전방 라이다 정보: {}".format(len(lidar_rubber.data)))
        if lidar_rubber.lidar_points:
            filtered_lidar = lidar_rubber.filtering()

            delta, prev_lateral_err, prev_heading_err = tracking(filtered_lidar, prev_lateral_err, prev_heading_err)
            drive(delta, Fix_Speed)
            
            rospy.loginfo("조향각: {}".format(delta))
            
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
	    pass