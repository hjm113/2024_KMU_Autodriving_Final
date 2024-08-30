#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import rospy, time, math
from track_drive.msg import mode_info
from sensor_msgs.msg import LaserScan

motor = None  # 모터 노드 변수


class Avoidance:

    def __init__(self):
        self.speed = 10
        self.detect_dist = 0.7
        self.steering_angle = 0.0
        self.angle_increment = 0.01745329238474369
        self.MIN_DISTANCE_THRESHOLD = 0.8
        self.CAR_WIDTH = 0.3
        self.lidar_points = []
        self.state = 0
        self.kp = 1.0
        self.mode_msg = mode_info()

        """
        0: 직진
        1: 조향
        2: 복귀
        """
        self.mission = None
        """
        1: 왼쪽 장애물 => 오른쪽 조향
        0: 정면 장애물
        -1: 오른쪽 장애물 => 왼쪽 조향
        """
        self.mode_pub = rospy.Publisher('obstacle', mode_info, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        rospy.wait_for_message("/scan", LaserScan)
    
    def lidar_callback(self, data):
        self.lidar_points = list(data.ranges[:181])
        
        if self.state == 0:
            self.avoid_mode()
        elif self.state == 1:
            self.perform_first_turn()
        elif self.state == 2:
            self.perform_second_turn()
        elif self.state == 3:
            self.align_to_center()

        # 계산된 조향각을 이용해 주행 명령 실행
        #drive(self.steering_angle, self.speed)

    def avoid_mode(self):

        self.front_ranges = list(self.lidar_points[50:130])
        indices_in_range = [i+50 for i, distance in enumerate(self.lidar_points[50: 130]) if 0.2 <= distance <= 0.7]

        if len(indices_in_range) < 10:
            self.steering_angle = 0
            self.drive(self.steering_angle, self.speed, 1)
            print("not")
            return

        count_left = sum(1 for i in indices_in_range if i > 90)
        count_right = len(indices_in_range) - count_left
        print("LEFT", count_left)
        print("RIGHT", count_right)

        if count_left < count_right:
            self.state = 1
            self.mission = -1
            print("오른쪽 장애물 감지: 첫 번째 조향 시작, 조향각 =", self.steering_angle)
        else:
            print("왼쪽  장애물 감지: 첫 번째 조향 시작, 조향각 =", self.steering_angle)
            self.mission = 1
            self.state = 1

        self.state_start_time = rospy.get_time()
        # if not math.isinf(self.lidar_points[104]) and self.lidar_points[104] <= self.detect_dist:
        #     self.state = 1
        #     self.steering_angle = self.calculate_steering_angle()
        #     self.mission = 1
        #     self.state_start_time = rospy.get_time()
        #     print("왼쪽 장애물 감지: 첫 번째 조향 시작, 조향각 =", self.steering_angle)

        # elif not math.isinf(self.lidar_points[89]) and self.lidar_points[89] <= self.detect_dist:
        #     self.state = 1
        #     self.steering_angle = self.calculate_steering_angle()
        #     self.mission = 0
        #     self.state_start_time = rospy.get_time()
        #     print("중앙 장애물 감지: 첫 번째 조향 시작, 조향각 =", self.steering_angle)

        # elif not math.isinf(self.lidar_points[74]) and self.lidar_points[74] <= self.detect_dist:
        #     self.state = 1
        #     self.steering_angle = self.calculate_steering_angle()
        #     self.mission = -1
        #     self.state_start_time = rospy.get_time()
        #     print("오른쪽 장애물 감지: 첫 번째 조향 시작, 조향각 =", self.steering_angle)

        # else:
        #     # 장애물이 없을 경우 직진
        #     self.steering_angle = 0
        #     self.drive(self.steering_angle, self.speed)
        #     print("장애물 없음: 직진 중")

    def perform_first_turn(self):
        # 첫 번째 조향 동작을 수행하다가 조건을 만족하면 두 번째 조향으로 전환
        # min_value = min([[index, value] for index, value in enumerate(self.lidar_points) if not math.isinf(value)], key=lambda x: x[1])
        # theta_rad = min_value[0] * self.angle_increment
        
        # # 조건: self.CAR_WIDTH <= abs(min_value[1] * np.cos(theta_rad))
        # if self.CAR_WIDTH/2 > abs(min_value[1] * np.cos(theta_rad)):
        #     self.drive(self.steering_angle, self.speed)
        if rospy.get_time() - self.state_start_time < 0.45:
            if self.mission == -1:
                self.steering_angle = -self.calculate_steering_angle() * self.kp
                self.drive(self.steering_angle, self.speed, 1)
            else:
                self.steering_angle = self.calculate_steering_angle() * self.kp
                self.drive(self.steering_angle, self.speed, 1)
        else:
            self.state = 2
            self.state_start_time = rospy.get_time()
            print("첫 번째 조향 완료: 두 번째 조향 시작")

    def perform_second_turn(self):
        # 반대 방향으로 일정 시간 동안 두 번째 조향 동작을 수행
        # 0.5초 동안 두 번째 조향
            if self.mission == 1:
                if rospy.get_time() - self.state_start_time < 1.6:
                    self.recovery = -30  # 반대 방향으로 조향
                    self.drive(self.recovery, self.speed, 1)
                else:
                    self.state = 3  # 두 번째 조향 완료 후, idle 상태로 복귀
                    self.state_start_time = rospy.get_time()
            else:
                if rospy.get_time() - self.state_start_time < 1.2:
                    self.recovery = 30  # 반대 방향으로 조향
                    self.drive(self.recovery, self.speed, 1)
                else:
                    self.state = 3  # 두 번째 조향 완료 후, idle 상태로 복귀
                    self.state_start_time = rospy.get_time()
            print("두 번째 조향 완료: 대기 상태로 복귀")

    def align_to_center(self):
        # if self.mission == 1:   # 오른쪽 조향. 장애물 왼쪽
        #     if self.lidar_points[180] < 0.2:
        #         if rospy.get_time() - self.state_start_time < 0.5:
        #             self.recovery = -30
        #             print("장애물 왼쪽. -30 조향")
        #     self.drive(self.recovery, self.speed, 1)
        # elif self.mission == -1:   # 왼쪽 조향. 장애물 왼쪽
        #     if self.lidar_points[0] < 0.2:
        #         if rospy.get_time() - self.state_start_time < 0.5:
        #             self.recovery = 30
        #             print("장애물 오른쪽. 30 조향")
        #     self.drive(self.recovery, self.speed, 1)
        self.state = 0
        self.drive(0, 10, 0)
        print("두 번째 조향 완료: 직진")
            
    def calculate_steering_angle(self):
        filtered_list = [[i+50, distance] for i, distance in enumerate(self.lidar_points[50: 130]) if 0.2 <= distance <= 0.7]
        if not filtered_list:
            rospy.logwarn("유효한 라이다 데이터가 없습니다. 조향각을 0으로 설정합니다.")
            return 0
        
        min_value = min(filtered_list, key=lambda x: x[1])
        min_idx = min_value[0]
        
        theta_rad = min_idx * self.angle_increment

        if theta_rad > math.pi / 2:
            theta_rad = math.pi - theta_rad

        ratio = (self.CAR_WIDTH + self.MIN_DISTANCE_THRESHOLD) / min_value[1]

        if ratio > 1:
            ratio = 1

        delta_rad = theta_rad - math.acos(ratio)
        delta_deg = math.degrees(delta_rad)

        delta_deg = normalize_steering(delta_deg)

        return delta_deg

#=============================================
# 모터 토픽을 발행하는 함수 
#=============================================
    def drive(self, angle, speed, mode):
        self.mode_msg.angle = int(angle)   # 소수점 이하 부분 날아간다는 우려점
        self.mode_msg.speed = int(speed)    # 소수점 이하 부분 날아간다는 우려점
        self.mode_msg.mode = int(mode)
        print("DEBUGG", angle)
        self.mode_pub.publish(self.mode_msg)

def normalize_steering(angle, max_steer=50):
    if angle >= max_steer:
        angle = max_steer
    elif angle <= -max_steer:
        angle = -max_steer
    return angle

def main():
    global motor
    rospy.init_node('lidar_obstacle_detection')
    Avoidance()  # Avoidance 클래스 인스턴스를 생성하고, 라이다 콜백 함수가 실행되도록 함
    rospy.spin()  # ROS 노드가 종료될 때까지 대기

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass