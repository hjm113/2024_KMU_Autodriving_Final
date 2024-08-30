#!/usr/bin/env python
# -*- coding: utf-8 -*- 1
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, rospy, time, math, os
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from ar_track_alvar_msgs.msg import AlvarMarkers
import importlib.util

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
# motor
motor = None  # 모터 노드 변수
Fix_Speed = 5  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
motor_msg = xycar_motor()  # 모터 토픽 메시지

# motor 토픽 발행
def drive(angle, speed):
    motor_msg.angle = int(angle)
    motor_msg.speed = int(speed)
    motor.publish(motor_msg)

def start():
    global motor

    LANE_DRIVE = 4
    drive_mode = LANE_DRIVE

    # initialize node, define subscribe/publish node
    rospy.init_node('Track_Driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    while not rospy.is_shutdown():
        while drive_mode == LANE_DRIVE:
            angle_= new_angle
            speed_ = Fix_Speed

            drive(angle_, speed_)

# 메인함수 호출
# start() 함수 = 실질적 main함수
if __name__ == '__main__':
    start()