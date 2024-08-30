#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
#from sensor_msgs.msg import Ultrasonic
from ar_track_alvar_msgs.msg import AlvarMarkers

from tf.transformations import euler_from_quaternion

class Sensor():
    def __init__(self):
        # camera
        self.image = None
        self.bridge = CvBridge()
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.usbcam_callback, queue_size=1)

        # lidar
        self.lidar = None
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)

        # ultrasonic
        self.ultra = None
        rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, self.ultrasonic_callback, queue_size=1)

        # AR
        self.ar_id, self.ar_data, self.ar_x, self.ar_y, self.ar_yaw = None, None, None, None, None
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback, queue_size=1 )

    # 카메라 콜백 함수 
    def usbcam_callback(self, data):
        self.image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        # rospy.loginfo("camera clear")
        cv2.waitKey(1)

    # 라이다 콜백 함수
    def lidar_callback(self, data):
        self.lidar = data
        # rospy.loginfo("lidar clear")

    # 초음파 콜백 함수
    def ultrasonic_callback(self, data):
        self.ultra = data.data
        # rospy.loginfo("ultrasonic clear")


    # AR 콜백 함수
    def ar_callback(self,data):
        self.ar_data = data
        
        # id, x, y, yaw = [],[],[],[]

        # # 발견된 모든 AR태그에 대해서 정보 수집
        # for i in data.markers:
        #     id["ID"].append(i.id) # AR태그의 ID값을 리스트에 추가
        #     x["DX"].append(int(i.pose.pose.position.x*100)) # X값을 cm로 바꿔서 리스트에 추가
        #     y["DY"].append(int(i.pose.pose.position.z*100)) # Z값을 cm로 바꿔서 리스트에 추가
        #     yaw["YAW"].append(euler_from_quaternion((i.pose.pose.orientation.x, i.pose.pose.orientation.y,
        #                                       i.pose.pose.orientation.z, i.pose.pose.orientation.w))[1])
        # target_idx = 0 # id 딕셔너리 중 어떤 인덱스를 사용할지. 아니면 list의 median 써도 될듯
        
        # self.ar_id = id["ID"][target_idx]
        # self.ar_x = x["DX"][target_idx]
        # self.ar_y = y["DY"][target_idx]
        # self.ar_yaw = yaw["YAW"][target_idx]

        # rospy.loginfo("AR clear")

    def setting(self, rate):
        '''
        wait for initial callbacks from all sensors
        set initial yaw
        '''
        # ready lidar
        while self.lidar is None:
            rate.sleep()
        print("lidar ready")

        while self.ultra is None:
            rate.sleep()
        print("ultrasonic ready")

        while self.ar_data is None:
            rate.sleep()
        print("AR check ready")

        # set initial yaw => 의도를 모르겠음 (오차 보정용인가)
        time = []
        for i in range(11):
            time.append(i)
            rate.sleep()
        time_taken = np.median(time)
        print("initial yaw = {}".format(time_taken))
        return time_taken