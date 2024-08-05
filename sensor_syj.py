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
        self.lidar_range = None
        self.angle_increments = None
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)

        # imu
        self.yaw = None
        rospy.Subscriber('imu', Imu, self.imu_callback, queue_size=1)

        # ultrasonic
        self.ultra = None
        rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, self.ultrasonic_callback, queue_size=1)

        # AR
        self.ar_id, self.ar_data, self.ar_x, self.ar_y, self.ar_yaw = None, None, None, None, None
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback, queue_size=1 )

    # 카메라 콜백 함수 
    def usbcam_callback(self, data):
        self.image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")

    # 라이다 콜백 함수
    def lidar_callback(self, data):
        self.lidar_ranges = data.ranges # 각 각도에서의 거리 값
        self.angle_increments = data.angle_increment # 각 스캔 포인트 사이 각도 간격

        #roll, pitch, yaw = euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        roll, pitch, yaw = euler_from_quaternion((0,0,0,0))
        self.yaw = yaw%(2*np.pi)

    # imu 콜백 함수
    def imu_callback(self, data):
        roll, pitch, yaw = euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        self.yaw = yaw % (2*np.pi)

    # 초음파 콜백 함수
    def ultrasonic_callback(self, data):
        self.ultra_msg = data.data

    # AR 콜백 함수
    def ar_callback(self,data):
        self.ar_data = data
        
        id, x, y, yaw = [],[],[],[]

        # 발견된 모든 AR태그에 대해서 정보 수집
        for i in data.markers:
            id["ID"].append(i.id) # AR태그의 ID값을 리스트에 추가
            x["DX"].append(int(i.pose.pose.position.x*100)) # X값을 cm로 바꿔서 리스트에 추가
            y["DY"].append(int(i.pose.pose.position.z*100)) # Z값을 cm로 바꿔서 리스트에 추가
            yaw["YAW"].append(euler_from_quaternion((i.pose.pose.orientation.x, i.pose.pose.orientation.y,
                                              i.pose.pose.orientation.z, i.pose.pose.orientation.w))[1])
        target_idx = 0 # id 딕셔너리 중 어떤 인덱스를 사용할지. 아니면 list의 median 써도 될듯
        
        self.ar_id = id["ID"][target_idx]
        self.ar_x = x["DX"][target_idx]
        self.ar_y = y["DY"][target_idx]
        self.ar_yaw = yaw["YAW"][target_idx]

    def setting(self, rate):
        '''
        wait for initial callbacks from all sensors
        set initial yaw
        '''
        # ready lidar
        while self.lidar_ranges is None:
            rate.sleep()
        print("lidar ready")

        # ready imu
        while self.yaw is None:
            rate.sleep()
        print("imu ready")

        while self.ultra is None:
            rate.sleep()
        print("ultrasonic ready")

        while self.ar_data is None:
            rate.sleep()
        print("AR check ready")

        # set initial yaw => 의도를 모르겠음 (오차 보정용인가)
        yaws = []
        for _ in range(11):
            yaws.append(self.yaw)
            rate.sleep()
        yaw_0 = np.median(yaws)
        print("initial yaw = {}".format(yaw_0))
        return yaw_0
    
def main():
    rospy.init_node('sensor_node')
    sensor = Sensor()  # Create an instance of the Sensor class
    rate = rospy.Rate(10)  # 10 Hz

    initial_yaw = sensor.setting(rate)

    while not rospy.is_shutdown():
        # Access sensor data
        if sensor.image is not None:
            # Process the image data as needed
            cv2.imshow("Camera View", sensor.image)
            cv2.waitKey(1)

        if sensor.ultra_msg is not None:
            # Process the ultrasonic data as needed
            rospy.loginfo("Ultrasonic Data:",sensor.ultra_msg)

        rate.sleep()
    
if __name__ == "main":
    try:
        main()
    except rospy.ROSInterruptException:
        pass