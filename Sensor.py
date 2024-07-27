#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Ultrasonic
from ar_track_alvar_msgs.msg import AlvarMarkers
from detection_msgs.msg import BoundingBox, BoundingBoxes
from tf.transformations import euler_from_quaternion

class Sensor():

    def __init__(self):
        # 카메라
        self.image = None
        self.bridge = CvBridge()
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.usbcam_callback, queue_size=1)

        # 라이다
        self.lidar_range = None
        self.angle_increments = None
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)

        # imu
        self.yaw = None
        rospy.Subscriber('imu', Imu, self.imu_callback, queue_size=1)

        # 초음파
        self.ultra = None
        rospy.Subscriber("ultrasonic", Ultrasonic, self.ultrasonic_callback, queue_size=1)

        # ar 태그
        self.ar_id, self.ar_msg, self.ar_x, self.ar_y, self.ar_yaw = None, None, None, None, None
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback, queue_size = 1)

        # yolo node
        self.detect = {"grandeur":0, "avante":0, "sonata":0}
        self.x_mid = {"grandeur":0.0, "avante":0.0, "sonata":0.0}
        self.y = {"grandeur":0.0, "avante":0.0, "sonata":0.0}
        rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.yolo_callback)

    # 카메라 콜백 함수
    def usbcam_callback(self, data):
        self.image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")

    # 라이다 콜백 함수
    def lidar_callback(self, data):
        self.lidar_range = data.ranges # 각 각도에서의 거리 값
        self.angle_increments = data.angle_increment # 각 스캔 포인트 사이 각도 간격
        _, _, yaw = euler_from_quaternion((0, 0, 0, 0)) # (w, x, y, z)
        self.yaw = yaw % (2*np.pi)
    
    # imu 콜백 함수
    def imu_callback(self, data):
        _, _, yaw = euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        self.yaw = yaw % (2*np.pi)

    # 초음파 콜백 함수
    def ultrasonic_callback(self, data) :
        self.ultra = data.data

    # ar 콜백 함수
    def ar_callback(self, data):
        self.ar_msg = data
        for i in data.markers:     
            pose = i.pose.pose
            self.ar_x = pose.position.x
            self.ar_y = pose.position.z
            self.ar_yaw = euler_from_quaternion((pose.orientation.x, pose.orientation.y,
                                              pose.orientation.z, pose.orientation.w))[1]
            self.ar_id = i.id
    
    # yolo 콜백 함수
    def yolo_callback(self, data):
        self.detect = {"grandeur":0, "avante":0, "sonata":0}
        self.x_mid = {"grandeur":0.0, "avante":0.0, "sonata":0.0}
        self.y = {"grandeur":0.0, "avante":0.0, "sonata":0.0}
        for bbox in msg.bounding_boxes:
            self.detect[bbox.Class] = 1
            self.x_mid[bbox.Class] = (bbox.xmin + bbox.xmax) / 2
            self.y[bbox.Class] = abs(bbox.ymax - bbox.ymin)

    def init(self, rate):
        # ready lidar
        while self.lidar is None:
            rate.sleep()
        print("lidar ready")

        # set initial yaw
        yaws = []
        for _ in range(11):
            yaws.append(self.yaw)
            rate.sleep()
        yaw0 = np.median(yaws)
        print("initial yaw = {}".format(yaw0))
        return yaw0
