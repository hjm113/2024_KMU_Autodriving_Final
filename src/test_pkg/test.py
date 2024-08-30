#! /usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from math import *
from track_drive.msg import LaneInfo
from collections import deque
from test_pkg.sensor import Sensor

from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class Test():
    def __init__(self):
        self.sensor = Sensor()
        self.wait = self.sensor.setting(rospy.Rate(10))
        print("DEBUGGGGGGGGGGGGGGGGGGGGGG")
        self.data = self.sensor.lidar
        
    def print_lidar_len(self):
        print(len(self.data.ranges))

if __name__ == "__main__":
    rospy.init_node('sensor_get_node', anonymous=True)
    rate = rospy.Rate(10)
    test = Test()
    while not rospy.is_shutdown():
        test.print_lidar_len()
        rate.sleep()