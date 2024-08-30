#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import numpy as np
import cv2, rospy, time, math, os
from std_msgs.msg import Int32MultiArray
from track_drive.msg import mode_info


def main():
    rospy.init_node('xycar_start')
    mode_pub = rospy.Publisher('start', mode_info, queue_size=1)

    mode_msg = mode_info()
    mode_msg.speed = 5
    mode_msg.angle = 0
    mode_msg.mode = 1

    while not rospy.is_shutdown():
        for i in range(3):
            mode_pub.publish(mode_msg)
            print("DEBUGGGG")
            rospy.sleep(1)
        for i in range(3):
            mode_msg.speed = 0
            mode_msg.angle = 0
            mode_pub.publish(mode_msg)
            print("DEBUGGGG")
            rospy.sleep(1)
        mode_msg.mode = 0
        mode_pub.publish(mode_msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass