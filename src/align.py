#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import numpy as np
import cv2, rospy, time, math, os
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor


def main():
    rospy.init_node('xycar_align')
    motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    while not rospy.is_shutdown():
        motor_msg = xycar_motor()
        motor_msg.speed = 0
        motor_msg.angle = 0
        motor_pub.publish(motor_msg)
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass