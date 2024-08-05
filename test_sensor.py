#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import cv2
import sensor_syj as sen  # sensor_syj 모듈을 import
import numpy as np

def main():
    rospy.init_node('test_sensor_node', anonymous=True)  # ROS 노드 초기화
    rate = rospy.Rate(10)  # 10 Hz

    # sensor_syj 모듈에서 Sensor 클래스 인스턴스 생성
    sensor_instance = sen.Sensor()

    # 초기 yaw 설정
    initial_yaw = sensor_instance.setting(rate)
    rospy.loginfo("초기 Yaw: {}".format(initial_yaw))

    while not rospy.is_shutdown():
        # 카메라 이미지 데이터 접근
        if sensor_instance.image is not None:
            # 이미지 데이터 처리 (필요에 따라 수정)
            cv2.imshow("카메라 뷰", sensor_instance.image)
            cv2.waitKey(1)

        # 초음파 데이터 접근
        if sensor_instance.ultra_msg is not None:
            # 초음파 데이터 처리 (필요에 따라 수정)
            rospy.loginfo("초음파 데이터: {}".format(sensor_instance.ultra_msg))

        # LiDAR 데이터 접근
        if sensor_instance.lidar_ranges is not None:
            # LiDAR 데이터 처리 (필요에 따라 수정)
            rospy.loginfo("LiDAR 범위 데이터: {}".format(sensor_instance.lidar_ranges))

        # IMU 데이터 접근
        if sensor_instance.yaw is not None:
            # IMU 데이터 처리 (필요에 따라 수정)
            rospy.loginfo("Yaw: {:.2f}".format(sensor_instance.yaw))

        # AR 태그 데이터 접근
        if sensor_instance.ar_id is not None:
            # AR 태그 데이터 처리 (필요에 따라 수정)
            rospy.loginfo("AR ID: {}, 위치: ({}, {}), Yaw: {:.2f}".format(
                sensor_instance.ar_id, sensor_instance.ar_x, sensor_instance.ar_y, sensor_instance.ar_yaw
            ))

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

