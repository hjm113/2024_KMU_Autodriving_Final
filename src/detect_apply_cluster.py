#! /usr/bin/env python3
import rospy
import cv2, os
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from math import *
from track_drive.msg import LaneInfo, reset
from collections import deque

from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

from time import time

red, green, blue = (0, 0, 255), (0, 255, 0), (255, 0, 0)

class lane_detect():
    def __init__(self):
        # 모드 측정해서 초기화할거
        self.current_mode = None
        self.previous_mode = None



        #warp, img size(w,h)
        self.warp_img_w, self.warp_img_h = 600, 300
        # canny params
        self.canny_low, self.canny_high = 20, 35
        # HoughLineP params
        self.hough_threshold, self.min_length, self.min_gap = 10, 6, 10
        # filtering params:
        self.angle_tolerance = np.radians(35) # 이전 차선각도와의 차이허용값 둘다 줄이는쪽으로 예상중   ############# 0828 0255 40 -> 35
        self.cluster_threshold = 30    # 차선 후보 군집화 시 허용 픽셀 거리
        self.lane_width = 255 #260 -> 255 카메라 각도 이전      
        self.mark_threshold = 10
        self.angle_constant = 0.85 #0.75


        self.warp_img_mid = int((self.warp_img_h/2)-20) #-20 good     0 00 dddd        speed 5 k 
        self.angle = 0.0
        self.prev_angle = deque([0.0], maxlen=5)
        self.lane = np.array([20,300,580])

        # custom message
        self.left_x = 0
        self.right_x = 0
        self.left_theta = 0.0
        self.right_theta = 0.0
        self.image = None
        self.bridge = CvBridge()
        rospy.init_node('lane_detection_node', anonymous=False)
        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.camera_callback, queue_size=1)
        rospy.Subscriber('/mode', reset, self.mode_callback, queue_size=1)  

        # publish
        self.pub = rospy.Publisher("lane_information", LaneInfo, queue_size=1)


        # self.yaw = None
        # rospy.Subscriber('/imu', Imu, self.imu_callback)
    # def imu_callback(self, data):
    #     roll, pitch, yaw = euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    #     self.yaw = -(yaw*180/np.pi - 37.117)
    #     print("yaw", self.yaw)
    def mode_callback(self, mode_data):
        """
        모드 값이 변경될 때 호출되는 콜백 함수.
        """
        print("Mode :", mode_data.mode)
        self.current_mode = mode_data.mode
        if self.previous_mode is None:
            self.previous_mode = self.current_mode

        if self.current_mode != self.previous_mode:
            self.detect_reset()  # 모드가 변경되었을 때 초기화 수행
            rospy.loginfo(f"모드가 변경되었습니다: {self.previous_mode} -> {self.current_mode}")
            self.previous_mode = self.current_mode

    def detect_reset(self):
        """
        모드 변경 시 상태 변수를 초기화합니다.
        """
        self.angle = 0.0
        self.prev_angle = deque([0.0], maxlen=5)
        self.lane = np.array([20, 300, 580])
        rospy.loginfo("상태가 초기화되었습니다.")

    def cam_exposure(self, value):
        # Auto exposure를 수동 모드로 설정
        command = 'v4l2-ctl -d /dev/videoCAM -c auto_exposure=1'  # 1: Manual Mode
        os.system(command)
        # Exposure time 설정
        command = 'v4l2-ctl -d /dev/videoCAM -c exposure_time_absolute=' + str(value)
        os.system(command)


    def camera_callback(self, data):
       self.image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
    #    cv2.imshow("Original", self.image)
       self.main()
       self.pub.publish(self.main())

    def warpping(self, image, show=False):
        # 좌상 좌하 우상 우하
        source = np.float32([[150, 290], [0,365], [525, 290], [640, 365]]) 
        destination = np.float32([[0, 0], [0, self.warp_img_h], [self.warp_img_w, 0] , [self.warp_img_w, self.warp_img_h]])         
        transform_matrix = cv2.getPerspectiveTransform(source, destination)
        minv = cv2.getPerspectiveTransform(destination, source)
        _image = cv2.warpPerspective(image, transform_matrix, (self.warp_img_w, self.warp_img_h))
        # if show:
        #     cv2.imshow("warpped_img", _image)
        return _image, minv

    def hough(self, img, show=False):
        # HoughLinesP(image, rho, theta, threshold, minLineLength, maxLineGap)
        lines = cv2.HoughLinesP(img, 2, np.pi/180, self.hough_threshold, self.min_gap, self.min_length)
        if show:
            hough_img = np.zeros((img.shape[0], img.shape[1], 3))
            if lines is not None:
                for x1, y1, x2, y2 in lines[:, 0]:
                    cv2.line(hough_img, (x1, y1), (x2, y2), red, 2)
            # cv2.imshow('hough', hough_img)
        return lines

    def filter(self, lines, show=True):
        thetas, positions = [], []
        if show:
            filter_img = np.zeros((self.warp_img_h, self.warp_img_w, 3))

        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                if y1 == y2:
                    continue
                flag = 1 if y1-y2 > 0 else -1
                theta = np.arctan2(flag * (x2-x1), 0.9*flag * (y1-y2))  #키우면 작은 변화에 둔감: 작은 기울기 변화나 노이즈에 덜 민감해집니다.더 강한 선 검출: 환경 노이즈나 작은 변화에 영향을 덜 받으므로, 더 강한 선분만 검출됩니다.
                if abs(theta - self.angle) < self.angle_tolerance: # modified value of self.angle_tolerance from 30 to 20 
                    position = float((x2-x1)*(self.warp_img_mid-y1))/(y2-y1) + x1
                    thetas.append(theta)
                    positions.append(position) 
                    if show:
                        cv2.line(filter_img, (x1, y1), (x2, y2), red, 2)
        self.prev_angle.append(self.angle)
        if thetas:
            self.angle = np.mean(thetas)
        if show:
            cv2.imshow('각도 필터 이미지', filter_img)
            cv2.moveWindow("각도 필터 이미지", 0, 480)

        return positions

    def get_cluster(self, positions):
        clusters = []
        for position in positions:
            if 0 <= position < 360:
                for cluster in clusters:
                    if abs(np.median(cluster) - position) < self.cluster_threshold:
                        cluster.append(position)
                        break
                else:
                    clusters.append([position])
        lane_candidates = [np.mean(cluster) for cluster in clusters]
        return lane_candidates


    def predict_lane(self):
        predicted_lane = self.lane[1] + [-self.lane_width/max(np.cos(self.angle), self.angle_constant), 0, self.lane_width/max(np.cos(self.angle), self.angle_constant)]
        predicted_lane = predicted_lane + (self.angle - np.mean(self.prev_angle))*70
        return predicted_lane

    def update_lane(self, lane_candidates, predicted_lane):
        if not lane_candidates:
            self.lane = predicted_lane
            return
        possibles = []
        for lc in lane_candidates:
            idx = np.argmin(abs(self.lane-lc))

            if idx == 0:
                estimated_lane = [lc, lc + self.lane_width/max(np.cos(self.angle), self.angle_constant), lc + (self.lane_width*2)/max(np.cos(self.angle), self.angle_constant)]
                lc2_candidate, lc3_candidate = [], []
                for lc2 in lane_candidates:
                    if abs(lc2-estimated_lane[1]) < self.mark_threshold :
                        lc2_candidate.append(lc2)
                for lc3 in lane_candidates:
                    if abs(lc3-estimated_lane[2]) < self.mark_threshold :
                        lc3_candidate.append(lc3)
                if not lc2_candidate:
                    lc2_candidate.append(estimated_lane[1])
                if not lc3_candidate:
                    lc3_candidate.append(estimated_lane[2])
                for lc2 in lc2_candidate:
                    for lc3 in lc3_candidate:
                        possibles.append([lc, lc2, lc3])

            elif idx == 1:
                estimated_lane = [lc - self.lane_width/max(np.cos(self.angle), self.angle_constant), lc, lc + self.lane_width/max(np.cos(self.angle), self.angle_constant)]
                lc1_candidate, lc3_candidate = [], []
                for lc1 in lane_candidates:
                    if abs(lc1-estimated_lane[0]) < self.mark_threshold :
                        lc1_candidate.append(lc1)
                for lc3 in lane_candidates:
                    if abs(lc3-estimated_lane[2]) < self.mark_threshold :
                        lc3_candidate.append(lc3)
                if not lc1_candidate:
                    lc1_candidate.append(estimated_lane[0])
                if not lc3_candidate:
                    lc3_candidate.append(estimated_lane[2])
                for lc1 in lc1_candidate:
                    for lc3 in lc3_candidate:
                        possibles.append([lc1, lc, lc3])

            else :
                estimated_lane = [lc - (self.lane_width*2)/max(np.cos(self.angle), self.angle_constant), lc - self.lane_width/max(np.cos(self.angle), self.angle_constant), lc]
                lc1_candidate, lc2_candidate = [], []
                for lc1 in lane_candidates:
                    if abs(lc1-estimated_lane[0]) < self.mark_threshold :
                        lc1_candidate.append(lc1)
                for lc2 in lane_candidates:
                    if abs(lc2-estimated_lane[1]) < self.mark_threshold :
                        lc2_candidate.append(lc2)
                if not lc1_candidate:
                    lc1_candidate.append(estimated_lane[0])
                if not lc2_candidate:
                    lc2_candidate.append(estimated_lane[1])
                for lc1 in lc1_candidate:
                    for lc2 in lc2_candidate:
                        possibles.append([lc1, lc2, lc])

        possibles = np.array(possibles)
        error = np.sum((possibles-predicted_lane)**2, axis=1)
        best = possibles[np.argmin(error)]
        self.lane = 0.5 * best + 0.5* predicted_lane
        self.mid = np.mean(self.lane)

    def mark_lane(self, img, lane=None):
        if lane is None:
            lane = self.lane
            self.mid = self.lane[1]
        l1, l2, l3 = self.lane
        cv2.circle(img, (int(l1), self.warp_img_mid), 3, red, 5, cv2.FILLED)
        cv2.circle(img, (int(l2), self.warp_img_mid), 3, green, 5, cv2.FILLED)
        cv2.circle(img, (int(l3), self.warp_img_mid), 3, blue, 5, cv2.FILLED)
        cv2.imshow('marked', img)
        cv2.moveWindow("marked", 0, 800)


        cv2.waitKey(1)

    def adjust_lightness(self, image, beta=0):
        # BGR 이미지를 HLS 색 공간으로 변환
        hls_img = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        h, l, s = cv2.split(hls_img)
        # 밝기 채널 조정
        l = cv2.add(l, beta)
        l = np.clip(l, 0, 255)
        # 조정된 채널을 다시 합쳐 HLS 이미지 생성
        adjusted_hls = cv2.merge((h, l, s))
        # HLS 이미지를 다시 BGR 색 공간으로 변환
        adjusted_img = cv2.cvtColor(adjusted_hls, cv2.COLOR_HLS2BGR)

        return adjusted_img
       
    def detect_reset(self):
        """
        reset_switch가 켜질 때 상태 변수를 초기화합니다.
        """
        self.angle = 0.0
        self.prev_angle = deque([0.0], maxlen=5)
        self.lane = np.array([20, 300, 580])
        # 초기화해야 할 다른 변수들도 리셋
        rospy.loginfo("카메라 스위치로 인해 상태가 초기화되었습니다.")

    def main(self):
        # 주행 로직
        # self.cam_exposure(80)
        pub_msg = LaneInfo()
        warpped_img, minverse = self.warpping(self.image, show=True)
        blurred_img = cv2.GaussianBlur(warpped_img, (0,0), 3)
        canny_img = cv2.Canny(blurred_img, self.canny_low, self.canny_high) 
        lines = self.hough(canny_img, show=True)
        positions = self.filter(lines, show=True)
        # cv2.imshow("blur",blurred_img)
        # cv2.imshow('canny', canny_img)
        lane_candidates = self.get_cluster(positions)
        predicted_lane = self.predict_lane()
        self.update_lane(lane_candidates, predicted_lane)
        self.mark_lane(warpped_img)
        pub_msg.left = np.float32(self.lane[0])
        pub_msg.mid = np.float32(self.lane[1])
        pub_msg.right = np.float32(self.lane[2])
        pub_msg.angle = np.float32((self.angle)*0.667)
        return pub_msg
    
if __name__ == "__main__":
    if not rospy.is_shutdown():
        lane_detect()
        rospy.spin()