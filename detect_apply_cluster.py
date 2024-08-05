#!/usr/bin/env python
# -*- coding: utf-8 -*- 1

import numpy as np
import cv2, rospy, time, math, os
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from ar_track_alvar_msgs.msg import AlvarMarkers


# lane_msgs 디렉토리 안의 msg 디렉토리 안에 있는 LaneInfo.msg 파일.
from lane_msgs.msg import LaneInfo

import importlib.util

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================


bridge = CvBridge()

image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수

WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
Blue =  (255,0,0) # 파란색
Green = (0,255,0) # 녹색
Red =   (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색

View_Center = WIDTH//2  # 화면의 중앙값 = 카메라 위치


#=============================================
# 카메라의 Exposure 값을 변경하는 함수 
# 입력으로 0~255 값을 받는다.
#=============================================
def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)

#=============================================
# 콜백함수 - USB 카메라 토픽을 받아서 처리하는 콜백함수
#=============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")



class Clustering():

    def __init__(self):
        # warpping 상수들. 이 값으로 했을 때 bird's eye view느낌 잘 났음

        #notion 값들
        #self.warp_img_h, self.warp_img_w, self.warp_img_mid = 460, 250, 230

        #self.source = np.float32([[235, 250], [330, 250], [80, 475], [460, 475]])
        #self.destination = np.float32([[0, 0], [250, 0], [0, 460], [250, 460]])
        
        """
        -------------Case 1
        self.warp_img_h, self.warp_img_w, self.warp_img_mid = 460, 250, 125

        # 사다리꼴 좌표를 아래로 이동하고 넓게 조정
        self.source = np.float32([[100, 300], [540, 300], [0, 480], [640, 480]])
        self.destination = np.float32([[0, 0], [250, 0], [0, 460], [250, 460]])"""
   
        """#Case 2
        self.warp_img_h, self.warp_img_w, self.warp_img_mid = 460, 250, 125

        self.source = np.float32([[150, 290], [490, 290], [0, 460], [640, 460]])
        self.destination = np.float32([[0, 0], [250, 0], [0, 460], [250, 460]])"""

        # 사다리꼴 좌표를 위로 이동하고 넓게 조정
        self.warp_img_h, self.warp_img_w, self.warp_img_mid = 460, 250, 125

        # 사다리꼴 좌표를 수직으로 내려다보는 뷰로 조정
        self.source = np.float32([[200, 290], [440, 290], [0, 460], [640, 460]])
        self.destination = np.float32([[0, 0], [250, 0], [0, 460], [250, 460]])


        # canny edge 임계값. 정확한 수치 확인하기
        self.canny_low = 100
        self.canny_high = 120

        # Hough Transform 상수들. 정확한 수치 확인하기 ## 3,5,5
        self.hough_threshold = 3
        self.min_gap = 5
        self.min_length = 5

        self.angle = 0.0
        self.prev_angle = deque([0.0], maxlen=5)

        self.lane = np.array([20.0, 125., 230.])

        # filtering params:
        self.angle_tolerance = np.radians(30)
        self.cluster_threshold = 25

    def warpping(self,image):
        """
            차선을 BEV로 변환하는 함수
            
            Return
            1) _image : BEV result image
            2) minv : inverse matrix of BEV conversion matrix
        """

        transform_matrix = cv2.getPerspectiveTransform(self.source, self.destination)
        minv = cv2.getPerspectiveTransform(self.destination, self.source)
        _image = cv2.warpPerspective(image, transform_matrix, (250, 460))

        return _image, minv

    # Canny Edge Detection
    def to_canny(self, img, show=False):
        img = cv2.GaussianBlur(img, (7,7), 0)
        img = cv2.Canny(img, self.canny_low, self.canny_high)
        if show:
            cv2.imshow('canny', img)
        return img

    # 허프 변환으로 차선 검출
    def hough(self, img, show=False):
        lines = cv2.HoughLinesP(img, 1, np.pi/180, self.hough_threshold, self.min_gap, self.min_length)
        if show:
            hough_img = np.zeros((img.shape[0], img.shape[1], 3))
            if lines is not None:
                for x1, y1, x2, y2 in lines[:, 0]:
                    cv2.line(hough_img, (x1, y1), (x2, y2), Red, 2)
            cv2.imshow('hough', hough_img)
        return lines
    

    # 차선 후보 filtering
    def filter(self, lines, show=False):
        '''
        filter lines that are close to previous angle and calculate its positions
        '''
        thetas, positions = [], []
        if show:
            filter_img = np.zeros((self.warp_img_h, self.warp_img_w, 3))

        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                if y1 == y2:
                    continue
                flag = 1 if y1-y2 > 0 else -1
                theta = np.arctan2(flag * (x2-x1), flag * 0.9* (y1-y2))
                if abs(theta - self.angle) < self.angle_tolerance:
                    position = float((x2-x1)*(self.warp_img_mid-y1))/(y2-y1) + x1
                    thetas.append(theta)
                    positions.append(position) 
                    if show:
                        cv2.line(filter_img, (x1, y1), (x2, y2), Red, 2)

        self.prev_angle.append(self.angle)
        if thetas:
            self.angle = np.mean(thetas)
        if show:
            cv2.imshow('filtered lines', filter_img)
        return filter_img, positions
    
    # 거리 기반 클러스터링
    def get_cluster(self, positions):
        '''
        group positions that are close to each other
        '''
        clusters = []
        for position in positions:
            if 0 <= position < 250:
                for cluster in clusters:
                    if abs(cluster[0] - position) < self.cluster_threshold:
                        cluster.append(position)
                        break
                else:
                    clusters.append([position])
        lane_candidates = [np.mean(cluster) for cluster in clusters]
        return lane_candidates
    
    def predict_lane(self):
        '''
        predicts lane positions from previous lane positions and angles
        '''
        predicted_lane = self.lane[1] + [-105/max(np.cos(self.angle), 0.75), 0, 105/max(np.cos(self.angle), 0.75)]
        predicted_lane = predicted_lane + (self.angle - np.mean(self.prev_angle))*70
        return predicted_lane


    def update_lane(self, lane_candidates, predicted_lane):
        '''
        calculate lane position using best fitted lane and predicted lane
        '''
        
        if not lane_candidates:
            self.lane = predicted_lane
            return
        
        possibles = []
        
        for lc in lane_candidates:
        
            idx = np.argmin(abs(self.lane-lc))
        
            if idx == 0:
                estimated_lane = [lc, lc + 105/max(np.cos(self.angle), 0.75), lc + 210/max(np.cos(self.angle), 0.75)]
                lc2_candidate, lc3_candidate = [], []
                for lc2 in lane_candidates:
                    if abs(lc2-estimated_lane[1]) < 50 :
                        lc2_candidate.append(lc2)
                for lc3 in lane_candidates:
                    if abs(lc3-estimated_lane[2]) < 50 :
                        lc3_candidate.append(lc3)
                if not lc2_candidate:
                    lc2_candidate.append(estimated_lane[1])
                if not lc3_candidate:
                    lc3_candidate.append(estimated_lane[2])
                for lc2 in lc2_candidate:
                    for lc3 in lc3_candidate:
                        possibles.append([lc, lc2, lc3])
        
            elif idx == 1:
                estimated_lane = [lc - 105/max(np.cos(self.angle), 0.75), lc, lc + 105/max(np.cos(self.angle), 0.75)]
                lc1_candidate, lc3_candidate = [], []
                for lc1 in lane_candidates:
                    if abs(lc1-estimated_lane[0]) < 50 :
                        lc1_candidate.append(lc1)
                for lc3 in lane_candidates:
                    if abs(lc3-estimated_lane[2]) < 50 :
                        lc3_candidate.append(lc3)
                if not lc1_candidate:
                    lc1_candidate.append(estimated_lane[0])
                if not lc3_candidate:
                    lc3_candidate.append(estimated_lane[2])
                for lc1 in lc1_candidate:
                    for lc3 in lc3_candidate:
                        possibles.append([lc1, lc, lc3])
        
            else :
                estimated_lane = [lc - 210/max(np.cos(self.angle), 0.75), lc - 105/max(np.cos(self.angle), 0.75), lc]
                lc1_candidate, lc2_candidate = [], []
                for lc1 in lane_candidates:
                    if abs(lc1-estimated_lane[0]) < 50 :
                        lc1_candidate.append(lc1)
                for lc2 in lane_candidates:
                    if abs(lc2-estimated_lane[1]) < 50 :
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
        self.lane = 0.5 * best + 0.5 * predicted_lane
        self.mid = np.mean(self.lane)

        # print("leftttttttttttt", left)
        # print("middddddddddddd", mid)
        # print("righttttttttttt", right)


    def mark_lane(self, img, lane=None):
        '''
        mark calculated lane position to an image 
        '''
        # img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if lane is None:
            lane = self.lane
            self.mid = self.lane[1]
        l1, l2, l3 = self.lane

        # 원의 이미지 상 높이 현재 230으로 설정돼 있음. 필요 시 수정
        cv2.circle(img, (int(l1), 230), 3, Red, 5, cv2.FILLED)
        cv2.circle(img, (int(l2), 230), 3, Green, 5, cv2.FILLED)
        cv2.circle(img, (int(l3), 230), 3, Blue, 5, cv2.FILLED)
        cv2.imshow('marked', img)


def detect():
    pub = rospy.Publisher("lane_information", LaneInfo, queue_size=10)
    rospy.init_node('detect_node', anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1) #카메라
    """
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback, queue_size=1) #초음파
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback, queue_size=1 ) #AR
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1) #Lidar
    """

    #=========================================
    # 발행자 노드들로부터 첫번째 토픽들이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")

    """
    rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
    print("UltraSonic Ready ----------")
    rospy.wait_for_message("/scan", LaserScan)
    print("Lidar Ready ----------")
    rospy.wait_for_message("ar_pose_marker", AlvarMarkers)
    print("AR detector Ready ----------")
    """

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        cluster_lane = Clustering()

        #Bird's eye view 적용
        warpped_img , minv = cluster_lane.warpping(image)
                
        # Canny_Edge 적용
        canny_img = cluster_lane.to_canny(warpped_img,True)

        # Hough Transform 적용
        hough_lines = cluster_lane.hough(canny_img,True)

        # Filtering 적용
        filtered_img, filtered_pos = cluster_lane.filter(hough_lines, True)

        # Clustering 적용
        lane_candidates = cluster_lane.get_cluster(filtered_pos)

        # 현재 차선 위치 예측
        predicted_lane = cluster_lane.predict_lane()

        # 최종 차선 update
        cluster_lane.update_lane(lane_candidates,predicted_lane)

        # 차선 표시
        cluster_lane.mark_lane(warpped_img)

        # 메시지 내용 담기
        lane = LaneInfo()
        lane.left = cluster_lane.lane[0]
        lane.mid = cluster_lane.mid
        lane.right = cluster_lane.lane[2]
        lane.angle = cluster_lane.angle

        # 디버깅용
        rospy.loginfo("------------")
        rospy.loginfo("left lane: %d", lane.left)
        rospy.loginfo("right lane: %d", lane.right)
        rospy.loginfo("middle lane: %d", lane.mid)
        rospy.loginfo("lane angle: %f", lane.angle)

        # 메시지 Publish
        pub.publish(lane)
        rospy.loginfo("**************")
        rospy.loginfo("Publishing topic, topic name: lane_information")
        rate.sleep()

if __name__ == '__main__':
    try:
        detect()
    except rospy.ROSInterruptException:
        pass
