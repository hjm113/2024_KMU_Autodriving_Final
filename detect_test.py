import os
import rosbag
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sensor_msgs.msg
from collections import deque

# colors
red, green, blue = (0, 0, 255), (0, 255, 0), (255, 0, 0)

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

        #Case 2
        self.warp_img_h, self.warp_img_w, self.warp_img_mid = 460, 250, 125

        self.source = np.float32([[150, 290], [490, 290], [0, 460], [640, 460]])
        self.destination = np.float32([[0, 0], [250, 0], [0, 460], [250, 460]])

        
        # canny edge 임계값. 정확한 수치 확인하기
        self.canny_low = 100
        self.canny_high = 120

        # Hough Transform 상수들. 정확한 수치 확인하기
        self.hough_threshold = 10
        self.min_gap = 30
        self.min_length = 10

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
                    cv2.line(hough_img, (x1, y1), (x2, y2), red, 2)
            #cv2.imshow('hough', hough_img)
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
                        cv2.line(filter_img, (x1, y1), (x2, y2), red, 2)

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

# bagfile save할 함수
def process_and_save_bag(input_bag_file, image_topic, output_bag_file):
    try:
        input_bag = rosbag.Bag(input_bag_file, 'r')
        output_bag = rosbag.Bag(output_bag_file, 'w')
    except FileNotFoundError as e:
        print(f"File not found: {e}")
        return

    bridge = CvBridge()

    print(f"Reading messages from bag file: {input_bag_file}")
    for topic, msg, t in input_bag.read_messages():
        if topic == image_topic:
            try:
                # Convert ROS Compressed Image message to OpenCV image
                cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

                cluster = Clustering()

                warpped_img , minv = cluster.warpping(cv_image)
                
                # Canny_Edge 적용
                canny_img = cluster.to_canny(warpped_img)

                # Hough Transform 적용
                hough_lines = cluster.hough(canny_img,True)

                # Filtering 적용
                filtered_img, filtered_pos = cluster.filter(hough_lines, True)

                # Clustering 적용
                #lane_candidates = cluster.get_cluster(filtered_pos)

                # 현재 차선 위치 예측
                #predicted_lane = cluster.predicted_lane()

                # 최종 차선 update



                # Convert the modified OpenCV image back to ROS Image message
                modified_msg = bridge.cv2_to_compressed_imgmsg(filtered_img)

                # Use original message's header
                modified_msg.header = msg.header


                # Write the modified message to the output bag file
                output_bag.write(image_topic, modified_msg, t)
            except CvBridgeError as e:
                print(f"Error processing image: {e}")
        else:
            # For non-image topics, write the original message
            output_bag.write(topic, msg, t)

    input_bag.close()
    output_bag.close()
    print(f"Finished processing and saved to {output_bag_file}")

if __name__ == "__main__":
    # 현재 작업 디렉토리와 스크립트 디렉토리 확인
    print("Current working directory:", os.getcwd())
    script_dir = os.path.dirname(os.path.abspath(__file__))
    print("Script directory:", script_dir)
    
    # 상대 경로를 절대 경로로 변환
    bag_file = os.path.join(script_dir, '../0723_솦콤/0723_솦콤_자동2(커버o).bag')
    print(f"Bag file path: {bag_file}")
    image_topic = '/usb_cam/image_raw/compressed'  # 실제 사용하는 이미지 토픽으로 변경
    output_bag_file = 'output_0723_jadong2_filtered_test_case2.bag'  # 새 bag 파일 경로

    process_and_save_bag(bag_file, image_topic, output_bag_file)



