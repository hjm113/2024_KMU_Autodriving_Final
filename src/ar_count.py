#! /usr/bin/env python3  
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospy, os, time, math
from tf.transformations import euler_from_quaternion
from track_drive.msg import mode_info
import numpy as np
from sensor_msgs.msg import Image, Imu
import cv2
from cv_bridge import CvBridge 

Blue =  (255,0,0) # 파란색
Green = (0,255,0) # 녹색
Red =   (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색

def normalize_angle(angle, max_steer = 50):
    if angle > max_steer:
        angle = max_steer
    elif angle < -max_steer:
        angle = -max_steer

    return angle


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.prev_time = None

    def calculate(self, error):

        current_time = rospy.get_time()  # 현재 시간 가져오기
        if self.prev_time is None:
            dt = 0  # 처음에는 dt를 0으로 설정
        else:
            dt = current_time - self.prev_time

        # 적분 계산
        self.integral += error * dt
        # 미분 계산
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        # PID 제어 값 계산
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        # 이전 오차 저장
        self.prev_error = error
        return output

class AR_Detect():
    def __init__(self):
        self.ar_msg = {"ID": [], "DX": [], "DZ": [],"AX": [], "AY": [], "AZ": [],"AW": []}
        self.roll, self.pitch, self.yaw = 0, 0, 0
        rospy.init_node('ar_node', anonymous=False)
        self.sub_ar = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback, queue_size=1)

        # Imu 추가
        self.sub_imu = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)

        self.pub_ar = rospy.Publisher('ar', mode_info, queue_size=1)
        

        ###################################################################
        # PID 제어기 초기화
        self.pid_controller = PIDController(Kp=1.0, Ki=0.01, Kd=0.1)
        ###################################################################
        
        self.traffic = False # False
        # 몇번째 ar을 보고있는지
        self.count = 0

        self.count_speed = [3, 3, 3, 3, 3, 3]

        ##############################
        # 더 늘리면 빨리 꺾음
        # 더 줄이면 AR에 더 가까워진 뒤 꺾음
        # self.dist_thresh = 80 # 100
        self.dist_thresh = [80, 70, 88, 30, 110, 100] # idx 6 : redundant value 85 decent
        self.turn = [-30, -30, -50, -10, -100, -100] # idx 6 : redundant value # -60
        self.target_angle = [-50, -50, -45, -50, -50, -30]
      
        self.new_angle = 0
        self.control_angle = 0
        self.yaw_imu = 0
        self.control_speed = 3

        self.distance_queue = [0]*6

        ## camera
        self.image = None
        self.bridge = CvBridge()
        rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_callback, queue_size=1)

        # 신호등 하드코딩
        self.range = [False] * 10
        self.rate = rospy.Rate(10)

        self.flag = 0
        self.flag2 = 0

    def imu_callback(self, data):
        # IMU에서 Yaw 값을 가져오기 (라디안)
        orientation_q = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        )
        roll, pitch, yaw = euler_from_quaternion(orientation_q)
        self.yaw_imu = yaw  # 라디안 값으로 저장
        self.yaw_imu = yaw % (2 * np.pi)  # Yaw 값을 [0, 2π] 범위로 변환

    def camera_callback(self, data):
       self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
       cv2.imshow("Original", self.image)
       cv2.waitKey(1)
    
    # distance값 저장
    # count +1 하는 경우 : 작,작,작, 큰,큰,큰 일때 only
    def update_distance(self, distance):
        self.distance_queue.pop(0)
        self.distance_queue.append(distance)
    
    def should_update_count(self):

        # 급격히 증가한 이후의 첫 번째 값과 비교하는 방식으로 변경
        increased_value = self.distance_queue[3]  # 증가한값
        previous_value = self.distance_queue[2]   # 첫번째값
        similar_range_small = False
        similar_range_big = False
        similar_range = False

        # 급격한 증가가 확인되면, 그 이후의 3개의 값이 증가한 값과 비슷한지 확인
        if (increased_value - previous_value) > 70: 
            similar_range_small = all(abs(previous_value - self.distance_queue[i])<40 for i in range(0,2))
            similar_range_big = all(abs(increased_value - self.distance_queue[i]) < 70 for i in range(4, 6))
        similar_range = similar_range_big and similar_range_small
        ###
        print(similar_range)
        ###
        return similar_range
    

    def ar_callback(self, msg):
        self.ar_msg["ID"] = []
        self.ar_msg["DX"] = []
        self.ar_msg["DZ"] = []
        self.ar_msg["AX"] = []
        self.ar_msg["AY"] = []
        self.ar_msg["AZ"] = []
        self.ar_msg["AW"] = []


        for i in msg.markers:
            self.ar_msg["ID"].append(i.id)
            self.ar_msg["DX"].append(int(i.pose.pose.position.x * 100))  # x값 cm로 바꿔서 리스트에 추가
            self.ar_msg["DZ"].append(int(i.pose.pose.position.z * 100))
            self.ar_msg["AX"].append(i.pose.pose.orientation.x)
            self.ar_msg["AY"].append(i.pose.pose.orientation.y)
            self.ar_msg["AZ"].append(i.pose.pose.orientation.z)
            self.ar_msg["AW"].append(i.pose.pose.orientation.w)

        pub_msg = mode_info()
        ar_ID, z_pos, x_pos, yaw_value = self.check_AR()

        if self.traffic == False:
            ## 신호등 하드코딩 ##
            self.control_speed = 5

            if self.range[0] == False:
                for i in range(25): # 20 30 25 25
                    pub_msg.speed = int(self.control_speed)
                    pub_msg.angle = int(-30)
                    pub_msg.mode = int(1)
                    self.pub_ar.publish(pub_msg)
                    self.rate.sleep()
                    print("-15")
                self.range[0] = True

            if self.range[0] == True and self.range[1] == False:
                for i in range(15): # 20 18 15 13
                    pub_msg.speed = int(self.control_speed)
                    pub_msg.angle = int(15)
                    pub_msg.mode = int(1)
                    self.pub_ar.publish(pub_msg)
                    self.rate.sleep()
                    print("15")
                self.range[1] = True

            # if self.range[1] == True and self.range[2] == False:
            #     for _ in range(3):
            #         pub_msg.speed, pub_msg.angle = int(self.control_speed), int(10)
            #         self.pub_ar.publish(pub_msg)
            #         self.rate.sleep()
            #         print("10")
            #     self.range[2] = True

            if self.check_traffic_sign():
                self.traffic = True
                for _ in range(6):
                    pub_msg.speed, pub_msg.angle = int(5), int(0) # angle : 0
                    pub_msg.mode = int(1)
                    print("hard")
                    self.pub_ar.publish(pub_msg)
                    self.rate.sleep()

        else:
            if ar_ID != 99:
                distance = math.sqrt(z_pos**2+x_pos**2)
                self.update_distance(distance)
                self.control_speed = 3
                """
                # 이전거 -> 중앙값 아니라 하나만 비교
                if (abs(self.distance_prev - distance) > 100 and abs(self.distance_prev - distance)<320) :
                    self.count += 1
                """
                if self.should_update_count():
                    self.count+=1
                
                ################################################
                # 22보다 큼: 인코스 더 잘돎
                # 22보다 작음: 아웃코스 더 잘돎
                # angle_usage = np.arctan2(x_pos-30, z_pos) # -22
                angle_usage = np.arctan2(x_pos+self.turn[self.count], z_pos)

                yaw_error = angle_usage
                self.control_angle = self.pid_controller.calculate(yaw_error)
                print(f"count {self.count}")
                print(f"distance", distance)

                target_angle = math.degrees(self.control_angle)

                target_angle = normalize_angle(target_angle)

                # if distance <= self.dist_thresh:
                if distance <= self.dist_thresh[self.count]:
                    target_angle = self.target_angle[self.count] # -50
                pub_msg.speed = int(self.control_speed)
                pub_msg.angle = int(target_angle)
                pub_msg.mode = int(1)
                self.pub_ar.publish(pub_msg)  # 메시지 발행
            else:
                if self.count == 2 and self.distance_queue[-1]<=self.dist_thresh[2] and self.flag == 0:
                    self.flag = 1
                    print("not detected")
                    for _ in range(8): # 3
                        print("-38")
                        pub_msg.speed = int(self.control_speed)
                        pub_msg.angle = int(-38) #25
                        pub_msg.mode = int(1)
                        self.pub_ar.publish(pub_msg)
                        self.rate.sleep()
                    for _ in range(4): # 3
                        print("50")
                        pub_msg.speed = int(self.control_speed)
                        pub_msg.angle = int(50)
                        pub_msg.mode = int(1)
                        self.pub_ar.publish(pub_msg)
                        self.rate.sleep()
                if self.count == 5 and self.flag2 == 0:
                    for _ in range (3):
                        pub_msg.speed = int(3)
                        pub_msg.angle = int(30)
                        pub_msg.mode = int(1)
                        self.pub_ar.publish(pub_msg)
                        self.rate.sleep()
                        print("ddone")
                    for _ in range(11):
                        pub_msg.speed, pub_msg.angle = int(3), int(0)
                        pub_msg.mode = int(1)
                        self.pub_ar.publish(pub_msg)
                        self.rate.sleep()
                    self.flag2 = 1

                    rospy.sleep(3.5)
                    pub_msg.speed, pub_msg.angle = int(0), int(0)
                    pub_msg.mode = int(0)
                    self.pub_ar.publish(pub_msg)


  
        cv2.waitKey(1)

        
                

    def check_AR(self):
        id_value = 99
        yaw_value = 0

        if len(self.ar_msg["ID"]) == 0:
            # 발견된 AR 태그가 없으면 ID값 99, z위치 x위치 500cm로 return
            return 99, 200, 0, 0

        z_pos = 300
        x_pos = 0
        # 발견된 AR 태그 모두에 대해서 조사
        for i in range(len(self.ar_msg["ID"])):
            distance = math.sqrt(self.ar_msg["DZ"][i]**2 + self.ar_msg["DX"][i]**2)
            # print("1거리 : ", distance)

            if (self.ar_msg["DZ"][i] < z_pos) and (distance > 50) and (distance < 280):
                # print("2필터된 거리 : ",distance)
                # 더 가까운 거리에 AR 태그가 있으면 그걸 사용
                id_value = self.ar_msg["ID"][i]
                z_pos = self.ar_msg["DZ"][i]
                x_pos = self.ar_msg["DX"][i]
                # Quaternion을 euler angles로 변환하여 roll, pitch, yaw 값 계산
                orientation_q = (
                    self.ar_msg["AX"][i],
                    self.ar_msg["AY"][i],
                    self.ar_msg["AZ"][i],
                    self.ar_msg["AW"][i]
                )
                (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_q)
                self.roll = math.degrees(self.roll)
                self.pitch = math.degrees(self.pitch)
                self.yaw = math.degrees(self.yaw)
                yaw_value = self.pitch

        # id 번호, 거리값(미터), 좌우치우침값(미터), yaw 값 리턴
        return id_value, round(z_pos, 2), (round(x_pos, 2)-16), yaw_value

        
    def check_traffic_sign(self):

        MIN_RADIUS, MAX_RADIUS = 15, 25
        
        # 원본이미지를 복제한 후에 특정영역(ROI Area)을 잘라내기
        cimg = self.image.copy()
        # Center_X, Center_Y = 320, 100  # ROI 영역의 중심위치 좌표 
        # XX, YY = 220, 80  # 위 중심 좌표에서 좌우로 XX, 상하로 YY만큼씩 벌려서 ROI 영역을 잘라냄   

        Center_X, Center_Y = 300, 120  # ROI 영역의 중심위치 좌표 # 360, 120
        XX, YY = 180, 60  # 위 중심 좌표에서 좌우로 XX, 상하로 YY만큼씩 벌려서 ROI 영역을 잘라냄  
        # ROI 영역에 녹색 사각형으로 테두리를 쳐서 표시함 
        cv2.rectangle(cimg, (Center_X-XX, Center_Y-YY), (Center_X+XX, Center_Y+YY) , Green, 2)
        
        # 원본 이미지에서 ROI 영역만큼 잘라서 roi_img에 담음 
        roi_img = cimg[Center_Y-YY:Center_Y+YY, Center_X-XX:Center_X+XX]

        # roi_img 칼라 이미지를 회색 이미지로 바꾸고 노이즈 제거를 위해 블러링 처리를 함  
        img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(img, (5, 5), 0)

        # Hough Circle 함수를 이용해서 이미지에서 원을 (여러개) 찾음 
        circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, 20,
                    param1=40, param2=20, 
                    minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS)

        cv2.waitKey(1)

        if circles is not None:
        
            # 정수값으로 바꾸고 발견된 원의 개수를 출력
            circles = np.round(circles[0, :]).astype("int")
            # print("\nFound",len(circles),"circles")
            
            # 중심의 Y좌표값 순서대로 소팅해서 따로 저장
            y_circles = sorted(circles, key=lambda circle: circle[1])
    
            # 중심의 X좌표값 순서대로 소팅해서 circles에 다시 저장
            circles = sorted(circles, key=lambda circle: circle[0])
            
            # 발견된 원들에 대해서 루프를 돌면서 하나씩 녹색으로 그리기 
            for i, (x, y, r) in enumerate(circles):
                cv2.circle(cimg, (x+Center_X-XX, y+Center_Y-YY), r, Green, 2)
    
        # 이미지에서 정확하게 3개의 원이 발견됐다면 신호등 찾는 작업을 진행  
        if (circles is not None) and (len(circles)==3):
                
            # 가장 밝은 원을 찾을 때 사용할 변수 선언
            max_mean_value = 0
            max_mean_value_circle = None
            max_mean_value_index = None

            # 발견된 원들에 대해서 루프를 돌면서 하나씩 처리 
            # 원의 중심좌표, 반지름. 내부밝기 정보를 구해서 화면에 출력
            for i, (x, y, r) in enumerate(circles):
                roi = img[y-(r//2):y+(r//2),x-(r//2):x+(r//2)]
                # 밝기 값은 반올림해서 10의 자리수로 만들어 사용
                mean_value = round(np.mean(roi),-1)
                # print(f"Circle {i} at ({x},{y}), radius={r}: brightness={mean_value}")
                
                # 이번 원의 밝기가 기존 max원보다 밝으면 이번 원을 max원으로 지정  
                if mean_value > max_mean_value:
                    max_mean_value = mean_value
                    max_mean_value_circle = (x, y, r)
                    max_mean_value_index = i
                    
                # 원의 밝기를 계산했던 사각형 영역을 빨간색으로 그리기 
                cv2.rectangle(cimg, ((x-(r//2))+Center_X-XX, (y-(r//2))+Center_Y-YY),
                    ((x+(r//2))+Center_X-XX, (y+(r//2))+Center_Y-YY), Red, 2)

            # 가장 밝은 원을 찾았으면 그 원의 정보를 출력하고 노란색으로 그리기 
            if max_mean_value_circle is not None:
                (x, y, r) = max_mean_value_circle
                # print(f" --- Circle {max_mean_value_index} is the brightest.")
                cv2.circle(cimg, (x+Center_X-XX, y+Center_Y-YY), r, Yellow, 2)
                
            # 신호등 찾기 결과가 표시된 이미지를 화면에 출력
            cv2.imshow('Circles Detected', cimg)
            
            # 제일 위와 제일 아래에 있는 2개 원의 Y좌표값 차이가 크면 안됨 
            vertical_diff = MAX_RADIUS * 2
            if (y_circles[-1][1] - y_circles[0][1]) > vertical_diff:
                # print("Circles are scattered vertically!")
                return False
            
            # 제일 왼쪽과 제일 오른쪽에 있는 2개 원의 X좌표값 차이가 크면 안됨 
            horizontal_diff = MAX_RADIUS * 8
            if (circles[-1][0] - circles[0][0]) > horizontal_diff:
                # print("Circles are scattered horizontally!")
                return False      
                
            # 원들이 좌우로 너무 붙어 있으면 안됨 
            min_distance = MIN_RADIUS * 3
            for i in range(len(circles) - 1):
                if (circles[i+1][0] - circles[i][0]) < min_distance:
                    # print("Circles are too close horizontally!")
                    return False 
                
            # 3개 중에서 세번째 원이 가장 밝으면 (파란색 신호등) True 리턴 
            if (max_mean_value_index == 2):
                # print("Traffic Sign is Blue...!")
                return True
            
            # 첫번째나 두번째 원이 가장 밝으면 (파란색 신호등이 아니면) False 반환 
            else:
                # print("Traffic Sign is NOT Blue...!")
                return False

        # 신호등 찾기 결과가 표시된 이미지를 화면에 출력
        cv2.imshow('Circles Detected', cimg)
        
        # 원본 이미지에서 원이 발견되지 않았다면 False 리턴   
        #print("Can't find Traffic Sign...!")
        return False
    

if __name__ == "__main__":
    if not rospy.is_shutdown():
        AR_Detect()
        rospy.spin()