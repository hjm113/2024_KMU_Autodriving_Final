#! /usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from xycar_msgs.msg import xycar_motor
from cv_bridge import CvBridge
# import time

class ARDetectWithAbsDiff:
    def __init__(self):
        rospy.init_node('ar_detect_with_abdiff', anonymous=False)
        ##
        self.count = 0
        self.wait_count = 0 # 가림막 본 뒤, 몇번뒤에 변화를 detect 할건지
        ##
        self.bridge = CvBridge()
        
        # Initialize variables
        self.prev_image = None
        self.current_image = None

        self.change_num = 0 # 몇번 바뀌었는지, 홀수일때 motor on
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_callback, queue_size=1)
        
        # Publishers
        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.speed = 0

        # variables
        self.threshold = 3000 # 픽셀 threshold
        self.go_to_ar = 10 # ar 보기까지 직진할 횟수

    def camera_callback(self, data):
        # Convert ROS Image message to OpenCV image
        self.current_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # Get the ROI (vertical strip in the center of the image)
        height, width = self.current_image.shape[:2]
        roi_width = width // 4  # Width of the ROI (adjust as needed)
        x_center = width // 2
        x_start = x_center - roi_width // 2
        x_end = x_center + roi_width // 2

        # Extract the ROI from the current image
        self.current_roi = self.current_image[int(height*0.334):, x_start:x_end]
        cv2.imshow('ROI', self.current_roi)
        cv2.waitKey(1)   

        
        self.block()
    
    def block(self):
        pub_msg = xycar_motor()      
        if self.prev_image is not None:
            # Compute absolute difference between current and previous ROI
            diff = cv2.absdiff(self.prev_image, self.current_roi)
            gray_diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
            _, thresh_diff = cv2.threshold(gray_diff, 50, 255, cv2.THRESH_BINARY)
            non_zero_count = np.count_nonzero(thresh_diff)
            print(non_zero_count)
            
            # If difference is significant, detect it as mask installation

            if self.change_num < 5:
                if self.change_num == 0:
                    if non_zero_count > self.threshold:
                        self.speed = 5
                        self.change_num += 1
                        print("222222")
                    else:
                        print("1111111
                        self.speed = 0")
                elif self.change_num == 1:
                    if non_zero_count > self.threshold:
                        self.speed = 0
                        self.change_num += 1
                        print("33333")
                    else:
                        self.speed = 5
                        print("222222")
                elif self.change_num == 2:
                    if non_zero_count > self.threshold:
                        self.speed = 5
                        self.count += 1
                        self.change_num += 1
                        print("444444")
                    else:
                        self.speed = 0
                        print("33333")
                else:
                    if self.count < 10:
                        self.count += 1
                        print("444444")
                    else:
                        self.speed = 0
                        self.change_num+=1
                        print("done")

        pub_msg.speed = self.speed
        pub_msg.angle = 0  
        self.motor_pub.publish(pub_msg)
                
        
        # Update the previous ROI for the next iteration
        self.prev_image = self.current_roi.copy()

    # def move_forward(self, speed):
    #     # Publish a command to move forward at a constant speed
    #     pub_msg = xycar_motor()
    #     pub_msg.speed = speed
    #     pub_msg.angle = 0
    #     start_time = time.time()
        
    #     for i in range(5):
    #         self.motor_pub.publish(pub_msg)
    #         rospy.sleep(0.1)
        
    #     # Stop the car after moving forward
    #     if self.mask_detected:
    #         pub_msg.speed = 0
    #         self.motor_pub.publish(pub_msg)
        
if __name__ == "__main__":
    try:
        ARDetectWithAbsDiff()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
