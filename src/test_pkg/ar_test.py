#! /usr/bin/env python3  
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def image_callback(msg):
    try:
        # ROS Image 메시지를 OpenCV 이미지로 변환
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # OpenCV를 사용해 이미지를 화면에 표시
        cv2.imshow("Rectified Image", cv_image)

        # 'q' 키를 누르면 창이 닫히도록 함
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User exited the program.")

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    # ROS 노드 초기화
    rospy.init_node('image_viewer', anonymous=True)

    # /image_rect_color 토픽 구독
    rospy.Subscriber("/image_rect_color", Image, image_callback)

    # ROS 루프 실행
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    # 모든 창 닫기
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
