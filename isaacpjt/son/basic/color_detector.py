#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector', anonymous=True)
        
        # Subscriber 설정
        self.rgb_sub = rospy.Subscriber('/rgb', Image, self.rgb_callback)
        self.color_id_sub = rospy.Subscriber('/color_id', Int32, self.color_id_callback)
        
        # Publisher 설정
        self.result_pub = rospy.Publisher('/color_detector/result', Image, queue_size=10)
        
        self.bridge = CvBridge()
        self.current_color_id = None
        self.current_image = None
    
    def rgb_callback(self, msg):
        # RGB 이미지 수신
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def color_id_callback(self, msg):
        # color_id 수신
        self.current_color_id = msg.data
        
        if self.current_image is not None:
            # 색상 검출 처리
            result = self.detect_color(self.current_image, self.current_color_id)
            
            # 결과 송신
            result_msg = self.bridge.cv2_to_imgmsg(result, "bgr8")
            self.result_pub.publish(result_msg)
    
    def detect_color(self, image, color_id):
        # color_id에 따라 다른 색상 검출
        # color_id=1 → 파란색 감지
        # color_id=2 → 초록색 감지
        
        # 예시: HSV 색상 범위 정의
        color_ranges = {
            1: [(100, 50, 50), (130, 255, 255)],  # 파란색
            2: [(40, 50, 50), (80, 255, 255)]     # 초록색
        }
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower, upper = color_ranges.get(color_id, [(0, 0, 0), (180, 255, 255)])
        
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        result = cv2.bitwise_and(image, image, mask=mask)
        
        return result

if __name__ == '__main__':
    detector = ColorDetector()
    rospy.spin()