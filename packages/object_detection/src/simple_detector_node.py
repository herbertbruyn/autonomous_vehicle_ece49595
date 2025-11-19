#!/usr/bin/env python3
"""
Simple Color-Based Object Detection Node
Fast alternative to YOLO for detecting colored objects
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String


class SimpleDetectorNode:
    def __init__(self):
        # Color parameters (HSV)
        self.lower_yellow = np.array(rospy.get_param('~lower_yellow', [20, 100, 100]))
        self.upper_yellow = np.array(rospy.get_param('~upper_yellow', [30, 255, 255]))
        self.lower_orange = np.array(rospy.get_param('~lower_orange', [5, 150, 150]))
        self.upper_orange = np.array(rospy.get_param('~upper_orange', [15, 255, 255]))
        
        self.min_area = rospy.get_param('~min_area', 3000)
        self.danger_zone_height = rospy.get_param('~danger_zone_height', 0.6)
        self.enable_yellow = rospy.get_param('~enable_yellow', True)
        self.enable_orange = rospy.get_param('~enable_orange', True)
        
        # Publishers
        self.pub_obstacle = rospy.Publisher('~obstacle_alert', Bool, queue_size=1)
        self.pub_classes = rospy.Publisher('~detection_classes', String, queue_size=1)
        self.pub_debug_image = rospy.Publisher('~detection_image/compressed', 
                                               CompressedImage, queue_size=1)
        
        # Subscribers
        self.sub_image = rospy.Subscriber('~camera/image/compressed', CompressedImage,
                                         self.callback_image, queue_size=1, buff_size=2**24)
        
        rospy.loginfo("Simple Color Detector initialized")
    
    def callback_image(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                return
            
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            masks = []
            if self.enable_yellow:
                mask_yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
                masks.append(('yellow', mask_yellow))
            if self.enable_orange:
                mask_orange = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
                masks.append(('orange', mask_orange))
            
            kernel = np.ones((5, 5), np.uint8)
            
            img_height, img_width = image.shape[:2]
            danger_zone_y = int(img_height * (1 - self.danger_zone_height))
            
            obstacle_detected = False
            detected_classes = []
            
            for color_name, mask in masks:
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                              cv2.CHAIN_APPROX_SIMPLE)
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    
                    if area > self.min_area:
                        x, y, w, h = cv2.boundingRect(contour)
                        
                        if y + h > danger_zone_y:
                            obstacle_detected = True
                            detected_classes.append(color_name)
                            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 3)
                        else:
                            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            cv2.line(image, (0, danger_zone_y), (img_width, danger_zone_y), 
                    (255, 255, 0), 2)
            
            status_text = "OBSTACLE!" if obstacle_detected else "CLEAR"
            status_color = (0, 0, 255) if obstacle_detected else (0, 255, 0)
            cv2.putText(image, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 3)
            
            alert_msg = Bool()
            alert_msg.data = obstacle_detected
            self.pub_obstacle.publish(alert_msg)
            
            if detected_classes:
                classes_msg = String()
                classes_msg.data = ', '.join(set(detected_classes))
                self.pub_classes.publish(classes_msg)
            
            if self.pub_debug_image.get_num_connections() > 0:
                _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = buffer.tobytes()
                self.pub_debug_image.publish(debug_msg)
        
        except Exception as e:
            rospy.logerr(f"Error: {e}")


if __name__ == "__main__":
    rospy.init_node('simple_detector_node', anonymous=False)
    node = SimpleDetectorNode()
    rospy.spin()
