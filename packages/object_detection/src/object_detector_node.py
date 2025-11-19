#!/usr/bin/env python3
"""
Object Detection Node for Collision Avoidance

Subscribes:
  ~camera/image/compressed (sensor_msgs/CompressedImage)

Publishes:
  ~obstacle_alert (std_msgs/Bool)
  ~detection_classes (std_msgs/String)
  ~detection_image/compressed (sensor_msgs/CompressedImage) [debug]
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
import torch


class ObjectDetectorNode:
    def __init__(self):
        # Parameters
        self.model_path = rospy.get_param('~model_path', 'yolov5s.pt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.45)
        self.target_classes = rospy.get_param('~target_classes', ['person', 'dog', 'cat', 'bird'])
        self.danger_zone_height = rospy.get_param('~danger_zone_height', 0.6)
        self.min_box_area = rospy.get_param('~min_box_area', 1000)
        self.process_every_n_frames = rospy.get_param('~process_every_n_frames', 1)
        self.image_scale = rospy.get_param('~image_scale', 1.0)
        
        # State
        self.frame_counter = 0
        self.last_obstacle_state = False
        
        # Load YOLO model
        rospy.loginfo("Loading YOLO model...")
        try:
            self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
            self.model.conf = self.confidence_threshold
            self.model.iou = self.iou_threshold
            rospy.loginfo("YOLO model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            rospy.signal_shutdown("Model loading failed")
            return
        
        # Publishers
        self.pub_obstacle_alert = rospy.Publisher('~obstacle_alert', Bool, queue_size=1)
        self.pub_detection_classes = rospy.Publisher('~detection_classes', String, queue_size=1)
        self.pub_debug_image = rospy.Publisher('~detection_image/compressed', 
                                               CompressedImage, queue_size=1)
        
        # Subscribers
        self.sub_image = rospy.Subscriber('~camera/image/compressed', CompressedImage,
                                         self.callback_image, queue_size=1, buff_size=2**24)
        
        rospy.loginfo("Object Detection Node initialized")
        rospy.loginfo(f"Target classes: {self.target_classes}")
    
    def callback_image(self, msg):
        """Process incoming camera images"""
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            alert_msg = Bool()
            alert_msg.data = self.last_obstacle_state
            self.pub_obstacle_alert.publish(alert_msg)
            return
        
        try:
            import time
            start_time = time.time()
            
            # Decode image
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                return
            
            # Optional scaling
            if self.image_scale != 1.0:
                new_width = int(image.shape[1] * self.image_scale)
                new_height = int(image.shape[0] * self.image_scale)
                image = cv2.resize(image, (new_width, new_height))
            
            # Run inference
            results = self.model(image)
            
            # Process detections
            obstacle_detected = False
            detected_classes = []
            
            img_height, img_width = image.shape[:2]
            danger_zone_y = int(img_height * (1 - self.danger_zone_height))
            
            detections = results.pandas().xyxy[0]
            
            for _, detection in detections.iterrows():
                class_name = detection['name']
                confidence = detection['confidence']
                
                if class_name not in self.target_classes:
                    continue
                
                x1, y1, x2, y2 = int(detection['xmin']), int(detection['ymin']), \
                                 int(detection['xmax']), int(detection['ymax'])
                
                box_area = (x2 - x1) * (y2 - y1)
                
                if box_area > self.min_box_area and y2 > danger_zone_y:
                    obstacle_detected = True
                    detected_classes.append(class_name)
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 3)
                    label = f"{class_name}: {confidence:.2f}"
                    cv2.putText(image, label, (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw danger zone
            cv2.line(image, (0, danger_zone_y), (img_width, danger_zone_y), (255, 255, 0), 2)
            
            status_text = "OBSTACLE!" if obstacle_detected else "CLEAR"
            status_color = (0, 0, 255) if obstacle_detected else (0, 255, 0)
            cv2.putText(image, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 3)
            
            # Publish
            alert_msg = Bool()
            alert_msg.data = obstacle_detected
            self.pub_obstacle_alert.publish(alert_msg)
            self.last_obstacle_state = obstacle_detected
            
            if detected_classes:
                classes_msg = String()
                classes_msg.data = ', '.join(set(detected_classes))
                self.pub_detection_classes.publish(classes_msg)
            
            # Debug image
            if self.pub_debug_image.get_num_connections() > 0:
                _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = buffer.tobytes()
                self.pub_debug_image.publish(debug_msg)
            
            processing_time = time.time() - start_time
            rospy.loginfo_throttle(5.0, f"Detection: {processing_time*1000:.1f}ms")
        
        except Exception as e:
            rospy.logerr(f"Error: {e}")


if __name__ == "__main__":
    rospy.init_node('object_detector_node', anonymous=False)
    node = ObjectDetectorNode()
    rospy.spin()
