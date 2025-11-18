#!/usr/bin/env python3
"""
YOLO-based Obstacle Detection for Racing
Detects obstacles using YOLOv5 and estimates their position relative to the robot

Subscribes:
  /camera_node/image/compressed (sensor_msgs/CompressedImage)  # Camera feed
  /lane_filter_node/lane_pose (duckietown_msgs/LanePose)       # For context

Publishes:
  ~obstacle_detected (std_msgs/Bool)           # True if obstacle in path
  ~obstacle_position (geometry_msgs/Point)     # Obstacle position (x, y in robot frame)
  ~obstacle_bbox (std_msgs/Float32MultiArray)  # Bounding box [x1, y1, x2, y2, confidence, class]
  ~detection_image (sensor_msgs/CompressedImage)  # Annotated image for debugging
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import Point
from duckietown_msgs.msg import LanePose
import torch

class YOLOObstacleDetector:
    def __init__(self):
        # Parameters
        self.model_name = rospy.get_param('~model_name', 'yolov5n')  # nano model for speed
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)
        
        # Detection zones (normalized image coordinates)
        self.danger_zone_top = rospy.get_param('~danger_zone_top', 0.4)     # Top 40% of image
        self.danger_zone_bottom = rospy.get_param('~danger_zone_bottom', 1.0)  # Bottom of image
        self.danger_zone_left = rospy.get_param('~danger_zone_left', 0.2)   # Left 20%
        self.danger_zone_right = rospy.get_param('~danger_zone_right', 0.8)  # Right 80%
        
        # Camera parameters (for distance estimation)
        self.camera_height = rospy.get_param('~camera_height', 0.07)  # meters above ground
        self.camera_pitch = rospy.get_param('~camera_pitch', 0.0)     # radians (tilt angle)
        self.focal_length_px = rospy.get_param('~focal_length_px', 320)  # approximate
        
        # Object size assumptions (for distance estimation)
        self.assumed_object_height = rospy.get_param('~assumed_object_height', 0.08)  # meters (duckiebot height)
        
        # State
        self.bridge = CvBridge()
        self.lane_d = 0.0  # Lateral offset from lane center
        self.detection_count = 0
        self.last_detection_time = rospy.Time.now()
        
        # Load YOLO model
        rospy.loginfo("Loading YOLO model...")
        try:
            self.model = torch.hub.load('ultralytics/yolov5', self.model_name, pretrained=True)
            self.model.conf = self.confidence_threshold
            # Set to CPU or CUDA based on availability
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.model.to(self.device)
            rospy.loginfo(f"YOLO model loaded on {self.device}")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            rospy.logerr("Falling back to simple detection mode")
            self.model = None
        
        # Publishers
        self.pub_obstacle = rospy.Publisher('~obstacle_detected', Bool, queue_size=1)
        self.pub_position = rospy.Publisher('~obstacle_position', Point, queue_size=1)
        self.pub_bbox = rospy.Publisher('~obstacle_bbox', Float32MultiArray, queue_size=1)
        self.pub_debug_image = rospy.Publisher('~detection_image', CompressedImage, queue_size=1)
        
        # Subscribers
        self.sub_image = rospy.Subscriber('/camera_node/image/compressed', 
                                         CompressedImage, self.callback_image, queue_size=1, buff_size=2**24)
        self.sub_lane_pose = rospy.Subscriber('/lane_filter_node/lane_pose',
                                             LanePose, self.callback_lane_pose, queue_size=1)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("YOLO Obstacle Detector initialized")
        rospy.loginfo(f"  Model: {self.model_name}")
        rospy.loginfo(f"  Confidence threshold: {self.confidence_threshold}")
        rospy.loginfo(f"  Danger zone: [{self.danger_zone_left:.2f}, {self.danger_zone_right:.2f}] x "
                     f"[{self.danger_zone_top:.2f}, {self.danger_zone_bottom:.2f}]")
        rospy.loginfo("=" * 60)
    
    def callback_lane_pose(self, msg):
        """Update lane position for context"""
        self.lane_d = msg.d
    
    def estimate_distance(self, bbox_height_px):
        """
        Estimate distance to object based on its height in pixels
        Uses pinhole camera model: distance = (real_height * focal_length) / pixel_height
        """
        if bbox_height_px < 10:  # Too small to be reliable
            return float('inf')
        
        distance = (self.assumed_object_height * self.focal_length_px) / bbox_height_px
        return max(0.1, min(distance, 5.0))  # Clamp to reasonable range
    
    def estimate_lateral_offset(self, bbox_center_x, image_width):
        """
        Estimate lateral offset of object from robot's centerline
        Negative = left, Positive = right
        """
        # Normalize to [-1, 1]
        normalized_x = (bbox_center_x / image_width) - 0.5
        # Convert to approximate meters (rough estimate)
        lateral_offset = normalized_x * 0.5  # Assume ~0.5m width of view at 1m distance
        return lateral_offset
    
    def is_in_danger_zone(self, bbox, img_width, img_height):
        """Check if bounding box is in the danger zone (path ahead)"""
        x1, y1, x2, y2 = bbox[:4]
        
        # Normalize coordinates
        center_x = (x1 + x2) / 2.0 / img_width
        center_y = (y1 + y2) / 2.0 / img_height
        
        # Check if center is in danger zone
        in_x_range = self.danger_zone_left <= center_x <= self.danger_zone_right
        in_y_range = self.danger_zone_top <= center_y <= self.danger_zone_bottom
        
        return in_x_range and in_y_range
    
    def callback_image(self, msg):
        """Process camera image and detect obstacles"""
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img is None:
                rospy.logwarn_throttle(5.0, "Failed to decode image")
                return
            
            img_height, img_width = img.shape[:2]
            
            # Run YOLO detection
            if self.model is not None:
                results = self.model(img)
                detections = results.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2, conf, class]
            else:
                # Fallback: no detections
                detections = np.array([])
            
            # Process detections
            obstacle_detected = False
            closest_obstacle = None
            closest_distance = float('inf')
            
            # Draw danger zone on debug image
            debug_img = img.copy()
            zone_x1 = int(self.danger_zone_left * img_width)
            zone_x2 = int(self.danger_zone_right * img_width)
            zone_y1 = int(self.danger_zone_top * img_height)
            zone_y2 = int(self.danger_zone_bottom * img_height)
            cv2.rectangle(debug_img, (zone_x1, zone_y1), (zone_x2, zone_y2), (0, 255, 255), 2)
            
            for det in detections:
                x1, y1, x2, y2, conf, cls = det
                
                # Check if in danger zone
                if self.is_in_danger_zone(det, img_width, img_height):
                    obstacle_detected = True
                    
                    # Estimate distance and position
                    bbox_height = y2 - y1
                    bbox_center_x = (x1 + x2) / 2.0
                    
                    distance = self.estimate_distance(bbox_height)
                    lateral_offset = self.estimate_lateral_offset(bbox_center_x, img_width)
                    
                    # Track closest obstacle
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_obstacle = {
                            'bbox': det,
                            'distance': distance,
                            'lateral_offset': lateral_offset
                        }
                    
                    # Draw detection (red for danger)
                    cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                    cv2.putText(debug_img, f"{distance:.2f}m", 
                               (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                else:
                    # Draw detection (green for safe)
                    cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 1)
            
            # Publish results
            self.pub_obstacle.publish(Bool(data=obstacle_detected))
            
            if obstacle_detected and closest_obstacle is not None:
                # Publish obstacle position
                pos = Point()
                pos.x = closest_obstacle['distance']
                pos.y = closest_obstacle['lateral_offset']
                pos.z = 0.0
                self.pub_position.publish(pos)
                
                # Publish bounding box
                bbox_msg = Float32MultiArray()
                bbox_msg.data = closest_obstacle['bbox'].tolist()
                self.pub_bbox.publish(bbox_msg)
                
                self.detection_count += 1
                self.last_detection_time = rospy.Time.now()
                
                # Log detections (throttled)
                if self.detection_count % 10 == 1:
                    rospy.loginfo(f"Obstacle detected: {closest_obstacle['distance']:.2f}m ahead, "
                                f"{closest_obstacle['lateral_offset']:.2f}m lateral offset")
            
            # Publish debug image
            _, debug_img_encoded = cv2.imencode('.jpg', debug_img)
            debug_msg = CompressedImage()
            debug_msg.header.stamp = rospy.Time.now()
            debug_msg.format = "jpeg"
            debug_msg.data = debug_img_encoded.tobytes()
            self.pub_debug_image.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr_throttle(5.0, f"Error processing image: {e}")

if __name__ == "__main__":
    rospy.init_node('yolo_obstacle_detector', anonymous=False)
    YOLOObstacleDetector()
    rospy.spin()

