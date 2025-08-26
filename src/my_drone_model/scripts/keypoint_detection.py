import rospy
from sensor_msgs.msg import Image
from my_custom_msgs.msg import KeyPoints, KpYolo
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import os
import time
import csv
import pyautogui
import torch
from ultralytics import YOLO

class DrawCubeOverlay:
    def __init__(self):
        rospy.init_node("draw_cube_overlay")

        self.running = True

        self.bridge = CvBridge()
        self.image_with_cube = None
        self.screen_width, self.screen_height = pyautogui.size()

        #KP RCNN MODEL
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

        self.root_dir = os.path.join(os.path.expanduser("~"), 'drone_ws')
        self.model_path = os.path.join(self.root_dir, "src","train_yolo_model","jackal_detection_3d3","weights","best.pt")

        self.model = YOLO(self.model_path)
        self.model.to(self.device)

        self.image_sub = rospy.Subscriber('/front_camera/image_raw', Image, self.image_callback, queue_size=1)
        self.publish_keypoints = rospy.Publisher('/yolo_keypoints', KpYolo, queue_size=1)
        
        self.key_points = KpYolo()

        self.show_img_thread = threading.Thread(target = self.show_img_thread_callback)
        self.show_img_thread.daemon = True
        self.show_img_thread.start()



            
    def draw_cube(self, image, keypoints):

        edges = [
            (0, 1), (1, 3), (2, 3), (2, 0),  # Bottom face
            (4, 5), (5, 7), (6, 7), (6, 4),  # Top face
            (0, 4), (1, 5), (2, 6), (3, 7)   # Vertical edges
        ]

        # Draw edges on the image
        for start, end in edges:
            pt1 = tuple(map(int, keypoints[start]))
            pt2 = tuple(map(int, keypoints[end]))
            cv2.line(image, pt1, pt2, color=(0, 255, 0), thickness=2)  # Green edges

        # Draw keypoints as circles
        for idx, (x, y) in enumerate(keypoints):
            cv2.circle(image, (int(x), int(y)), radius=5, color=(0, 0, 255), thickness=-1)  # Red dots
            cv2.putText(image, str(idx + 1), (int(x) + 5, int(y) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        return image


    def image_callback(self, msg):

        try:
            # Convert ROS2 Image message to OpenCV image (NumPy array)
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        if image is None:
            print(f"Error: No image input.")
            return
        
        results = self.model(image)
        if not results or not hasattr(results[0], 'keypoints'):
            print(f"Error: Unable to detect keypoints.")
            return

        keypoints_tensor = results[0].keypoints.xy  # Extract xy coordinates
        keypoints = keypoints_tensor.cpu().numpy()[0]  # Convert to NumPy array

        if keypoints.shape[0] != 8:
            print(f"Error: Expected 8 keypoints, but got {keypoints.shape[0]}. Skipping.")
            return
        
        self.image_with_cube = self.draw_cube(image, keypoints)


        keypoints_flat = keypoints.flatten().tolist()
        self.key_points.target = True
        self.key_points.keypoints.data = keypoints_flat
        self.publish_keypoints.publish(self.key_points)

    def show_img_thread_callback(self):
        while self.running:
            if self.image_with_cube is not None:
                new_width = self.screen_width // 2
                new_height = self.screen_height // 2
                
                cv2.namedWindow("Image with keypoints cube", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Image with keypoints cube", new_width, new_height)
                
                cv2.imshow("Image with keypoints cube", self.image_with_cube)
                cv2.waitKey(1)
            time.sleep(0.02)



if __name__ == '__main__':
    node = DrawCubeOverlay()
    rospy.spin()
