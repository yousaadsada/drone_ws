#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

bridge = CvBridge()
save_dir = "/home/yousa/drone_ws/src/image"  # Change to your desired path

# Create directory if it doesn't exist
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

def callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Create filename using timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = os.path.join(save_dir, f"image_{timestamp}.jpg")
        
        cv2.imwrite(filename, cv_image)
        rospy.loginfo(f"Saved: {filename}")
        
    except Exception as e:
        rospy.logerr(f"Failed to save image: {e}")

rospy.init_node('image_saver_continuous')
rospy.Subscriber('/front_camera/image_raw', Image, callback)
rospy.spin()
