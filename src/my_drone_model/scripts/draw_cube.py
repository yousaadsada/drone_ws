import rospy
from sensor_msgs.msg import Image
from my_custom_msgs.msg import KeyPoints
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import os
import time
import csv
import tkinter as tk
from std_msgs.msg import Bool

class DrawCubeOverlay:
    def __init__(self):
        rospy.init_node("draw_cube_overlay")

        self.bridge = CvBridge()
        self.image_raw = None
        self.image_cube = None

        self.start_time = time.time()

        self.image_sub = rospy.Subscriber('/front_camera/image_raw', Image, self.image_callback, queue_size=1)
        self.keypoints_sub = rospy.Subscriber('/keypoints_2d', KeyPoints, self.keypoints_callback, queue_size=1)
        self.data_collection_flag_sub = rospy.Subscriber('/data_collection_flag', Bool, self.data_collection_flag_callback, queue_size=1)

        self.jackal_figure_raw_dir = os.path.join(os.path.expanduser("~"), 'drone_ws', 'data', 'jackal_figure','raw')
        os.makedirs(self.jackal_figure_raw_dir, exist_ok=True) 

        self.jackal_figure_with_cube_dir = os.path.join(os.path.expanduser("~"), 'drone_ws', 'data', 'jackal_figure','cube')
        os.makedirs(self.jackal_figure_with_cube_dir, exist_ok=True) 

        self.keypoint_dir = os.path.join(os.path.expanduser("~"), 'drone_ws', 'data', 'keypoint')
        os.makedirs(self.keypoint_dir, exist_ok=True) 
        self.csv_file_path = os.path.join(self.keypoint_dir, 'keypoints.csv')


        self.number = 0
        self.last_save_time = 0 
        self.keypoints_2d = None  # store the latest keypoints
        self.keypoints_name  = [
            'front_top_left',     # 0
            'front_top_right',    # 1
            'front_bottom_left',  # 2
            'front_bottom_right', # 3
            'rear_top_left',      # 4
            'rear_top_right',     # 5
            'rear_bottom_left',   # 6
            'rear_bottom_right'   # 7
        ]

        self.init_csv_file()

        self.collecting = False

    #     self.gui_thread = threading.Thread(target=self.run_gui)
    #     self.gui_thread.daemon = True
    #     self.gui_thread.start()

    # def run_gui(self):
    #     self.root = tk.Tk()
    #     self.root.title("Collecting data")

    #     self.start_button = tk.Button(self.root, text="Start Collecting", command=self.start_collection)
    #     self.start_button.grid(row=0, column=0)

    #     self.stop_button = tk.Button(self.root, text="Stop Collecting", command=self.stop_collection)
    #     self.stop_button.grid(row=0, column=1)
    #     self.root.mainloop()

    # def start_collection(self):
    #     self.collecting = True
    #     rospy.loginfo("Data collection started.")

    # def stop_collection(self):
    #     self.collecting = False
    #     rospy.loginfo("Data collection stopped.")

    def data_collection_flag_callback(self, msg):
        self.collecting = msg.data


    def init_csv_file(self):
        with open(self.csv_file_path, mode="w", newline="") as file:
            writer = csv.writer(file)
            header = ["Timestamp"] + ["center_x","center_y","width","height"] + [f"{kp}_{coord}" for kp in self.keypoints_name for coord in ["x", "y"]]
            writer.writerow(header)


    def keypoints_callback(self, msg):
        self.keypoints_2d = msg

    def get_point(self, keypoints, name):
        return getattr(keypoints, name)

    def image_callback(self, msg):
            
        def save_figure():
            filename = os.path.join(self.jackal_figure_with_cube_dir, f"jackal_cube_{self.number}.png")
            cv2.imwrite(filename, self.image_cube)

            filename = os.path.join(self.jackal_figure_raw_dir, f"jackal_raw_{self.number}.png")
            cv2.imwrite(filename, self.image_raw)

        def save_keypoint(center_x, center_y, width, height, points):
            with open(self.csv_file_path, mode="a", newline="") as file:
                writer = csv.writer(file)
                # Flatten the keypoints_list
                bbox = [center_x, center_y, width, height]
                flattened_keypoints = [coord for keypoint in points for coord in keypoint]
                combined_row = bbox + flattened_keypoints
                writer.writerow(combined_row) 


        if self.keypoints_2d is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]
            # Convert to numpy points
            points = []
            for name in self.keypoints_name:
                pt = self.get_point(self.keypoints_2d, name)
                points.append((int(pt.x), int(pt.y)))
            
            all_points_valid = True
            for x, y in points:
                if not (0 <= x < width and 0 <= y < height):
                    all_points_valid = False
          
            self.image_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Draw edges of the cube
            color = (0, 255, 0)
            thickness = 1

            color_2d = (255, 0, 0)

            # Top face
            cv2.line(cv_image, points[0], points[1], color, thickness)
            cv2.line(cv_image, points[1], points[5], color, thickness)
            cv2.line(cv_image, points[5], points[4], color, thickness)
            cv2.line(cv_image, points[4], points[0], color, thickness)

            # Bottom face
            cv2.line(cv_image, points[2], points[3], color, thickness)
            cv2.line(cv_image, points[3], points[7], color, thickness)
            cv2.line(cv_image, points[7], points[6], color, thickness)
            cv2.line(cv_image, points[6], points[2], color, thickness)

            # Vertical edges
            cv2.line(cv_image, points[0], points[2], color, thickness)
            cv2.line(cv_image, points[1], points[3], color, thickness)
            cv2.line(cv_image, points[4], points[6], color, thickness)
            cv2.line(cv_image, points[5], points[7], color, thickness)

            x_coords = [x for x, y in points]
            y_coords = [y for x, y in points]
            min_x, max_x = min(x_coords), max(x_coords)
            min_y, max_y = min(y_coords), max(y_coords)
            center_x = int((min_x + max_x) / 2)
            center_y = int((min_y + max_y) / 2)
            width = int(max_x - min_x)
            height = int(max_y - min_y)

            point_1 = (min_x, min_y) 
            point_2 = (min_x, max_y) 
            point_3 = (max_x, max_y) 
            point_4 = (max_x, min_y) 

            cv2.line(cv_image, point_1, point_2, color_2d, thickness)
            cv2.line(cv_image, point_2, point_3, color_2d, thickness)
            cv2.line(cv_image, point_3, point_4, color_2d, thickness)
            cv2.line(cv_image, point_4, point_1, color_2d, thickness)

            self.image_cube = cv_image

            current_time = time.time()

            # if current_time - self.start_time >= 5.0:  # wait at least 2 seconds after start
            if all_points_valid and (current_time - self.last_save_time >= 0.05):
                if self.collecting == True:
                    self.number += 1  
                    save_figure()
                    save_keypoint(center_x, center_y, width, height, points)
                    self.last_save_time = current_time

            # Show or save
            cv2.imshow("Jackal 3D Cube Projection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Failed to draw cube: {e}")



if __name__ == '__main__':
    node = DrawCubeOverlay()
    rospy.spin()
