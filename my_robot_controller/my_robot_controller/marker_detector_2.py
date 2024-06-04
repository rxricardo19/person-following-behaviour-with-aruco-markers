#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node  
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import cv2
import cv2.aruco as aruco 
from cv_bridge import CvBridge

import numpy as np
from geometry_msgs.msg import PoseStamped 
from squaternion import Quaternion
 
#ArUco dictionaries 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

desired_aruco_dictionary = "DICT_5X5_100"
desired_aruco_dictionary2 = "DICT_4X4_100"

class MarkerDetectorNode2(Node):

    def __init__(self):
        super().__init__('marker_detector')
        self.image_subcriber = self.create_subscription(Image,'/camera/camera/color/image_raw', self.image_callback,10)   
        self.camera_info_subscriber = self.create_subscription(CameraInfo, 'camera/camera/color/camera_info',self.camera_info_callback,10)

        self.aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
        self.aruco_dict2 = aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary2])
        
        self.aruco_parameters = aruco.DetectorParameters() #default aruco parameters used for detection
        
        self.marker_points_3d = np.array([[0, 0, 0], [0.110, 0, 0], [0.110, 0.110, 0], [0, 0.110, 0]]) #coordinates of the marker 4 corners
        self.marker_points2_3d = np.array([[0, 0, 0], [0.090, 0, 0], [0.090, 0.090, 0], [0, 0.090, 0]])
        
        self.pose_publisher = self.create_publisher(PoseStamped, '/aruco_pose',10)
        self.pose_publisher2 = self.create_publisher(PoseStamped, '/aruco_pose_2',10)
        
    def image_callback(self, color_image): #called whenever /image_raw topic gets a msg 
        br_rgb = CvBridge()
        current_frame = br_rgb.imgmsg_to_cv2(color_image,'bgr8') #conversion to an opencv image format
        gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, _ = aruco.detectMarkers(gray_frame, self.aruco_dict, parameters=self.aruco_parameters) 
        corners2, ids2, _ = aruco.detectMarkers(gray_frame, self.aruco_dict2, parameters=self.aruco_parameters) 
        
        if ids is not None: #if any marker was detected
            aruco.drawDetectedMarkers(current_frame, corners, ids) #draws the corners and the id of the detected marker       
            for i in range(len(ids)):
                ret, rvec, tvec = cv2.solvePnP(self.marker_points_3d, corners[i], self.camera_matrix, self.distortion_coeffs)
                rvec = np.round(rvec,2)
                tvec = np.round(tvec,2)
                
                if ret: #If the pose estimation was successful: - ret is a boolean value (true or false) that indicates if solvePnP found or not a solution.
                    cv2.drawFrameAxes(current_frame, self.camera_matrix, self.distortion_coeffs, rvec, tvec, 0.03) #draws the axis frame on the detected marker
                    
                    #we add a string that represents the trans. and rot. vectors of the marker's pose
                    pose_str = f"T1: {np.squeeze(tvec)} R1:{np.squeeze(rvec)}"                    
                    cv2.putText(current_frame,pose_str,(0,i*30+20),cv2.FONT_HERSHEY_TRIPLEX,0.80,(0,0,0),4)
                    cv2.putText(current_frame,pose_str,(0,i*30+20),cv2.FONT_HERSHEY_TRIPLEX,0.80,(0,255,0),1)
                    quat = Quaternion.from_euler(rvec[0],rvec[1],rvec[2], degrees=True)
                    
                    msg = PoseStamped() #PoseStamped msg: gives the pose info
                    msg.header.frame_id = 'map'
                    msg.pose.position.x = float(tvec[0])
                    msg.pose.position.y = float(tvec[1])
                    msg.pose.position.z = float(tvec[2])
                    msg.pose.orientation.w = float(quat[0])
                    msg.pose.orientation.x = float(quat[1])
                    msg.pose.orientation.y = float(quat[2])
                    msg.pose.orientation.z = float(quat[3])
            
                    self.pose_publisher.publish(msg) #we publish the PoseStamped msg to the /aruco_pose topic
                    
        if ids2 is not None: #if any marker was detected
            aruco.drawDetectedMarkers(current_frame, corners2, ids2) #draws the corners and the id of the detected marker       
            for i in range(len(ids2)):
                ret2, rvec2, tvec2 = cv2.solvePnP(self.marker_points2_3d, corners2[i], self.camera_matrix, self.distortion_coeffs)
                rvec2 = np.round(rvec2,2)
                tvec2 = np.round(tvec2,2)
                
                if ret2: #If the pose estimation was successful: - ret is a boolean value (true or false) that indicates if solvePnP found or not a solution.
                    cv2.drawFrameAxes(current_frame, self.camera_matrix, self.distortion_coeffs, rvec2, tvec2, 0.03) #draws the axis frame on the detected marker
                    
                    #we add a string that represents the trans. and rot. vectors of the marker's pose
                    pose2_str = f"T2: {np.squeeze(tvec2)} R2:{np.squeeze(rvec2)}" 
                    cv2.putText(current_frame,pose2_str,(0,i*30+60),cv2.FONT_HERSHEY_TRIPLEX,0.80,(0,0,0),4)
                    cv2.putText(current_frame,pose2_str,(0,i*30+60),cv2.FONT_HERSHEY_TRIPLEX,0.80,(0,0,255),1)
                    quat2 = Quaternion.from_euler(rvec2[0],rvec2[1],rvec2[2], degrees=True)

                    msg2 = PoseStamped() #PoseStamped msg: gives the pose info
                    msg2.header.frame_id = 'map'         
                    msg2.pose.position.x = float(tvec2[0])
                    msg2.pose.position.y = float(tvec2[1])
                    msg2.pose.position.z = float(tvec2[2])
                    msg2.pose.orientation.w = float(quat2[0])
                    msg2.pose.orientation.x = float(quat2[1])
                    msg2.pose.orientation.y = float(quat2[2])
                    msg2.pose.orientation.z = float(quat2[3])
    
                    self.pose_publisher2.publish(msg2) #we publish the PoseStamped msg to the /aruco_pose topic
                    
        cv2.imshow("RGB", current_frame)
        cv2.waitKey(1)

    def camera_info_callback(self, camera_info): #called whenever /camera_info topic gets a msg 
        self.camera_matrix = np.array(camera_info.k).reshape(3,3) #reshapes the flattened array into a 3x3 matrix
        self.distortion_coeffs = np.array(camera_info.d)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetectorNode2()
    rclpy.spin(node)            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main ()