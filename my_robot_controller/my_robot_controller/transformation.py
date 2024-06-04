#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf2_ros
import tf2_geometry_msgs

class TransformationNode(Node): 
    def __init__(self):
        super().__init__("transformation") 
        self.navigator = BasicNavigator()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)
        self.currentpose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.goal_pose_transf = PoseStamped()
        
        #Subscribers
        self.pose_subcriber = self.create_subscription(PoseStamped,'/aruco_pose',self.goalpose_callback,10) 
        
        #Publishers
        self.currentpose_publisher = self.create_publisher(PoseStamped,'/current_pose',10)
        self.transformation_publisher = self.create_publisher(PoseStamped,'/external_pose',10)
                            
    def goalpose_callback(self,aruco): 
         
        self.goal_pose.header.frame_id = 'base_link'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        if aruco.pose.position.z >= 0.80: 
            self.goal_pose.pose.position.x = (aruco.pose.position.z - 0.80) #0.80m for base_link (keeps camera_frame at 0.30m to the person)
            
        if not (-0.10 < aruco.pose.position.x < 0.10):
            self.goal_pose.pose.position.y = -aruco.pose.position.x 
        else:
            self.goal_pose.pose.position.y = 0.00
        
        self.goal_pose.pose.orientation = aruco.pose.orientation #goal_orientation = current orientation

        self.transform = self.tf_buffer.lookup_transform('map','base_link',rclpy.time.Time().to_msg(),rclpy.duration.Duration(seconds=3.0)) #with 3 seconds timeout to compute a pose transformation or throw an exception
        goal_pose_in_map = tf2_geometry_msgs.do_transform_pose(self.goal_pose.pose, self.transform)

        self.currentpose.header.frame_id = 'map'
        self.currentpose.pose.position.x = self.transform.transform.translation.x
        self.currentpose.pose.position.y = self.transform.transform.translation.y
        self.currentpose.pose.orientation = self.transform.transform.rotation
        self.currentpose_publisher.publish(self.currentpose)
         
        self.goal_pose_transf.header.frame_id = 'map'
        self.goal_pose_transf.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose_transf.pose = goal_pose_in_map
        self.transformation_publisher.publish(self.goal_pose_transf)
          
def main(args=None):
    rclpy.init(args=args)
    node = TransformationNode()
    rclpy.spin(node)
    rclpy.shutdown()       
	
if __name__ == "__main__":
	main()