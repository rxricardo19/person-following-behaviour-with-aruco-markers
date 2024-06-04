#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class NavigationNode(Node): 
    def __init__(self):
        super().__init__("navigation") 
        self.navigator = BasicNavigator()
        self.goal = PoseStamped()        
        self.current_pose = PoseStamped()     
        self.path_history = []
    
        self.last_detection_time = None
        self.task_canceled = False
        
        #Timers        
        self.return_timer = self.create_timer(0.5, self.check_return_to_base)
        
        #Subscribers
        self.transformation_subscriber = self.create_subscription(PoseStamped,'/goal_pose_external', self.goalpose_callback,10)  
        self.current_pose_subscriber = self.create_subscription(PoseStamped,'/current_pose',self.current_pose_callback,10)
    
    def goalpose_callback(self,msg):
        #Handle incoming goal pose
        self.navigator.cancelTask()
        self.task_canceled = True
        self.last_detection_time = self.get_clock().now()
        self.goal.header.frame_id = 'map'
        self.goal.header.stamp = self.last_detection_time.to_msg()
        self.goal.pose.position.x = msg.pose.position.x 
        self.goal.pose.position.y = msg.pose.position.y 
        self.goal.pose.orientation = self.current_pose.pose.orientation #goal orientation = current orientation

        #Gets and smooth the path to the goal pose
        path = self.navigator.getPath(self.current_pose,self.goal)
        sm_path = self.navigator.smoothPath(path)
        self.navigator.followPath(sm_path)
        
    def current_pose_callback(self,currentpose):
        #Keeps track of the navigated postions
        self.current_pose.header.frame_id = 'map'
        self.current_pose.pose = currentpose.pose 
        self.path_history.append(self.current_pose.pose)
            
    def check_return_to_base(self):
        #Return-to-base behaviour
        if self.task_canceled and self.last_detection_time:
            current_time = self.get_clock().now()
            elapsed_time = current_time - self.last_detection_time
            
            if elapsed_time.nanoseconds / 1e9 > 30.0: #detection timeout
                #If marker becomes undetected for more than 30.0 secs:
                self.get_logger().info("No marker detected for a while; returning to base.")
                reverse_path = list(reversed(self.path_history))
                goals = []
                for pose in reverse_path:
                    goal = PoseStamped()
                    goal.header.frame_id = 'map'
                    goal.pose = pose
                    goals.append(goal)    
                self.navigator.goThroughPoses(goals)
                
                self.path_history = []
                self.last_detection_time = None
                
    # def save_path_history(self):
    # #creates text file with the tracked navigated positions
    #     with open('path_history.txt', 'w') as file:
    #         for pose in self.path_history:
    #             file.write(f"X= {pose.position.x}, Y= {pose.position.y}\n")
    
def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    finally:
        # node.save_path_history()
        node.destroy_node()
        rclpy.shutdown()       

if __name__ == "__main__":
	main()