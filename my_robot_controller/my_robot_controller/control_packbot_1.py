#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Twist


class ControlPackbot1(Node):

    def __init__(self):
        super().__init__('control_packbot')
        
        #Publishers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel_to_tracks',10)

        #Timers
        self.timer = self.create_timer(1, self.pose_callback)
        
        self.target_position = None
        self.target_orientation = None
        self.kp_linear = 1.1
        self.kp_angular = 1.0
        
    def pose_callback(self): #executed in 1 sec. intervals
        self.pose_subscriber = self.create_subscription(PoseStamped,'/aruco_pose',self.velocity_command_callback,10)

    def velocity_command_callback(self,msg):
        self.target_position = msg.pose.position.z
        self.target_orientation = msg.pose.position.x
        
        if self.target_orientation is not None and self.target_position is not None:
            # self.get_logger().info('Command velocity')           
            cmd = Twist()    
            cmd.linear.x = self.kp_linear * (self.target_position-0.50) 
            
            if not (-0.10 <= self.target_orientation <= 0.10):
                cmd.angular.z = -(self.kp_angular * self.target_orientation)

            # # Speed limits
            # max_linear_speed = 0.50
            # max_angular_speed = 0.85
                
            # if abs(cmd.linear.x) > max_linear_speed:
            #    cmd.linear.x = max_linear_speed if cmd.linear.x > 0.00 else -max_linear_speed

            # if abs(cmd.angular.z) > max_angular_speed:
            #    cmd.angular.z = max_angular_speed if cmd.angular.z > 0.00 else -max_angular_speed

            self.velocity_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlPackbot1()
    rclpy.spin(node)            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main ()