#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Twist


class ControlPackbot2(Node):

    def __init__(self):
        super().__init__('control_backwards')
        self.target_position = None
        self.target_orientation = None

        #Publishers
        self.velocity_publisher = self.create_publisher(Twist,'/cmd_vel_to_tracks',10)

        #Timers
        self.timer = self.create_timer(1, self.pose_callback)

    def pose_callback(self): #exec. in 1 sec. intervals
        self.pose_subcriber = self.create_subscription(PoseStamped,'/aruco_pose_2',self.velocity_command_callback,10)

    def velocity_command_callback(self,msg):
        #Control Gains
        kp_linear = 1.1
        kp_angular = 1.0
    
        self.target_position = msg.pose.position.z
        self.target_orientation = msg.pose.position.x
            
        if self.target_orientation is not None and self.target_position is not None:                   
            cmd = Twist() 
            if self.target_position < 0.50:
                cmd.linear.x = kp_linear * (self.target_position-0.50)     
            if not (-0.10 <= self.target_orientation <= 0.10):
                cmd.angular.z = -(kp_angular * self.target_orientation)

            #Speed limits
            #max_linear_speed = 1.00
            #max_angular_speed = 1.00

            #if abs(cmd.linear.x) > max_linear_speed:
             #   cmd.linear.x = max_linear_speed if cmd.linear.x > 0.00 else -max_linear_speed

            #if abs(cmd.angular.z) > max_angular_speed:
             #   cmd.angular.z = max_angular_speed if cmd.angular.z > 0.00 else -max_angular_speed

            self.velocity_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlPackbot2()
    rclpy.spin(node)            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main ()