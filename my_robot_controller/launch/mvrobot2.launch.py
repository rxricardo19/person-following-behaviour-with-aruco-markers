from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os 

def generate_launch_description():
    ld = LaunchDescription()

    MarkerDetectorNode2 = Node(
        package="my_robot_controller",
        executable="marker_detector2",
    )

    ControlPackbot2 = Node(
        package="my_robot_controller",
        executable="control_packbot2",
        remappings= [('/cmd_vel_to_tracks','/tracks_controller/cmd_vel_unstamped')],

    )    

    TransformationNode = Node(
        package="my_robot_controller",
        executable="transformation",
        remappings=[('/external_pose','/goal_pose_external')],

    )    
    
    NavigationNode = Node(
        package="my_robot_controller",
        executable="nav_to_pose",

    ) 
    
    RealSenseNode = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",

    )
            
    #Include other launch file: 
    RealSenseNode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 
                         'launch/rs_launch.py')
        )
    )
    
    ld.add_action(MarkerDetectorNode2)
    ld.add_action(ControlPackbot2)
    ld.add_action(TransformationNode)
    ld.add_action(NavigationNode)
    ld.add_action(RealSenseNode)
    
    return ld

#remapping launch files
#make the robot to turn with the aruco horizontal movement (y)