from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os 

def generate_launch_description():
    ld = LaunchDescription()

    MarkerDetectorNode1 = Node(
        package="my_robot_controller",
        executable="marker_detector1",
    )

    ControlPackbot1 = Node(
        package="my_robot_controller",
        executable="control_packbot1",
        remappings= [('/cmd_vel_to_tracks','/tracks_controller/cmd_vel_unstamped')],

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
    
    ld.add_action(MarkerDetectorNode1)
    ld.add_action(ControlPackbot1)
    ld.add_action(RealSenseNode)

    return ld

#remapping launch files
#make the robot to turn with the aruco horizontal movement (y)