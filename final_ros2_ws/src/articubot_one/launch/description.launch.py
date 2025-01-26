import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('articubot_one'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
        
    # Command to process xacro and include parameters for ROS2 control and simulation time
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=false', ' sim_mode:=false'])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': False}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Return LaunchDescription with both the robot_state_publisher and rplidar nodes
    return LaunchDescription([
        node_robot_state_publisher
    ])
