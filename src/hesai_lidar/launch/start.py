from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config=get_package_share_directory('hesai_lidar')+'/rviz/rviz2.rviz'
    return LaunchDescription([
        Node(namespace='hesai_lidar', package='hesai_lidar', executable='hesai_lidar_node', output='screen'),
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])
