from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    rviz_config=get_package_share_directory('hesai_lidar')+'/rviz/rviz2.rviz'
    return LaunchDescription([
        Node(
            package='hesai_lidar',
            node_namespace='hesai_lidar',
            node_name='hesai_lidar_node',
            node_executable='hesai_lidar_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            node_namespace='rviz2',
            node_name='rviz2',
            node_executable='rviz2',
            arguments=['-d',rviz_config]
        )
    ])
