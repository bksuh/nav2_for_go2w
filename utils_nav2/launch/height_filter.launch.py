from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='utils_nav2',
            executable='height_filter_node',
            name='height_filter_node',
            output='screen',
            parameters=[
                {'input_topic': '/kiss/local_map'},
                {'output_topic': '/kiss/filtered_map'},
                {'ground_frame': 'odom'},          # 또는 'base_footprint' / 'odom'
                {'min_height': 0.5},
                {'max_height': float('inf')},
                {'use_tf': True},
                {'preserve_stamp': True},
            ],
        )
    ])
