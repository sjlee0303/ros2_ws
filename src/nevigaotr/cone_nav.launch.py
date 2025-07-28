from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:

    # ① Cone Map Builder
    cone_map_node = Node(
        package='cone_navigation_pkg',
        executable='cone_map_builder',
        name='cone_map_builder',
        output='screen',
        parameters=[{
            'source_frame' : 'velodyne',   # 이미 존재하는 프레임
            'target_frame' : 'velodyne',   # ← map 대신 velodyne
            'merge_radius' : 0.45,
            'ema_alpha'    : 0.3,
            'forget_after' : 4.0
        }]
    )

    # ② Waypoint / Path Generator
    waypoint_node = Node(
        package='cone_navigation_pkg',
        executable='waypoint_path_node',
        name='waypoint_path_node',
        output='screen',
        parameters=[{
            'lane_half'     : 1.0,
            'pair_long_tol' : 0.7,
            'hist_keep'     : 50
        }]
    )

    return LaunchDescription([
        cone_map_node,
        waypoint_node,
    ])
