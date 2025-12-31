from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare('click_path_planner')

    ground_cloud = PathJoinSubstitution([
        pkg_share,
        'data',
        'hub_ground_points.pcd'
        #'microgrid_ground.pcd'
        #'gratiot_ground.pcd'
    ])

    offground_cloud = PathJoinSubstitution([
        pkg_share,
        'data',
        'hub_off_ground_points_segmented3.pcd'
        #'microgroud_obs.pcd'
        #'gratiot_off_ground.pcd'
    ])

    '''load_save_file = PathJoinSubstitution([
        pkg_share,
        'saves',
        'save_2025-11-26_19.57.34.txt',
    ])'''

    click_path_node = Node(
        package='click_path_planner',
        executable='click_path_planner',
        name='click_path_planner',
        output='screen',
        parameters=[{
            'ground_cloud_path': ground_cloud,
            'offground_cloud_path': offground_cloud,
            'resolution': 0.05,

            'saves_dir': '/root/ros_overlay_ws/src/click_path_planner/saves/',
            'load_save_file': '/root/ros_overlay_ws/src/click_path_planner/saves/save_2025-12-04_20.00.04.txt',
            'load_map_save_file': '/root/ros_overlay_ws/src/click_path_planner/saves/voltage_demo.txt'
        }]
    )

    return LaunchDescription([
        click_path_node
    ])