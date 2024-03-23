import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    orbslam3_node = Node(
        package='orbslam3',
        namespace='orbslam3',
        executable='mono-pcloud',
        name='monocular',
        output='screen',
        parameters=[{
            "vocabulary_file": "/workspaces/ORB_SLAM3_ROS2_Docker/vocabulary/ORBvoc.txt",
            "slam_config_file": "/workspaces/ORB_SLAM3_ROS2_Docker/config/monocular/TUM1_wide.yaml"
        }],
        # prefix=['gdbserver localhost:3000'],
        # remappings=[
        #     ('/cloud_in', '/no_ground/oneshot/pointcloud'),
        #     ('/orbslam3/monocular/pcloud_all', '/concatenated/pointcloud'),
        # ]
    )

    img_stream_node = Node(
        package='orbslam3',
        namespace='orbslam3',
        executable='image-stream',
        name='image_stream',
    )

    img_view_node = Node(
        package='image_view',
        namespace='orbslam3',
        executable='image_view',
        name='tracking_image_view',
        remappings=[
            ('image', '/orbslam3/image_stream/image_raw')
        ]
    )

    tracking_view_node = Node(
        package='image_view',
        namespace='orbslam3',
        executable='image_view',
        name='image_view',
        remappings=[
            ('image', '/orbslam3/monocular/tracking_image')
        ]
    )

    pcloud_to_map_config = os.path.join(
      'config',
      'pcloud-to-map',
      'pcloud-to-map.yaml',
      )
    
    pcloud_to_map_node = Node(
        package='orbslam3',
        namespace='orbslam3',
        executable='pcloud-to-map',
        name='pcloud_to_map',
        parameters=[pcloud_to_map_config]
    )

    project_map = Node(
        package='orbslam3',
        namespace='orbslam3',
        executable='project-map',
        name='project_map'
    )

    rviz2 = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join('config', 'rviz2', 'rviz_main.rviz')]
    )

    return LaunchDescription([
        orbslam3_node,
        # rviz2,
        # img_view_node,
        #tracking_view_node,
        # pcloud_to_map_node,
        # project_map,
        # img_stream_node,
    ])