import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    orbslam3_node = Node(
        package='orbslam3',
        namespace='orbslam3',
        executable='stereo',
        name='stereo',
        output='screen',
        parameters=[{
            "vocabulary_file": "/workspaces/ORB_SLAM3_ROS2_Docker/vocabulary/ORBvoc.txt",
            "slam_config_file": "/workspaces/ORB_SLAM3_ROS2_Docker/config/stereo/config_1024.yaml",
            "doRectify": True,
            "enablePangolin": False,
        }],
        # prefix=['gdbserver localhost:3000'],
        # remappings=[
        #     ('/cloud_in', '/no_ground/oneshot/pointcloud'),
        #     ('/orbslam3/monocular/pcloud_all', '/concatenated/pointcloud'),
        # ]
    )

    img_stream_left_node = Node(
        package='orbslam3',
        namespace='orbslam3',
        executable='image-stream',
        name='image_stream_left',
        parameters=[{
            "video_capture_stream": "tcp://192.168.1.17:8888",
        }]
    )

    img_stream_right_node = Node(
        package='orbslam3',
        namespace='orbslam3',
        executable='image-stream',
        name='image_stream_right',
        parameters=[{
            "video_capture_stream": "tcp://192.168.1.17:8889",
        }],
    )

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server_node',
        parameters=[{
            "base_frame_id": "octomap_link",
            "resolution": 0.01,
            "filter_speckles": True,
            # "sensor_model.max_range": 5.0,
            # "use_height_map": True,
            # "point_cloud_min_z": 0.0,
            # "point_cloud_max_z": 3.0,
        }],
    )

    img_view_left_node = Node(
        package='image_view',
        namespace='orbslam3',
        executable='image_view',
        name='image_view_left',
        remappings=[
            ('image', '/orbslam3/image_stream_left/image_raw')
        ]
    )

    img_view_right_node = Node(
        package='image_view',
        namespace='orbslam3',
        executable='image_view',
        name='image_view_right',
        remappings=[
            ('image', '/orbslam3/image_stream_right/image_raw')
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
        rviz2,
        octomap_node,
        # img_view_left_node,
        # img_view_right_node,
        # tracking_view_node,
        # pcloud_to_map_node,
        # project_map,
        img_stream_left_node,
        img_stream_right_node,
    ])