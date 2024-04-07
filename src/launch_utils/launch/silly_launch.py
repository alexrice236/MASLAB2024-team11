from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    color = DeclareLaunchArgument(
        "color",
        default_value=TextSubstitution(text="red")
    )
    return LaunchDescription([
        color,
        Node(
            package='v4l2_camera',
            namespace='v4l2_camera',
            executable='v4l2_camera_node',
            parameters=[
                {"camera_info_url": 'default'},
                {"video_device": '/dev/video0'},
                {"image_size": [640, 480]}
            ]
        ),
        Node(
            package='camera',
            namespace='camera',
            executable='image_proc'
        ),
        Node(
            package='camera',
            namespace='camera',
            executable='homography_transformer'
        ),
        Node(
            package='drive',
            namespace='drive',
            executable='rotation_conversion'
        ),
        Node(
            package='drive',
            namespace='drive',
            executable='driver',
            parameters=[
                {"color": LaunchConfiguration("color")}
            ]
        ),
        Node(
            package='drive',
            namespace='drive',
            executable='kitbot'
        ),
        # Node(
        #     package='rqt_gui',
        #     namespace='rqt_gui',
        #     executable='rqt_gui'
        # ),
    ])

