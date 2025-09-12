from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    rgbd_node = Node(
        package="astra_pro_ros2",
        executable="multi",
        name="rgbd_camera_publisher",
        namespace="astra",
        parameters=[
            {"width": 640},
            {"height": 480},
            {"fps": 30},
            {"check_fps": True},
            {"only_compressed": False},
        ],
        remappings={
            ("infrared", "infrared/image_rect_raw"),
            ("color", "color/image_raw"),
            ("depth", "depth/image_rect_raw"),
        }
    )

    ld.add_action(rgbd_node)

    return ld