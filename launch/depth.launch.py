from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    depth_node = Node(
        package="astra_pro_ros2",
        executable="depth",
        name="depth_camera_publisher",
        namespace="astra",
        parameters=[
            {"width": 640},
            {"height": 480},
            {"fps": 30},
            {"check_fps": False},
        ],
    )

    ld.add_action(depth_node)
    
    return ld