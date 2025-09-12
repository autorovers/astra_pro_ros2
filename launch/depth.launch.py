from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    namespace = "astra"

    depth_node = Node(
        package="astra_pro_ros2",
        executable="depth",
        name="depth_camera_publisher",
        namespace=namespace,
        parameters=[
            {"width": 640},
            {"height": 480},
            {"fps": 30},
            {"check_fps": False},
            {"only_compressed": False},
        ],
        remappings={
            ("depth", "depth/image_rect_raw"),
        }
    )

    ld.add_action(depth_node)
    
    return ld