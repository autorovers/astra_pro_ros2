from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    infrared_node = Node(
        package="astra_pro_ros2",
        executable="infrared",
        name="infrared_camera_publisher",
        namespace="astra",
        parameters=[
            {"width": 640},
            {"height": 480},
            {"fps": 30},
            {"check_fps": False},
            {"only_compressed": True},
        ],
        remappings={
            ("infrared", "infrared/image_rect_raw"),
        }
    )

    ld.add_action(infrared_node)
    
    return ld