from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    color_node = Node(
        package="astra_pro_ros2",
        executable="color",
        name="rgb_calibration_camera_publisher",
        namespace="astra",
        parameters=[
            {"width": 640},
            {"height": 480},
            {"fps": 30},
            {"check_fps": False},
        ],
    )

    # TODO: Implementar o parser do arquivo de calibracao da camera e criar o node de camera info publisher

    # camera_info_node = Node(
    #     package="astra_pro_ros2",
    #     executable="camera_info_calibration",
    #     name="camera_info_calibration",
    #     namespace="astra/color",
    # )

    ld.add_action(color_node)
    # ld.add_action(camera_info_node)

    return ld