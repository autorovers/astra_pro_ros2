#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

using namespace std::chrono_literals;

class CameraInfoNode : public rclcpp::Node
{
public:
    CameraInfoNode()
    : Node("camera_info")
    {
        // Camera Info publisher
        cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera_info", 
            10
        );

        // Service to update CameraInfo
        cam_info_srv_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(
            "set_camera_info",
            std::bind(&CameraInfoNode::setCameraInfoCb, 
                this,
                std::placeholders::_1, std::placeholders::_2
            )
        );

        // PStart camera info with values in zero
        cam_info_.header.frame_id = "camera_frame";
        cam_info_.width = 640;
        cam_info_.height = 480;

        // Matriz intrínseca (identidade inicial)
        cam_info_.k = {1.0, 0.0, cam_info_.width / 2.0,
                       0.0, 1.0, cam_info_.height / 2.0,
                       0.0, 0.0, 1.0};

        // Matriz de projeção (pode ser igual à K)
        cam_info_.p = {cam_info_.k[0], cam_info_.k[1], cam_info_.k[2], 0.0,
                       cam_info_.k[3], cam_info_.k[4], cam_info_.k[5], 0.0,
                       cam_info_.k[6], cam_info_.k[7], cam_info_.k[8], 0.0};

        cam_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0}; // distorção zerada

        // Timer para publicar periodicamente
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&CameraInfoNode::publishCameraInfo, this));

        RCLCPP_INFO(this->get_logger(), "CameraInfoNode iniciado.");
    }

private:
    void publishCameraInfo()
    {
        cam_info_.header.stamp = this->now();
        cam_info_pub_->publish(cam_info_);
    }

    void setCameraInfoCb(
        const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
        std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
    {
        cam_info_ = request->camera_info;
        response->success = true;
        response->status_message = "Camera info updated.";
        RCLCPP_INFO(this->get_logger(), "CameraInfo atualizado via serviço.");
    }

    sensor_msgs::msg::CameraInfo cam_info_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
    rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr cam_info_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraInfoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
