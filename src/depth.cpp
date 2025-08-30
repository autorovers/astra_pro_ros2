#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>

#include <astra/astra.hpp>
#include <memory>
#include <iomanip>
#include <chrono>
#include <cstring>

using namespace std::chrono_literals;

class AstraDepthPublisher : public rclcpp::Node, public astra::FrameListener
{
public:
    AstraDepthPublisher()
    : Node("astra_depth_publisher")
    {
        astra::initialize();

        streamSet_ = std::make_unique<astra::StreamSet>();
        reader_ = std::make_unique<astra::StreamReader>(streamSet_->create_reader());

        // Criar e configurar DepthStream
        auto depthStream = reader_->stream<astra::DepthStream>();

        std::uint32_t FrameWidth_ = 640;
        std::uint32_t FrameHeight_ = 480;
        std::uint32_t StreamFPS_ = 30;
        astra_pixel_format_t PixelFormat_ = ASTRA_PIXEL_FORMAT_DEPTH_MM;

        astra::ImageStreamMode depthMode(FrameWidth_, FrameHeight_, StreamFPS_, PixelFormat_);
        depthStream.set_mode(depthMode);
        depthStream.start();

        // Info extra
        char serialnumber[256];
        depthStream.serial_number(serialnumber, 256);
        std::cout << "depthStream -- hFov: "
                  << depthStream.hFov()
                  << " vFov: "
                  << depthStream.vFov()
                  << " serial number: "
                  << serialnumber
                  << std::endl;

        reader_->add_listener(*this);

        // Timer para manter astra_update rodando
        timer_ = this->create_wall_timer(
            33ms,
            [this]() {
                astra_update();
            }
        );
    }

    ~AstraDepthPublisher()
    {
        reader_->remove_listener(*this);
        astra::terminate();
    }

    void init_transport()
    {
        image_transport::ImageTransport it(shared_from_this());
        publisher_ = it.advertise("/camera/depth", 10);
    }

    virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override
    {
        check_fps();

        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();
        if (!depthFrame.is_valid())
            return;

        unsigned int width = depthFrame.width();
        unsigned int height = depthFrame.height();

        if (width != lastWidth_ || height != lastHeight_)
        {
            buffer_ = std::make_unique<int16_t[]>(depthFrame.length());
            lastWidth_ = width;
            lastHeight_ = height;
        }

        depthFrame.copy_to(buffer_.get());

        // Preencher mensagem ROS
        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = this->now();
        msg.header.frame_id = "astra_depth_optical_frame";
        msg.height = height;
        msg.width = width;
        msg.encoding = "16UC1";  // profundidade em mm
        msg.is_bigendian = false;
        msg.step = width * sizeof(uint16_t);
        msg.data.resize(width * height * sizeof(uint16_t));
        std::memcpy(msg.data.data(), buffer_.get(), width * height * sizeof(uint16_t));

        publisher_.publish(msg);
    }

    void check_fps()
    {
        const float frameWeight = .2f;

        const ClockType::time_point now = ClockType::now();
        const float elapsedMillis = std::chrono::duration_cast<DurationType>(now - prev_).count();

        elapsedMillis_ = elapsedMillis * frameWeight + elapsedMillis_ * (1.f - frameWeight);
        prev_ = now;

        const float fps = 1000.f / elapsedMillis_;

        const auto precision = std::cout.precision();
        std::cout << std::fixed
                  << std::setprecision(1)
                  << fps << " fps ("
                  << std::setprecision(1)
                  << elapsedMillis_ << " ms)"
                  << std::setprecision(precision)
                  << std::endl;
    }

private:
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<astra::StreamSet> streamSet_;
    std::unique_ptr<astra::StreamReader> reader_;

    std::unique_ptr<int16_t[]> buffer_;
    unsigned int lastWidth_ = 0;
    unsigned int lastHeight_ = 0;

    using DurationType = std::chrono::milliseconds;
    using ClockType = std::chrono::high_resolution_clock;

    ClockType::time_point prev_;
    float elapsedMillis_{.0f};
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AstraDepthPublisher>();

    node->init_transport();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
