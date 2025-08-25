// Core ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>

// Bridge OpenCV-ROS
#include <cv_bridge/cv_bridge.hpp>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Astra SDK
#include <astra/astra.hpp>
// #include <key_handler.h>


using namespace std::chrono_literals;

class AstraColorPublisher : public rclcpp::Node, public astra::FrameListener
{
public:
    AstraColorPublisher()
    : Node("astra_color_publisher")
    {
        astra::initialize();
        
        streamSet_ = std::make_unique<astra::StreamSet>();
        reader_ = std::make_unique<astra::StreamReader>(streamSet_->create_reader());
        
        // 1. Obter o stream de cor a partir do reader
        auto colorStream = reader_->stream<astra::ColorStream>();
        
        // 2. Criar um objeto ImageMode para definir a configuração desejada
        // Nota: Esta configuração deve ser suportada pela sua câmera.
        std::uint32_t Framewidth_ = 640;
        std::uint32_t FrameHeight_ = 480;
        std::uint32_t StreamFPS_ = 30;
        astra_pixel_format_t PixelFormat_ = ASTRA_PIXEL_FORMAT_RGB888;
        astra::ImageStreamMode colorMode = astra::ImageStreamMode(Framewidth_, FrameHeight_, StreamFPS_, PixelFormat_);
        
        // 3. Aplicar o modo desejado ao stream de cor
        colorStream.set_mode(colorMode);
        
        // 4. Iniciar o stream com a nova configuração
        colorStream.start();
        
        reader_->add_listener(*this);
                
        // Timer para manter astra_update rodando
        timer_ = this->create_wall_timer(
            33ms,
            [this]() {
                astra_update();
            }
        );
    }

    ~AstraColorPublisher()
    {
        reader_->remove_listener(*this);
        astra::terminate();
    }

    void init_transport()
    {
        image_transport::ImageTransport it(shared_from_this());
        publisher_ = it.advertise("/astra/color", 10);
    }

    virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override
    {
        this->check_fps();

        const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();
        if (!colorFrame.is_valid())
            return;

        unsigned int width = colorFrame.width();
        unsigned int height = colorFrame.height();

        if (width != lastWidth_ || height != lastHeight_)
        {
            buffer_ = std::make_unique<astra::RgbPixel[]>(colorFrame.length());
            lastWidth_ = width;
            lastHeight_ = height;
        }

        colorFrame.copy_to(buffer_.get());

        // Preencher mensagem ROS
        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = this->now();
        msg.header.frame_id = "astra_color_optical_frame";
        msg.height = height;
        msg.width = width;
        msg.encoding = "rgb8";  // formato RGB888
        msg.is_bigendian = false;
        msg.step = width * 3; // 3 bytes por pixel
        msg.data.resize(width * height * 3);
        std::memcpy(msg.data.data(), buffer_.get(), width * height * 3);

        publisher_.publish(msg);
    }

    void check_fps()
    {
        const float frameWeight = .2f;

        const ClockType::time_point now = ClockType::now();
        const float elapsedMillis = std::chrono::duration_cast<DurationType>(now - prev_).count();

        elapsedMillis_ = elapsedMillis * frameWeight + elapsedMillis_ * (1.f - frameWeight);
        prev_ = now;

        const float fps = 1000.f / elapsedMillis;

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

    std::unique_ptr<astra::RgbPixel[]> buffer_;
    unsigned int lastWidth_ = 0;
    unsigned int lastHeight_ = 0;

    // check fps
    using DurationType = std::chrono::milliseconds;
    using ClockType = std::chrono::high_resolution_clock;

    ClockType::time_point prev_;
    float elapsedMillis_{.0f};
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AstraColorPublisher>();

    node->init_transport();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
