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
        
        // Image dimensions
        std::uint32_t Framewidth_ = 640;
        std::uint32_t FrameHeight_ = 480;
        std::uint32_t StreamFPS_ = 30;

        // ----------------  COLOR STREAM  ----------------
        auto colorStream = reader_->stream<astra::ColorStream>();  // Color Stream
        
        astra_pixel_format_t ColorPixelFormat_ = ASTRA_PIXEL_FORMAT_RGB888;  // Color Pixel Format
        astra::ImageStreamMode colorMode = astra::ImageStreamMode(Framewidth_, FrameHeight_, StreamFPS_, ColorPixelFormat_);
        colorStream.set_mode(colorMode);
        
        colorStream.start();
        RCLCPP_INFO(this->get_logger(), "Color Sensor Initialized.");


        // ----------------  DEPTH STREAM  ----------------
        auto DepthStream = reader_->stream<astra::DepthStream>();  // Depth Stream
        
        astra_pixel_format_t DepthPixelFormat_ = ASTRA_PIXEL_FORMAT_DEPTH_MM;  // Depth Pixel Format
        astra::ImageStreamMode depthMode = astra::ImageStreamMode(Framewidth_, FrameHeight_, StreamFPS_, DepthPixelFormat_);
        DepthStream.set_mode(depthMode);
        
        DepthStream.start();
        RCLCPP_INFO(this->get_logger(), "Depth Sensor Initialized.");
        
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
        color_publisher_ = it.advertise("/camera/color", 10);
        depth_publisher_ = it.advertise("/camera/depth", 10);
    }

    virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override
    {
        now_ = this->now();

        auto color_msg = get_color_msg(reader, frame, now_);
        if (color_msg.header.frame_id != "astra_color_optical_frame")
            return;

        auto depth_msg = get_depth_msg(reader, frame, now_);
        if (depth_msg.header.frame_id != "astra_depth_optical_frame")
            return;

        // Publishe color and depth messages
        color_publisher_.publish(color_msg);
        depth_publisher_.publish(depth_msg);
    }


    sensor_msgs::msg::Image get_color_msg(astra::StreamReader& reader, astra::Frame& frame, const rclcpp::Time& now)
    {
        // Color
        const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();

        if (!colorFrame.is_valid())
            return sensor_msgs::msg::Image();

        unsigned int colorWidth = colorFrame.width();
        unsigned int colorHeight = colorFrame.height();

        if (colorWidth != lastColorWidth_ || colorHeight != lastColorHeight_)
        {
            colorBuffer_ = std::make_unique<astra::RgbPixel[]>(colorFrame.length());
            lastColorWidth_ = colorWidth;
            lastColorHeight_ = colorHeight;
        }

        colorFrame.copy_to(colorBuffer_.get());

        // Preencher mensagem ROS
        auto color_msg = sensor_msgs::msg::Image();
        color_msg.header.stamp = now;
        color_msg.header.frame_id = "astra_color_optical_frame";
        color_msg.height = colorHeight;
        color_msg.width = colorWidth;
        color_msg.encoding = "rgb8";           // formato RGB888
        color_msg.is_bigendian = false;
        color_msg.step = colorWidth * 3;       // 3 bytes por pixel
        color_msg.data.resize(colorWidth * colorHeight * 3);
        std::memcpy(color_msg.data.data(), colorBuffer_.get(), colorWidth * colorHeight * 3);

        return color_msg;
    }


    sensor_msgs::msg::Image get_depth_msg(astra::StreamReader& reader, astra::Frame& frame, const rclcpp::Time& now)
    {
        // Depth
        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

        if (!depthFrame.is_valid())
            return sensor_msgs::msg::Image();

        unsigned int depthWidth = depthFrame.width();
        unsigned int depthHeight = depthFrame.height();

        if (depthWidth != lastDepthWidth_ || depthHeight != lastDepthHeight_)
        {
            depthBuffer_ = std::make_unique<int16_t[]>(depthFrame.length());
            lastDepthWidth_ = depthWidth;
            lastDepthHeight_ = depthHeight;
        }

        depthFrame.copy_to(depthBuffer_.get());

        // Preencher mensagem ROS
        auto depth_msg = sensor_msgs::msg::Image();
        depth_msg.header.stamp = this->now();
        depth_msg.header.frame_id = "astra_depth_optical_frame";
        depth_msg.height = depthHeight;
        depth_msg.width = depthWidth;
        depth_msg.encoding = "16UC1";       // profundidade em mm
        depth_msg.is_bigendian = false;
        depth_msg.step = depthWidth * sizeof(uint16_t);
        depth_msg.data.resize(depthWidth * depthHeight * sizeof(uint16_t));
        std::memcpy(depth_msg.data.data(), depthBuffer_.get(), depthWidth * depthHeight * sizeof(uint16_t));

        return depth_msg;
    }
    
private:
    image_transport::Publisher color_publisher_;
    image_transport::Publisher depth_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<astra::StreamSet> streamSet_;
    std::unique_ptr<astra::StreamReader> reader_;

    std::unique_ptr<astra::RgbPixel[]> colorBuffer_;
    unsigned int lastColorWidth_ = 0;
    unsigned int lastColorHeight_ = 0;

    std::unique_ptr<int16_t[]> depthBuffer_;
    unsigned int lastDepthWidth_ = 0;
    unsigned int lastDepthHeight_ = 0;

    rclcpp::Time now_;
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
