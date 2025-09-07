// ROS 2
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



class AstraColorPublisher : public rclcpp::Node, public astra::FrameListener
{
public:
    AstraColorPublisher(const std::string& node_name)
    : Node(node_name)
    {
        // =================================================
        //     PARAMETERS
        // =================================================

        // Declare camera settings as ros2 parameters
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<int>("fps", 30);
        this->declare_parameter<bool>("check_fps", false);
        
        // Setting the plugins for image transporte publisher
        this->declare_parameter("color.enable_pub_plugins", std::vector<std::string>{"image_transport/compressed"});

        // =================================================
        //     ASTRA SYSTEM
        // =================================================

        // Initialize astra system
        astra::initialize();
        
        // Create a reader from the stream set
        streamSet_ = std::make_unique<astra::StreamSet>();
        reader_ = std::make_unique<astra::StreamReader>(streamSet_->create_reader());
        
        // Create a color stream from the reader
        astra::ColorStream colorStream = reader_->stream<astra::ColorStream>();
        
        // Get parameters values
        std::uint32_t FrameWidth_ = this->get_parameter("width").as_int();
        std::uint32_t FrameHeight_ = this->get_parameter("height").as_int();
        std::uint32_t StreamFPS_ = this->get_parameter("fps").as_int();
        bool check_fps_ = this->get_parameter("check_fps").as_bool();

        // Define stream mode
        astra::ImageStreamMode colorMode = astra::ImageStreamMode(FrameWidth_, FrameHeight_, StreamFPS_, ASTRA_PIXEL_FORMAT_RGB888);
        
        // Apply color mode settings to the stream
        colorStream.set_mode(colorMode);
        
        // Start color stream
        colorStream.start();
        
        // Add this node as a listener to camera info
        reader_->add_listener(*this);

        RCLCPP_INFO(this->get_logger(), "RGB camera stream started!");
          
        // =================================================
        //     TIMER SETTINGS
        // =================================================

        period_ = std::chrono::milliseconds(1000 / StreamFPS_);

        // Keep astra_update callback running
        timer_ = this->create_wall_timer(
            period_,
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

        publisher_ = it.advertise("color", 10, false);
        RCLCPP_INFO(this->get_logger(), "RGB image topics created!");
    }

    virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override
    {
        if (check_fps_) check_fps();

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

        // ROS msg
        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = this->now();
        msg.header.frame_id = "astra_color_optical_frame";
        msg.height = height;
        msg.width = width;
        msg.encoding = sensor_msgs::image_encodings::RGB8;  
        msg.is_bigendian = false;
        msg.step = width * 3; 
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
    // Publishers
    image_transport::Publisher publisher_;

    // Timers
    std::chrono::milliseconds period_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Astra SDK
    std::unique_ptr<astra::StreamSet> streamSet_;
    std::unique_ptr<astra::StreamReader> reader_;

    std::unique_ptr<astra::RgbPixel[]> buffer_;
    unsigned int lastWidth_ = 0;
    unsigned int lastHeight_ = 0;

    // Check fps 
    bool check_fps_;
    using DurationType = std::chrono::milliseconds;
    using ClockType = std::chrono::high_resolution_clock;
    ClockType::time_point prev_;
    float elapsedMillis_{.0f};
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AstraColorPublisher>("color_publisher");
    node->init_transport();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
