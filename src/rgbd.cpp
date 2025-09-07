// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

// Bridge OpenCV-ROS
#include <cv_bridge/cv_bridge.hpp>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Astra SDK
#include <astra/astra.hpp>



class AstraRGBDPublisher : public rclcpp::Node, public astra::FrameListener
{
public:
    AstraRGBDPublisher(const std::string& node_name)
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

        bool check_fps_ = this->get_parameter("check_fps").as_bool();
        
        // Setting the plugins for image transporte publisher
        this->declare_parameter("color.enable_pub_plugins", std::vector<std::string>{"image_transport/compressed"});
        this->declare_parameter("depth.enable_pub_plugins", std::vector<std::string>{"image_transport/compressedDepth"});

        // =================================================
        //     ASTRA SYSTEM
        // =================================================

        // Initialize astra system
        astra::initialize();
        
        // Create a reader from the stream set
        streamSet_ = std::make_unique<astra::StreamSet>();
        reader_ = std::make_unique<astra::StreamReader>(streamSet_->create_reader());
        
        // Get parameters values
        std::uint32_t FrameWidth_ = this->get_parameter("width").as_int();
        std::uint32_t FrameHeight_ = this->get_parameter("height").as_int();
        std::uint32_t StreamFPS_ = this->get_parameter("fps").as_int();

        // ----------------  COLOR STREAM  ----------------

        // Create a color stream from the reader
        astra::ColorStream  colorStream = reader_->stream<astra::ColorStream>();
        
        // Define stream mode
        astra::ImageStreamMode colorMode = astra::ImageStreamMode(FrameWidth_, FrameHeight_, StreamFPS_, ASTRA_PIXEL_FORMAT_RGB888);
        
        // Apply color mode settings to the stream
        colorStream.set_mode(colorMode);

        // ----------------  DEPTH STREAM  ----------------

        // Create a depth stream from the reader
        astra::DepthStream  depthStream = reader_->stream<astra::DepthStream>();  // Depth Stream
        
        // Enable depth align to RGB
        depthStream.enable_registration(true); // IT DOESNT WORK

        // Define stream mode
        astra::ImageStreamMode depthMode = astra::ImageStreamMode(FrameWidth_, FrameHeight_, StreamFPS_, ASTRA_PIXEL_FORMAT_DEPTH_MM);
        
        // Apply depth mode settings to the stream
        depthStream.set_mode(depthMode);
        
        // ----------------  START  ----------------

        // Start depth stream
        depthStream.start();
        RCLCPP_INFO(this->get_logger(), "Depth camera stream started!");

        // Start color stream
        colorStream.start();
        RCLCPP_INFO(this->get_logger(), "RGB camera stream started!");
        
        // Add this node as a listener to camera info
        reader_->add_listener(*this);
                
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

    ~AstraRGBDPublisher()
    {
        reader_->remove_listener(*this);
        astra::terminate();
    }

    void init_transport()
    {
        image_transport::ImageTransport it_color(shared_from_this());
        color_publisher_ = it_color.advertise("color", 10);
        RCLCPP_INFO(this->get_logger(), "RGB image topics created!");
        
        image_transport::ImageTransport it_depth(shared_from_this());
        depth_publisher_ = it_depth.advertise("depth", 10);
        RCLCPP_INFO(this->get_logger(), "Depth image topics created!");
    }

    virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override
    {
        if (check_fps_) check_fps();

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

        // ROS msg
        auto color_msg = sensor_msgs::msg::Image();
        color_msg.header.stamp = now;
        color_msg.header.frame_id = "astra_color_optical_frame";
        color_msg.height = colorHeight;
        color_msg.width = colorWidth;
        color_msg.encoding = sensor_msgs::image_encodings::RGB8;           
        color_msg.is_bigendian = false;
        color_msg.step = colorWidth * 3;      
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

        // ROS msg
        auto depth_msg = sensor_msgs::msg::Image();
        depth_msg.header.stamp = this->now();
        depth_msg.header.frame_id = "astra_depth_optical_frame";
        depth_msg.height = depthHeight;
        depth_msg.width = depthWidth;
        depth_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;       // depth in mm
        depth_msg.is_bigendian = false;
        depth_msg.step = depthWidth * sizeof(uint16_t);
        depth_msg.data.resize(depthWidth * depthHeight * sizeof(uint16_t));
        std::memcpy(depth_msg.data.data(), depthBuffer_.get(), depthWidth * depthHeight * sizeof(uint16_t));

        return depth_msg;
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
    image_transport::Publisher color_publisher_;
    image_transport::Publisher depth_publisher_;

    // Timers
    std::chrono::milliseconds period_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Astra SDK
    std::unique_ptr<astra::StreamSet> streamSet_;
    std::unique_ptr<astra::StreamReader> reader_;

    std::unique_ptr<astra::RgbPixel[]> colorBuffer_;
    unsigned int lastColorWidth_ = 0;
    unsigned int lastColorHeight_ = 0;

    std::unique_ptr<int16_t[]> depthBuffer_;
    unsigned int lastDepthWidth_ = 0;
    unsigned int lastDepthHeight_ = 0;

    rclcpp::Time now_;

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
    auto node = std::make_shared<AstraRGBDPublisher>("rgbd_publisher");
    node->init_transport();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
