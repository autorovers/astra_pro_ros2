// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

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

        this->declare_parameter<bool>("image_compressed_only", false);
        this->declare_parameter<bool>("image_raw_only", false);
        this->declare_parameter<bool>("compressed_and_raw", true);

        this->declare_parameter<bool>("color", true);
        this->declare_parameter<bool>("depth", true);
        this->declare_parameter<bool>("infrared", true);

        bool image_compressed_only_ = this->get_parameter("image_compressed_only").as_bool();  // Activate only compressed topic
        bool image_raw_only_ = this->get_parameter("image_raw_only").as_bool();                // Activate only compressed topic
        bool compressed_and_raw_ = this->get_parameter("compressed_and_raw").as_bool();         // Activate only compressed topic

        color_on_ = this->get_parameter("color").as_bool();             // Activate RGB sensor
        depth_on_ = this->get_parameter("depth").as_bool();             // Activate Depth stream
        infrared_on_ = this->get_parameter("infrared").as_bool();       // Activate Infrared stream

        // Setting the plugins for image transporte publisher
        if (image_compressed_only_) 
        {
            if (infrared_on_)
                this->declare_parameter("infrared.enable_pub_plugins", std::vector<std::string>{"image_transport/compressed"});
            if (depth_on_)
                this->declare_parameter("depth.enable_pub_plugins", std::vector<std::string>{"image_transport/compressedDepth"});
            if (color_on_)
                this->declare_parameter("color.enable_pub_plugins", std::vector<std::string>{"image_transport/compressed"});
        }
        else if (image_raw_only_)
        {
            if (infrared_on_)
                this->declare_parameter("infrared.enable_pub_plugins", std::vector<std::string>{"image_transport/raw"});
            if (depth_on_)
                this->declare_parameter("depth.enable_pub_plugins", std::vector<std::string>{"image_transport/raw"});
            if (color_on_)
                this->declare_parameter("color.enable_pub_plugins", std::vector<std::string>{"image_transport/raw"});
        }
        else if (compressed_and_raw_)
        {
            if (infrared_on_)
                this->declare_parameter("infrared.enable_pub_plugins", std::vector<std::string>{"image_transport/compressed", "image_transport/raw"});
            if (depth_on_)
                this->declare_parameter("depth.enable_pub_plugins", std::vector<std::string>{"image_transport/compressedDepth", "image_transport/raw"});
            if (color_on_)
                this->declare_parameter("color.enable_pub_plugins", std::vector<std::string>{"image_transport/compressed", "image_transport/raw"});
        }
        
        // Setting the plugins for image transporte publisher

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
        StreamFPS_ = this->get_parameter("fps").as_int();

        // ----------------  COLOR STREAM  ----------------

        // Create a color stream from the reader
        astra::ColorStream  colorStream = reader_->stream<astra::ColorStream>();
        
        // Define color stream mode
        astra::ImageStreamMode colorMode = astra::ImageStreamMode(FrameWidth_, FrameHeight_, StreamFPS_, ASTRA_PIXEL_FORMAT_RGB888);
        
        // Apply color mode settings to the stream
        colorStream.set_mode(colorMode);

        // ----------------  DEPTH STREAM  ----------------

        // Create a depth stream from the reader
        astra::DepthStream  depthStream = reader_->stream<astra::DepthStream>();  // Depth Stream

        // Define depth stream mode
        astra::ImageStreamMode depthMode = astra::ImageStreamMode(FrameWidth_, FrameHeight_, StreamFPS_, ASTRA_PIXEL_FORMAT_DEPTH_MM);
        
        // Apply depth mode settings to the stream
        depthStream.set_mode(depthMode);

        // ----------------  Infrared STREAM  ----------------

        // Create a infrared stream from the reader
        astra::InfraredStream  infraredStream = reader_->stream<astra::InfraredStream>();  // Infrared Stream

        // Define infrared stream mode
        astra::ImageStreamMode infraredMode = astra::ImageStreamMode(FrameWidth_, FrameHeight_, StreamFPS_, ASTRA_PIXEL_FORMAT_RGB888);
        
        // Apply infrared mode settings to the stream
        infraredStream.set_mode(infraredMode);
        
        // ----------------  START  ----------------

        if (infrared_on_)
        {
            // Start infrared stream
            infraredStream.start();
            RCLCPP_INFO(this->get_logger(), "Infrared stream started!");
        }

        if (depth_on_)
        {
            // Start depth stream
            depthStream.start();
            RCLCPP_INFO(this->get_logger(), "Depth stream started!");
        }

        if (color_on_)
        {
            // Start color stream
            colorStream.start();
            RCLCPP_INFO(this->get_logger(), "RGB stream started!");
        }
    
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
        if (color_on_)
        {
            image_transport::ImageTransport it_color(shared_from_this());
            color_publisher_ = it_color.advertise("color", 10);
            RCLCPP_INFO(this->get_logger(), "RGB image topics created!");
        }
    
        if (depth_on_)
        {
            image_transport::ImageTransport it_depth(shared_from_this());
            depth_publisher_ = it_depth.advertise("depth", 10);
            RCLCPP_INFO(this->get_logger(), "Depth image topics created!");
        }

        if (infrared_on_)
        {
            image_transport::ImageTransport it_infrared(shared_from_this());
            infrared_publisher_ = it_infrared.advertise("infrared", 10);
            RCLCPP_INFO(this->get_logger(), "Infrared image topics created!");
        }
    }
    
private:

    // =================================================
    //     ATTRIBUTES
    // =================================================

    // Publishers
    image_transport::Publisher color_publisher_;
    image_transport::Publisher depth_publisher_;
    image_transport::Publisher infrared_publisher_;

    // Timers
    std::chrono::milliseconds period_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Astra SDK
    std::unique_ptr<astra::StreamSet> streamSet_;
    std::unique_ptr<astra::StreamReader> reader_;

    bool color_on_;
    std::unique_ptr<astra::RgbPixel[]> colorBuffer_;
    unsigned int lastColorWidth_ = 0;
    unsigned int lastColorHeight_ = 0;

    bool depth_on_;
    std::unique_ptr<int16_t[]> depthBuffer_;
    unsigned int lastDepthWidth_ = 0;
    unsigned int lastDepthHeight_ = 0;

    bool infrared_on_;
    std::unique_ptr<astra::RgbPixel[]> infraredBuffer_;
    unsigned int lastInfraredWidth_ = 0;
    unsigned int lastInfraredHeight_ = 0;

    rclcpp::Time now_;

    // Check fps 
    using DurationType = std::chrono::milliseconds;
    using ClockType = std::chrono::high_resolution_clock;

    ClockType::time_point prev_;
    float elapsedMillis_{.0f};
    std::uint32_t StreamFPS_;
    bool fps_ok = false;


    // =================================================
    //     METHODS
    // =================================================


    virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override
    {
        if (!fps_ok) check_fps();

        now_ = this->now();

        if (color_on_)
        {
            auto color_msg = get_color_msg(frame, now_);                         // convert color image frame to ros message 
            if (color_msg.header.frame_id != "astra_color_optical_frame")        // verify if frame is not empty
                return;
            color_publisher_.publish(color_msg);                                 // publish color message
        } 


        if (depth_on_)
        { 
            auto depth_msg = get_depth_msg(frame, now_);                         // convert depth image frame to ros message 
            if (depth_msg.header.frame_id != "astra_infrared_optical_frame")     // verify if frame is not empty
                return;
            depth_publisher_.publish(depth_msg);                                 // publish depth message
        }

        if (infrared_on_)
        {
            auto infrared_msg = get_infrared_msg(frame, now_);                   // convert infrared image frame to ros message 
            if (infrared_msg.header.frame_id != "astra_infrared_optical_frame")  // verify if frame is not empty
                return;
            infrared_publisher_.publish(infrared_msg);                           // publish infrared message
        }
    }


    sensor_msgs::msg::Image get_color_msg(astra::Frame& frame, const rclcpp::Time& now)
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
        // std::memcpy(msg.data.data(), colorBuffer_.get(), colorWidth * colorHeight * 3); Uncomment to unflip
        
        // Flip horizontally:
        std::vector<uint8_t> temp_line(colorWidth * 3);
        for (unsigned int y = 0; y < colorHeight; ++y)
        {
            uint8_t* row_src = reinterpret_cast<uint8_t*>(colorBuffer_.get()) + y * colorWidth * 3;
            uint8_t* row_dst = color_msg.data.data() + y * colorWidth * 3;

            for (unsigned int x = 0; x < colorWidth; ++x)
            {
                unsigned int src_idx = x * 3;
                unsigned int dst_idx = (colorWidth - 1 - x) * 3;
                row_dst[dst_idx + 0] = row_src[src_idx + 0]; // R
                row_dst[dst_idx + 1] = row_src[src_idx + 1]; // G
                row_dst[dst_idx + 2] = row_src[src_idx + 2]; // B
            }
        }

        return color_msg;
    }


    sensor_msgs::msg::Image get_depth_msg(astra::Frame& frame, const rclcpp::Time& now)
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
        depth_msg.header.frame_id = "astra_infrared_optical_frame";
        depth_msg.height = depthHeight;
        depth_msg.width = depthWidth;
        depth_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;       // depth in mm
        depth_msg.is_bigendian = false;
        depth_msg.step = depthWidth * sizeof(uint16_t);
        depth_msg.data.resize(depthWidth * depthHeight * sizeof(uint16_t));
        // std::memcpy(depth_msg.data.data(), depthBuffer_.get(), depthWidth * depthHeight * sizeof(uint16_t)); // Uncomment to unflip

        // Flip horizontally
        std::vector<uint8_t> temp_line(depthWidth * 3);
        for (unsigned int y = 0; y < depthHeight; ++y)
        {
            uint8_t* row_src = reinterpret_cast<uint8_t*>(depthBuffer_.get()) + y * depthWidth * 3;
            uint8_t* row_dst = depth_msg.data.data() + y * depthWidth * 3;

            for (unsigned int x = 0; x < depthWidth; ++x)
            {
                unsigned int src_idx = x * 3;
                unsigned int dst_idx = (depthWidth - 1 - x) * 3;
                row_dst[dst_idx + 0] = row_src[src_idx + 0]; // R
                row_dst[dst_idx + 1] = row_src[src_idx + 1]; // G
                row_dst[dst_idx + 2] = row_src[src_idx + 2]; // B
            }
        }

        return depth_msg;
    }


    sensor_msgs::msg::Image get_infrared_msg(astra::Frame& frame, const rclcpp::Time& now)
    {
        // Infrared
        const astra::InfraredFrameRgb infraredFrame = frame.get<astra::InfraredFrameRgb>();
        if (!infraredFrame.is_valid())
            return sensor_msgs::msg::Image();

        unsigned int infraredWidth = infraredFrame.width();
        unsigned int infraredHeight = infraredFrame.height();

        if (infraredWidth != lastInfraredWidth_ || infraredHeight != lastInfraredHeight_)
        {
            infraredBuffer_ = std::make_unique<astra::RgbPixel[]>(infraredFrame.length());
            lastInfraredWidth_ = infraredWidth;
            lastInfraredHeight_ = infraredHeight;
        }

        infraredFrame.copy_to(infraredBuffer_.get());

        // ROS msg
        auto infrared_msg = sensor_msgs::msg::Image();
        infrared_msg.header.stamp = this->now();
        infrared_msg.header.frame_id = "astra_infrared_optical_frame";
        infrared_msg.height = infraredHeight;
        infrared_msg.width = infraredWidth;
        infrared_msg.encoding = sensor_msgs::image_encodings::RGB8;  
        infrared_msg.is_bigendian = false;
        infrared_msg.step = infraredWidth * 3; 
        infrared_msg.data.resize(infraredWidth * infraredHeight * 3);
        // std::memcpy(infrared_msg.data.data(), infraredBuffer_.get(), infraredWidth * infraredHeight * 3); // Uncomment to unflip

        // Flip horizontally
        std::vector<uint8_t> temp_line(infraredWidth * 3);
        for (unsigned int y = 0; y < infraredHeight; ++y)
        {
            uint8_t* row_src = reinterpret_cast<uint8_t*>(infraredBuffer_.get()) + y * infraredWidth * 3;
            uint8_t* row_dst = infrared_msg.data.data() + y * infraredWidth * 3;

            for (unsigned int x = 0; x < infraredWidth; ++x)
            {
                unsigned int src_idx = x * 3;
                unsigned int dst_idx = (infraredWidth - 1 - x) * 3;
                row_dst[dst_idx + 0] = row_src[src_idx + 0]; // R
                row_dst[dst_idx + 1] = row_src[src_idx + 1]; // G
                row_dst[dst_idx + 2] = row_src[src_idx + 2]; // B
            }
        }

        return infrared_msg;
    }

    virtual void check_fps()
    {
        const float frameWeight = .2f;

        const ClockType::time_point now = ClockType::now();
        const float elapsedMillis = std::chrono::duration_cast<DurationType>(now - prev_).count();

        elapsedMillis_ = elapsedMillis * frameWeight + elapsedMillis_ * (1.f - frameWeight);
        prev_ = now;

        const std::uint32_t fps = 1000.f / elapsedMillis_;
        const auto precision = std::cout.precision();

        
        if (fps == StreamFPS_) 
        {
            fps_ok = true;
            RCLCPP_INFO(this->get_logger(), "\nFPS stabilized: %d FPS", fps);
        }
        else  
        {
            fps_ok = false;
            RCLCPP_INFO(this->get_logger(), "Stabilizing camera FPS...");
        }
    }
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
