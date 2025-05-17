#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode()
    : Node("camera_streamer_cpp"), frame_count_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&CameraNode::timer_callback, this));
        cap_.open(0);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
        }

        start_time_ = std::chrono::steady_clock::now();
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame.");
            return;
        }

        // Publish frame
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
        cv::imshow("Camera Stream", frame);
        cv::waitKey(1);
        RCLCPP_INFO(this->get_logger(), "Published frame.");

        // FPS calculation
        frame_count_++;
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start_time_;
        if (elapsed.count() >= 1.0) {
            RCLCPP_INFO(this->get_logger(), "FPS: %.2f", frame_count_ / elapsed.count());
            frame_count_ = 0;
            start_time_ = now;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    std::chrono::steady_clock::time_point start_time_;
    int frame_count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
