#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher() : Node("camera_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&CameraPublisher::publish_frame, this));
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
        }
    }

private:
    void publish_frame() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
