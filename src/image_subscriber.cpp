#include "image_subscriber.h"
#include <opencv2/imgproc.hpp>

ImageSubscriber::ImageSubscriber()
: Node("image_subscriber") {}

void ImageSubscriber::setCallback(std::function<void(const QImage &)> cb) {
    callback_ = std::move(cb);
}

void ImageSubscriber::start() {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10,
        std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1)
    );
}

void ImageSubscriber::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
        auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat rgb;
        cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
        QImage qimg(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888);
        if (callback_) {
            callback_(qimg.copy());
        }
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}
