#ifndef IMAGE_SUBSCRIBER_H
#define IMAGE_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <functional>
#include <QImage>

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber();
    void setCallback(std::function<void(const QImage &)> cb);
    void start();

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::function<void(const QImage &)> callback_;
};

#endif // IMAGE_SUBSCRIBER_H
