#include <rclcpp/rclcpp.hpp>
#include "yolo_detect_interfaces/msg/detect_obj_list.hpp"

class ObjectSubscriber : public rclcpp::Node {
public:
    ObjectSubscriber() : Node("object_subscriber") {
        sub_ = this->create_subscription<yolo_detect_interfaces::msg::DetectObjList>(
            "obj/data", 10,
            std::bind(&ObjectSubscriber::callback, this, std::placeholders::_1));
    }

private:
    void callback(const yolo_detect_interfaces::msg::DetectObjList::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Objects detected: %zu", msg->objects.size());
        for (auto &obj : msg->objects) {
            RCLCPP_INFO(this->get_logger(), "  %s at %.2f m", obj.label.c_str(), obj.distance);
        }
    }

    rclcpp::Subscription<yolo_detect_interfaces::msg::DetectObjList>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectSubscriber>());
    rclcpp::shutdown();
    return 0;
}
