#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include "yolo_detect_interfaces/msg/detect_obj_list.hpp"
#include "yolo_detect_interfaces/msg/detect_obj.hpp"

using namespace cv;
using namespace std;

// --- Camera parameters (adjust as needed) ---
const float CAMERA_FOV_DEG = 62.0; // Raspberry Pi cam ~62°, USB cams ~60°
int IMAGE_WIDTH = 640;             // will update dynamically

// Function to estimate distance (rough approximation)
float estimateDistance(int bbox_width_pixels)
{
    float fov_rad = CAMERA_FOV_DEG * CV_PI / 180.0;
    float fraction = (float)bbox_width_pixels / (float)IMAGE_WIDTH;

    if (fraction <= 0.0f) return -1.0f;

    // Assume average object ~0.5m wide (tune this per object if needed)
    float assumed_object_width_m = 0.5;
    float distance = assumed_object_width_m / (2.0f * tan(fov_rad / 2.0f) * fraction);
    return distance;
}

class ObjectDetectionNode : public rclcpp::Node
{
public:
    ObjectDetectionNode() : Node("object_detection")
    {
        // Sub and Pub
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&ObjectDetectionNode::image_callback, this, std::placeholders::_1));
        obj_pub_ = this->create_publisher<yolo_detect_interfaces::msg::DetectObjList>("obj/data", 10);

        // Get model path
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("yolo_vision");
        std::string model_path = std::filesystem::path(package_share_dir) / "models" / "yolov8n.onnx";

        if (!std::filesystem::exists(model_path))
        {
            RCLCPP_ERROR(this->get_logger(), "Model file not found at: %s", model_path.c_str());
            RCLCPP_ERROR(this->get_logger(), "Please place yolov8n.onnx in the models directory");
            throw std::runtime_error("Model file not found");
        }

        // ONNX setup
        session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), Ort::SessionOptions{});
        allocator_ = std::make_unique<Ort::AllocatorWithDefaultOptions>();

        // Get input and output names properly
        auto input_name_allocated = session_->GetInputNameAllocated(0, *allocator_);
        auto output_name_allocated = session_->GetOutputNameAllocated(0, *allocator_);

        input_name_ = std::string(input_name_allocated.get());
        output_name_ = std::string(output_name_allocated.get());

        RCLCPP_INFO(this->get_logger(), "YOLOv8 model loaded from: %s", model_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Input name: %s", input_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output name: %s", output_name_.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        IMAGE_WIDTH = frame.cols; // update each frame

        // -------- Preprocess --------
        cv::Mat resized;

        // Resize + normalize
        cv::resize(frame, resized, cv::Size(640, 640));
        resized.convertTo(resized, CV_32F, 1.0 / 255.0);

        // HWC -> CHW
        std::vector<float> input_tensor_values;
        input_tensor_values.reserve(3 * 640 * 640);
        for (int c = 0; c < 3; c++)
        {
            for (int y = 0; y < 640; y++)
            {
                for (int x = 0; x < 640; x++)
                {
                    input_tensor_values.push_back(resized.at<cv::Vec3f>(y, x)[c]);
                }
            }
        }

        std::array<int64_t, 4> input_shape = {1, 3, 640, 640}; // NCHW
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            allocator_->GetInfo(), input_tensor_values.data(), input_tensor_values.size(),
            input_shape.data(), input_shape.size());

        // -------- Inference --------
        const char *input_names[] = {input_name_.c_str()};
        const char *output_names[] = {output_name_.c_str()};

        auto output_tensors = session_->Run(
            Ort::RunOptions{nullptr},
            input_names, &input_tensor, 1,
            output_names, 1);

        float *output_data = output_tensors.front().GetTensorMutableData<float>();

        // YOLOv8 output format: [1, 84, 8400] (84 = 4 box coords + 80 classes)
        auto out_shape = output_tensors.front().GetTensorTypeAndShapeInfo().GetShape();
        int num_classes = 80;
        int num_boxes = out_shape[2]; // 8400
        int stride = num_classes + 4; // 84

        yolo_detect_interfaces::msg::DetectObjList objs_msg;
        objs_msg.header = msg->header;

        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;
        std::vector<int> class_ids;
        std::vector<yolo_detect_interfaces::msg::DetectObj> detections;

        // Loop over all predictions
        for (int i = 0; i < num_boxes; i++)
        {
            float cx = output_data[0 * num_boxes + i];
            float cy = output_data[1 * num_boxes + i];
            float w = output_data[2 * num_boxes + i];
            float h = output_data[3 * num_boxes + i];

            // Get best class confidence
            float max_conf = 0.0f;
            int class_id = -1;
            for (int c = 0; c < num_classes; c++)
            {
                float score = output_data[(4 + c) * num_boxes + i];
                if (score > max_conf)
                {
                    max_conf = score;
                    class_id = c;
                }
            }

            if (max_conf < confThreshold)
                continue;

            // Scale boxes back to original frame size
            int x = static_cast<int>((cx - w / 2) * frame.cols / 640.0f);
            int y = static_cast<int>((cy - h / 2) * frame.rows / 640.0f);
            int width = static_cast<int>(w * frame.cols / 640.0f);
            int height = static_cast<int>(h * frame.rows / 640.0f);

            cv::Rect box(x, y, width, height);

            // Clip box
            box &= cv::Rect(0, 0, frame.cols, frame.rows);

            boxes.push_back(box);
            confidences.push_back(max_conf);
            class_ids.push_back(class_id);

            // Fill custom ROS2 message
            yolo_detect_interfaces::msg::DetectObj obj;
            obj.label = std::to_string(class_id);
            obj.x = box.x;
            obj.y = box.y;
            obj.w = box.width;
            obj.h = box.height;
            obj.distance = estimateDistance(box.width); // TODO: replace with actual estimation
            detections.push_back(obj);
        }

        // Apply Non-Maximum Suppression
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

        // COCO class labels (shortened, add all 80)
        std::vector<std::string> class_names = {
            "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light",
            "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
            "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
            "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa",
            "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard",
            "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
            "teddy bear", "hair drier", "toothbrush"};

        // Draw final detections
        for (int idx : indices)
        {
            cv::Rect box = boxes[idx];
            int cls = class_ids[idx];
            float conf = confidences[idx];

            objs_msg.objects.push_back(detections[idx]);

            // Unique color per class
            cv::Scalar color((37 * cls) % 255, (17 * cls) % 255, (29 * cls) % 255);

            // Draw bounding box
            cv::rectangle(frame, box, color, 2);

            // Label text
            std::string label = class_names[cls] + cv::format(" %.2f", conf);
            //  if (obj.distance > 0)
            //     label << " - " << std::fixed << std::setprecision(2) << obj.distance << "m";
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::rectangle(frame, cv::Rect(cv::Point(box.x, box.y - labelSize.height - baseLine), cv::Size(labelSize.width, labelSize.height + baseLine)),
                          color, cv::FILLED);
            cv::putText(frame, label, cv::Point(box.x, box.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }

        // Publish results
        obj_pub_->publish(objs_msg);
        cv::resize(frame, frame, cv::Size(800, 600));

        cv::imshow("Detections", frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<yolo_detect_interfaces::msg::DetectObjList>::SharedPtr obj_pub_;

    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "yolov8"};
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::AllocatorWithDefaultOptions> allocator_;
    std::string input_name_;
    std::string output_name_;
    float confThreshold = 0.3; // discard low-confidence boxes
    float nmsThreshold = 0.45; // IOU threshold for NMS
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
