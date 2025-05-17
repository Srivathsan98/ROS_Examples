// #include "mainwindow.h"
// #include "./ui_mainwindow.h"

// MainWindow::MainWindow(QWidget *parent)
//     : QMainWindow(parent)
//     , ui(new Ui::MainWindow)
// {
//     ui->setupUi(this);
// }

// MainWindow::~MainWindow()
// {
//     delete ui;
// }

#include "qt_camera_gui/mainwindow.h"
#include "ui_mainwindow.h"

#include <opencv2/imgproc.hpp>
#include <chrono>

using namespace std::chrono_literals;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), is_publishing_(false)
{
    ui->setupUi(this);

    connect(ui->camera_stream, &QPushButton::clicked,
            this, &MainWindow::on_camera_stream_clicked);

    node_ = rclcpp::Node::make_shared("qt_ros_cam_gui");
    pub_ = node_->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
    
    ros_spin_thread_ = std::thread([this]() {
        rclcpp::spin(this->node_);
    });

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow::updateImage);
    timer_->start(30);  // Update at ~30 FPS

    capture_timer_ = new QTimer(this);
    connect(capture_timer_, &QTimer::timeout, this, &MainWindow::captureAndPublish);
}

MainWindow::~MainWindow()
{
    stopCamera();
    rclcpp::shutdown();
    ros_spin_thread_.join();
    delete capture_timer_;
    delete timer_;
    delete ui;
}

void MainWindow::startCamera()
{
    if (!cap_.isOpened()) {
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open camera");
            return;
        }
    }
    is_publishing_ = true;
    capture_timer_->start(33); // ~30 FPS
    ui->camera_stream->setText("Stop Camera");
}

void MainWindow::stopCamera()
{
    if (cap_.isOpened()) {
        capture_timer_->stop();
        cap_.release();
        is_publishing_ = false;
        ui->camera_stream->setText("Start Camera");
    }
}

void MainWindow::captureAndPublish()
{
    if (!cap_.isOpened()) return;

    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Captured empty frame");
        return;
    }

    // Update the display
    QImage img(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_BGR888);
    QMutexLocker locker(&image_mutex_);
    current_image_ = img.copy();

    // Publish the frame
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    pub_->publish(*msg);
}

void MainWindow::on_camera_stream_clicked()
{
    if (!is_publishing_) {
        startCamera();
    } else {
        stopCamera();
    }
}

void MainWindow::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat mat = cv_image->image;

        QImage img(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_BGR888);

        QMutexLocker locker(&image_mutex_);
        current_image_ = img.copy();  // Deep copy for thread safety
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void MainWindow::updateImage()
{
    QMutexLocker locker(&image_mutex_);
    if (!current_image_.isNull()) {
        ui->camera_label->setPixmap(QPixmap::fromImage(current_image_).scaled(
            ui->camera_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}
