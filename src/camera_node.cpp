#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
  CameraNode() : Node("camera_node")
  {
    source_ = this->declare_parameter<std::string>("source", "0");
    fps_ = this->declare_parameter<double>("fps", 30.0); //jgn diganti
    width_ = this->declare_parameter<int>("width", 640);
    height_ = this->declare_parameter<int>("height", 480);

    bool is_webcam = !source_.empty() &&
      std::all_of(source_.begin(), source_.end(),
        [](unsigned char c) { return std::isdigit(c); });

    if (is_webcam) {
      cap_.open(std::stoi(source_));
      is_file_ = false;
    } else {
      cap_.open(source_);
      is_file_ = true;
    }

    if (!cap_.isOpened()) {
      throw std::runtime_error("Gagal membuka sumber: " + source_);
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FPS, fps_);

    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);

    auto period = std::chrono::duration<double>(1.0 / fps_);
    timer_ = this->create_wall_timer(period, [this]() { on_timer(); });

    RCLCPP_INFO(this->get_logger(), "camera_node jalan, source=%s", source_.c_str());
  }

private:
  void on_timer()
  {
    cv::Mat frame;

    if (!cap_.read(frame) || frame.empty()) {
      if (is_file_) {
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
        if (!cap_.read(frame) || frame.empty()) {
          return;
        }
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Gagal membaca frame dari webcam");
        return;
      }
    }
    
    cv::flip(frame, frame, 1);  // 1 = biar ga mirror
    
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera";

    cv::Mat f = frame.isContinuous() ? frame : frame.clone();

    sensor_msgs::msg::Image msg;
    msg.header = header;
    msg.height = f.rows;
    msg.width = f.cols;
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = f.cols * 3;
    msg.data.assign(f.data, f.data + f.total() * f.elemSize());

    pub_->publish(msg);
  }

  std::string source_;
  double fps_;
  int width_;
  int height_;
  bool is_file_ = false;

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}