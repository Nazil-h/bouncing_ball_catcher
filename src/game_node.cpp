#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

struct Ball
{
  cv::Point2d pos;
  cv::Point2d vel;
  int radius;
  cv::Scalar color;
  double timer = 0.0;
};

class GameNode : public rclcpp::Node
{
public:
  GameNode() : Node("game_node")
  {
    auto lower = this->declare_parameter<std::vector<int64_t>>(
      "hsv_lower", std::vector<int64_t>{90, 80, 100});
    auto upper = this->declare_parameter<std::vector<int64_t>>(
      "hsv_upper", std::vector<int64_t>{110, 255, 255});
    min_area_ = this->declare_parameter<int>("min_area", 300);
    num_balls_ = this->declare_parameter<int>("num_balls", 3);
    ball_speed_ = this->declare_parameter<double>("ball_speed", 150.0);
    ball_radius_ = this->declare_parameter<int>("ball_radius", 40);
    hold_duration_ = this->declare_parameter<double>("hold_duration", 2.0);
    round_duration_ = this->declare_parameter<double>("round_duration", 60.0);
    smoothing_alpha_ = this->declare_parameter<double>("smoothing_alpha", 0.5);

    hsv_lower_ = cv::Scalar(lower[0], lower[1], lower[2]);
    hsv_upper_ = cv::Scalar(upper[0], upper[1], upper[2]);

    round_time_left_ = round_duration_;

    kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(msg); });

    pub_score_ = this->create_publisher<std_msgs::msg::Int32>("/game/score", 10);
    pub_debug_ = this->create_publisher<sensor_msgs::msg::Image>("/game/debug_image", 10);

    RCLCPP_INFO(this->get_logger(),
      "game_node jalan. Ronde %.0f detik. Tekan 'r' di jendela gambar untuk main ulang.",
      round_duration_);
  }

private:
  void spawn_balls(int count, int frame_w, int frame_h)
  {
    balls_.clear();
    for (int i = 0; i < count; ++i) {
      Ball b;
      b.radius = ball_radius_;
      b.color = random_color();
      respawn(b, frame_w, frame_h);
      balls_.push_back(b);
    }
  }

  void respawn(Ball & b, int frame_w, int frame_h)
  {
    std::uniform_real_distribution<double> ux(b.radius, frame_w - b.radius);
    std::uniform_real_distribution<double> uy(b.radius, frame_h - b.radius);
    std::uniform_real_distribution<double> uang(0.0, 2 * M_PI);

    double ang = uang(rng_);
    b.pos = cv::Point2d(ux(rng_), uy(rng_));
    b.vel = cv::Point2d(ball_speed_ * std::cos(ang), ball_speed_ * std::sin(ang));
    b.timer = 0.0;
  }

  cv::Scalar random_color()
  {
    std::uniform_int_distribution<int> ucol(80, 255);
    return cv::Scalar(ucol(rng_), ucol(rng_), ucol(rng_));
  }

  void update_balls(double dt, int frame_w, int frame_h)
  {
    for (auto & b : balls_) {
      b.pos += b.vel * dt;

      if (b.pos.x - b.radius < 0) {
        b.pos.x = b.radius;
        b.vel.x = -b.vel.x;
      } else if (b.pos.x + b.radius > frame_w) {
        b.pos.x = frame_w - b.radius;
        b.vel.x = -b.vel.x;
      }

      if (b.pos.y - b.radius < 0) {
        b.pos.y = b.radius;
        b.vel.y = -b.vel.y;
      } else if (b.pos.y + b.radius > frame_h) {
        b.pos.y = frame_h - b.radius;
        b.vel.y = -b.vel.y;
      }
    }
  }

  void update_game(bool detected, cv::Point2d centroid, double dt, int frame_w, int frame_h)
  {
    int chosen = -1;
    double best_dist = 1e18;
    if (detected) {
      for (size_t i = 0; i < balls_.size(); ++i) {
        double d = cv::norm(centroid - balls_[i].pos);
        if (d < balls_[i].radius && d < best_dist) {
          best_dist = d;
          chosen = static_cast<int>(i);
        }
      }
    }

    for (size_t i = 0; i < balls_.size(); ++i) {
      if (static_cast<int>(i) == chosen) {
        balls_[i].timer += dt;
      } else {
        balls_[i].timer = 0.0;
      }
    }

    if (chosen >= 0 && balls_[chosen].timer >= hold_duration_) {
      score_++;
      respawn(balls_[chosen], frame_w, frame_h);
      publish_score();
      RCLCPP_INFO(this->get_logger(), "Skor! Sekarang: %d", score_);
    }
  }

  void reset_game(int frame_w, int frame_h)
  {
    score_ = 0;
    round_time_left_ = round_duration_;
    round_over_ = false;
    spawn_balls(num_balls_, frame_w, frame_h);
    publish_score();
    RCLCPP_INFO(this->get_logger(), "Ronde baru dimulai.");
  }

  void publish_score()
  {
    std_msgs::msg::Int32 msg;
    msg.data = score_;
    pub_score_->publish(msg);
  }

  void publish_debug(const cv::Mat & img, const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::Image out;
    out.header = header;
    out.height = img.rows;
    out.width = img.cols;
    out.encoding = "bgr8";
    out.is_bigendian = false;
    out.step = img.cols * 3;
    out.data.assign(img.data, img.data + img.total() * img.elemSize());
    pub_debug_->publish(out);
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    // 1. Bungkus pesan jadi cv::Mat (tanpa menyalin)
    cv::Mat frame(msg->height, msg->width, CV_8UC3,
                  const_cast<uint8_t *>(msg->data.data()), msg->step);

    // 2. BGR -> HSV, lalu mask
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, hsv_lower_, hsv_upper_, mask);

    // 3. Bersihkan mask
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_);

    // 4. Cari kontur terbesar
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0.0;
    int best_index = -1;
    for (size_t i = 0; i < contours.size(); ++i) {
      double area = cv::contourArea(contours[i]);
      if (area > best_area) {
        best_area = area;
        best_index = static_cast<int>(i);
      }
    }

    // 5. Hitung centroid (dihaluskan) kalau objek cukup besar
    bool detected = false;
    cv::Point2d centroid(0, 0);
    cv::Rect box;
    if (best_index >= 0 && best_area >= min_area_) {
      cv::Moments m = cv::moments(contours[best_index]);
      if (m.m00 > 0) {
        cv::Point2d raw(m.m10 / m.m00, m.m01 / m.m00);
        if (!has_smoothed_) {
          smoothed_centroid_ = raw;
          has_smoothed_ = true;
        } else {
          smoothed_centroid_ =
            raw * smoothing_alpha_ + smoothed_centroid_ * (1.0 - smoothing_alpha_);
        }
        centroid = smoothed_centroid_;
        box = cv::boundingRect(contours[best_index]);
        detected = true;
      }
    } else {
      has_smoothed_ = false;
    }

    // 6. Hitung dt dari timestamp, dibatasi 0..0.2 detik
    rclcpp::Time stamp(msg->header.stamp);
    if (!has_balls_) {
      spawn_balls(num_balls_, frame.cols, frame.rows);
      has_balls_ = true;
    }
    double dt = 0.0;
    if (has_last_stamp_) {
      dt = std::clamp((stamp - last_stamp_).seconds(), 0.0, 0.2);
    }
    last_stamp_ = stamp;
    has_last_stamp_ = true;

    // 7. Ronde berjalan: gerakkan bola, jalankan aturan, kurangi waktu
    if (!round_over_) {
      update_balls(dt, frame.cols, frame.rows);
      update_game(detected, centroid, dt, frame.cols, frame.rows);

      round_time_left_ -= dt;
      if (round_time_left_ <= 0.0) {
        round_time_left_ = 0.0;
        round_over_ = true;
        RCLCPP_INFO(this->get_logger(),
          "Waktu habis! Skor akhir: %d. Tekan 'r' untuk main lagi.", score_);
      }
    } else {
      for (auto & b : balls_) {
        b.timer = 0.0;
      }
    }

    // 8. Gambar bola transparan dan bar progres
    cv::Mat annotated = frame.clone();
    cv::Mat overlay = annotated.clone();
    for (const auto & b : balls_) {
      cv::circle(overlay, b.pos, b.radius, b.color, -1);
    }
    cv::Mat blended;
    cv::addWeighted(overlay, 0.4, annotated, 0.6, 0.0, blended);
    annotated = blended;

    for (const auto & b : balls_) {
      if (b.timer > 0.0) {
        double progress = std::min(b.timer / hold_duration_, 1.0);
        int bar_w = static_cast<int>(2 * b.radius * progress);
        int x = static_cast<int>(b.pos.x - b.radius);
        int y = static_cast<int>(b.pos.y - b.radius - 12);
        cv::rectangle(annotated, cv::Rect(x, y, 2 * b.radius, 6), cv::Scalar(60, 60, 60), -1);
        cv::rectangle(annotated, cv::Rect(x, y, bar_w, 6), cv::Scalar(0, 255, 0), -1);
      }
    }

    // 9. Tandai objek pemain
    if (detected) {
      cv::circle(annotated, cv::Point(centroid.x, centroid.y), 6, cv::Scalar(0, 0, 255), -1);
      cv::rectangle(annotated, box, cv::Scalar(0, 255, 0), 2);
    }

    // 10. Skor (kiri atas) dan sisa waktu (kanan atas)
    cv::putText(annotated, "Score: " + std::to_string(score_),
                cv::Point(10, 35), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 255, 0), 2);

    char time_text[64];
    std::snprintf(time_text, sizeof(time_text), "Waktu: %.1f", round_time_left_);
    cv::putText(annotated, time_text,
                cv::Point(frame.cols - 200, 35), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 255, 0), 2);

    // 11. Layar akhir ronde
    if (round_over_) {
      cv::putText(annotated, "WAKTU HABIS",
                  cv::Point(frame.cols / 2 - 180, frame.rows / 2 - 20),
                  cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 3);
      cv::putText(annotated, "Tekan R untuk main lagi",
                  cv::Point(frame.cols / 2 - 190, frame.rows / 2 + 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    }

    // 12. Publish dan tampilkan
    publish_debug(annotated, msg->header);
    cv::imshow("frame", annotated);
    cv::imshow("mask", mask);

    // 13. Tombol 'r' untuk ronde baru
    int key = cv::waitKey(1);
    if (key == 'r' || key == 'R') {
      reset_game(frame.cols, frame.rows);
    }
  }

  cv::Scalar hsv_lower_;
  cv::Scalar hsv_upper_;
  int min_area_;
  int num_balls_;
  double ball_speed_;
  int ball_radius_;
  double hold_duration_;
  double round_duration_;
  double smoothing_alpha_;
  cv::Mat kernel_;

  std::vector<Ball> balls_;
  bool has_balls_ = false;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_stamp_ = false;
  int score_ = 0;
  double round_time_left_ = 0.0;
  bool round_over_ = false;
  cv::Point2d smoothed_centroid_{0, 0};
  bool has_smoothed_ = false;
  std::mt19937 rng_{std::random_device{}()};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_score_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GameNode>());
  rclcpp::shutdown();
  return 0;
}