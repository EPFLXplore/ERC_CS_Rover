#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class ScienceMicroscope : public rclcpp::Node {
public:
  ScienceMicroscope() : Node("science_microscope"), running_(true) {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();

    this->declare_parameter<std::string>("topic_name", "/SC/microscope");
    this->declare_parameter<int>("jpeg_quality", 80);
    this->declare_parameter<std::string>("frame_id", "camera");
    this->declare_parameter<double>("publish_rate", 30.0);
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<bool>("visualize", false);
    this->declare_parameter<std::string>("window_name", "Science Microscope");
    this->declare_parameter<std::string>(
      "ffmpeg_cmd",
      "ffmpeg -loglevel warning "
      "-fflags nobuffer+discardcorrupt "
      "-flags low_delay "
      "-err_detect ignore_err "
      "-f h264 "
      "-i \"udp://@:5000?fifo_size=1000000&overrun_nonfatal=1\" "
      "-vf scale=640:480 "
      "-f rawvideo -pix_fmt bgr24 pipe:1");

    topic_name_ = this->get_parameter("topic_name").as_string();
    jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    visualize_ = this->get_parameter("visualize").as_bool();
    window_name_ = this->get_parameter("window_name").as_string();
    ffmpeg_cmd_ = this->get_parameter("ffmpeg_cmd").as_string();

    science_publisher_ =
        this->create_publisher<sensor_msgs::msg::CompressedImage>(topic_name_,
                                                                  qos);

    if (visualize_) {
      cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    }

    open_pipe();
    reader_thread_ = std::thread(&ScienceMicroscope::reader_loop, this);

    auto period =
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));

    timer_ = this->create_wall_timer(
        period, std::bind(&ScienceMicroscope::publish_latest_frame, this));
  }

  ~ScienceMicroscope() override {
    running_ = false;

    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }

    if (ffmpeg_pipe_) {
      pclose(ffmpeg_pipe_);
      ffmpeg_pipe_ = nullptr;
    }

    if (visualize_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  void open_pipe() {
    if (!running_ || !rclcpp::ok()) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Opening ffmpeg pipe: %s",
                ffmpeg_cmd_.c_str());

    if (ffmpeg_pipe_) {
      pclose(ffmpeg_pipe_);
      ffmpeg_pipe_ = nullptr;
    }

    ffmpeg_pipe_ = popen(ffmpeg_cmd_.c_str(), "r");

    if (!ffmpeg_pipe_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open ffmpeg pipe");
    }
  }

  bool read_exact(uint8_t *dst, size_t total) {
    size_t got = 0;
    while (got < total && running_ && rclcpp::ok()) {
      size_t n = fread(dst + got, 1, total - got, ffmpeg_pipe_);
      if (n == 0) {
        return false;
      }
      got += n;
    }
    return got == total;
  }

  void reader_loop() {
    const size_t frame_size = static_cast<size_t>(width_) * height_ * 3;
    std::vector<uint8_t> buffer(frame_size);

    while (running_ && rclcpp::ok()) {
      if (!ffmpeg_pipe_) {
        std::this_thread::sleep_for(500ms);
        if (!running_ || !rclcpp::ok()) {
          break;
        }
        open_pipe();
        continue;
      }

      if (!read_exact(buffer.data(), frame_size)) {
        if (!running_ || !rclcpp::ok()) {
          break;
        }

        RCLCPP_WARN(this->get_logger(),
                    "Failed to read full frame from ffmpeg pipe. Reopening...");

        pclose(ffmpeg_pipe_);
        ffmpeg_pipe_ = nullptr;
        std::this_thread::sleep_for(300ms);
        continue;
      }

      cv::Mat frame(height_, width_, CV_8UC3, buffer.data());
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = frame.clone();
      }
    }
  }

  void publish_latest_frame() {
    cv::Mat frame_copy;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (latest_frame_.empty()) {
        return;
      }
      frame_copy = latest_frame_.clone();
    }

    if (visualize_) {
      cv::imshow(window_name_, frame_copy);
      cv::waitKey(1);
    }

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

    if (!cv::imencode(".jpg", frame_copy, buf, params)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to encode frame to JPEG");
      return;
    }

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.format = "jpeg";
    msg.data = std::move(buf);

    science_publisher_->publish(msg);
  }

private:
  std::string topic_name_;
  int jpeg_quality_;
  std::string frame_id_;
  double publish_rate_;
  int width_;
  int height_;
  bool visualize_;
  std::string window_name_;
  std::string ffmpeg_cmd_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      science_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  FILE *ffmpeg_pipe_ = nullptr;
  std::thread reader_thread_;
  std::mutex frame_mutex_;
  cv::Mat latest_frame_;
  std::atomic<bool> running_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScienceMicroscope>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}