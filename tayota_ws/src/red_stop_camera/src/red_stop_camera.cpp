#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <filesystem>

class RedStopCamera : public rclcpp::Node {
public:
  RedStopCamera() : Node("red_stop_camera") {
    declare_parameter<std::string>(
        "camera_path",
        "/dev/v4l/by-id/usb-046d_Brio_100_2509APEE8428-video-index0");

    declare_parameter<std::string>("save_directory",
                                   "/home/mars/tayota_ws/images");

    camera_path_ = get_parameter("camera_path").as_string();

    save_directory_ = get_parameter("save_directory").as_string();

    std::filesystem::create_directories(save_directory_);

    camera_.open(camera_path_, cv::CAP_V4L2);

    if (!camera_.isOpened()) {
      RCLCPP_FATAL(get_logger(), "Failed to open camera: %s",
                   camera_path_.c_str());

      throw std::runtime_error("Could not open camera");
    }

    camera_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/red_stop_trigger", 10,
        std::bind(&RedStopCamera::triggerCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Camera ready. Waiting for triggers.");
  }

  ~RedStopCamera() {
    if (camera_.isOpened())
      camera_.release();
  }

private:
  void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data)
      return;

    captureImage();
  }

  void captureImage() {
    cv::Mat frame;

    // grab a few frames to avoid stale buffers
    for (int i = 0; i < 3; i++) {
      camera_.grab();
    }

    camera_.retrieve(frame);

    if (frame.empty()) {
      RCLCPP_ERROR(get_logger(), "Captured empty frame");
      return;
    }

    auto now = std::chrono::system_clock::now();

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch())
                  .count();

    std::string filename =
        save_directory_ + "/red_stop_" + std::to_string(ms) + ".jpg";

    if (!cv::imwrite(filename, frame)) {
      RCLCPP_ERROR(get_logger(), "Failed to save image");
      return;
    }

    RCLCPP_INFO(get_logger(), "Saved %s", filename.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;

  cv::VideoCapture camera_;

  std::string camera_path_;
  std::string save_directory_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RedStopCamera>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
