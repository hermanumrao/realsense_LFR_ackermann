#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/bool.hpp>

class LineFollower : public rclcpp::Node {
public:
  LineFollower();

private:
  void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);

  void publishCommand(const std::string &cmd);

  bool detectRedStop(const cv::Mat &img);

  bool detectLineContour(const cv::Mat &img, double &steering,
                         double &width_ratio);

  int servoFromError(double error);

private:
  void publishRedTrigger();
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_pub_;

  bool waiting_;

  rclcpp::Time stop_start_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;

  int black_thresh_;

  double kp_;

  int servo_center_;
  int servo_min_;
  int servo_max_;

  int speed_fast_;
  int speed_turn_;

  int contour_min_;

  double stop_time_;
  int min_tape_width_;
  int max_tape_width_;

  double width_center_bonus_;

  int red_low_h_;
  int red_high_h_;

  int red2_low_h_;
  int red2_high_h_;

  int red_s_min_;
  int red_v_min_;

  int red_min_width_;
  int red_max_width_;

  int red_min_area_;

  double red_roi_height_;
  int stop_thresh_;

  int min_black_pixels_;

  double min_fill_ratio_;

  double min_aspect_ratio_;

  double max_aspect_ratio_;

  double max_black_percent_;
  bool red_cooldown_;

  rclcpp::Time red_resume_time_;

  double red_ignore_time_;

  enum class RobotState { FOLLOW_LINE, RECOVERY_REVERSE, STOPPED_NO_LINE };

  RobotState state_;

  rclcpp::Time line_lost_time_;

  rclcpp::Time reverse_start_time_;

  double line_loss_delay_;

  double reverse_timeout_;

  rclcpp::Time last_reverse_step_;

  double reverse_step_time_;

  bool reverse_step_active_;

  rclcpp::Time reverse_step_start_;
};
