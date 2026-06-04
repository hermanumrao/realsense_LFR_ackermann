#include "realsense_line_follower/line_follower.hpp"

LineFollower::LineFollower() : Node("line_follower") {
  declare_parameter("black_threshold", 75);

  declare_parameter("kp", 0.15);

  declare_parameter("servo_center", 90);
  declare_parameter("servo_min", 0);
  declare_parameter("servo_max", 180);

  declare_parameter("speed_fast", 120);
  declare_parameter("speed_turn", 60);

  declare_parameter("contour_min_area", 1200);

  declare_parameter("stop_time", 5.0);

  declare_parameter("stop_pixel_thresh", 2500);

  declare_parameter("min_tape_width", 80);

  declare_parameter("max_tape_width", 600);

  declare_parameter("width_center_bonus", 0.5);

  declare_parameter("red_low_h", 0);
  declare_parameter("red_high_h", 10);

  declare_parameter("red2_low_h", 170);
  declare_parameter("red2_high_h", 180);

  declare_parameter("red_s_min", 100);
  declare_parameter("red_v_min", 100);

  declare_parameter("red_min_width", 120);
  declare_parameter("red_max_width", 700);

  declare_parameter("red_min_area", 2500);

  declare_parameter("red_roi_height", 0.20);

  declare_parameter("min_black_pixels", 7000);

  declare_parameter("min_fill_ratio", 0.35);

  declare_parameter("max_black_percent", 0.65);
  declare_parameter("red_ignore_time", 2.0);

  declare_parameter("line_loss_delay", 1.0);

  declare_parameter("reverse_timeout", 4.0);

  declare_parameter("reverse_step_time", 0.5);

  get_parameter("reverse_step_time", reverse_step_time_);

  last_reverse_step_ = now();

  get_parameter("line_loss_delay", line_loss_delay_);

  get_parameter("reverse_timeout", reverse_timeout_);

  get_parameter("red_ignore_time", red_ignore_time_);

  get_parameter("min_black_pixels", min_black_pixels_);

  get_parameter("min_fill_ratio", min_fill_ratio_);

  get_parameter("max_black_percent", max_black_percent_);

  get_parameter("red_low_h", red_low_h_);
  get_parameter("red_high_h", red_high_h_);

  get_parameter("red2_low_h", red2_low_h_);
  get_parameter("red2_high_h", red2_high_h_);

  get_parameter("red_s_min", red_s_min_);
  get_parameter("red_v_min", red_v_min_);

  get_parameter("red_min_width", red_min_width_);
  get_parameter("red_max_width", red_max_width_);

  get_parameter("red_min_area", red_min_area_);

  get_parameter("red_roi_height", red_roi_height_);

  get_parameter("min_tape_width", min_tape_width_);

  get_parameter("max_tape_width", max_tape_width_);

  get_parameter("width_center_bonus", width_center_bonus_);

  get_parameter("black_threshold", black_thresh_);

  get_parameter("kp", kp_);

  get_parameter("servo_center", servo_center_);

  get_parameter("servo_min", servo_min_);

  get_parameter("servo_max", servo_max_);

  get_parameter("speed_fast", speed_fast_);

  get_parameter("speed_turn", speed_turn_);

  get_parameter("contour_min_area", contour_min_);

  get_parameter("stop_time", stop_time_);

  get_parameter("stop_pixel_thresh", stop_thresh_);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/color/image_raw", 10,
      std::bind(&LineFollower::imageCallback, this, std::placeholders::_1));

  motor_pub_ = create_publisher<std_msgs::msg::String>("/motor_command", 10);
  red_cooldown_ = false;

  trigger_pub_ = create_publisher<std_msgs::msg::Bool>("/red_stop_trigger", 10);
  waiting_ = false;
  state_ = RobotState::FOLLOW_LINE;

  line_lost_time_ = now();

  reverse_start_time_ = now();

  reverse_step_active_ = false;
}

void LineFollower::publishRedTrigger() {
  std_msgs::msg::Bool msg;

  msg.data = true;

  trigger_pub_->publish(msg);

  RCLCPP_WARN(get_logger(),

              "[RED] Published trigger");
}

void LineFollower::publishCommand(const std::string &s) {
  std_msgs::msg::String msg;

  msg.data = s;

  motor_pub_->publish(msg);
}

bool LineFollower::detectRedStop(const cv::Mat &img) {
  int y = img.rows * (1 - red_roi_height_);

  cv::Rect roi(0, y, img.cols, img.rows - y);

  cv::Mat crop = img(roi);

  cv::Mat hsv;

  cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);

  cv::Mat m1, m2;

  cv::inRange(hsv,

              cv::Scalar(red_low_h_, red_s_min_, red_v_min_),

              cv::Scalar(red_high_h_, 255, 255),

              m1);

  cv::inRange(hsv,

              cv::Scalar(red2_low_h_, red_s_min_, red_v_min_),

              cv::Scalar(red2_high_h_, 255, 255),

              m2);

  cv::Mat mask = m1 | m2;

  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,

                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));

  std::vector<std::vector<cv::Point>> contours;

  cv::findContours(mask, contours,

                   cv::RETR_EXTERNAL,

                   cv::CHAIN_APPROX_SIMPLE);

  double largest = 0;

  int idx = -1;

  for (size_t i = 0; i < contours.size(); i++) {
    double a = contourArea(contours[i]);

    if (a > largest) {
      largest = a;

      idx = i;
    }
  }

  if (idx < 0)
    return false;

  cv::Rect box = boundingRect(contours[idx]);

  if (largest < red_min_area_) {
    return false;
  }

  if (box.width < red_min_width_) {
    return false;
  }

  if (box.width > red_max_width_) {
    return false;
  }

  return true;
}

bool LineFollower::detectLineContour(const cv::Mat &img, double &steer,
                                     double &width_ratio) {
  cv::Rect roi(0, img.rows * 0.55, img.cols, img.rows * 0.45);

  cv::Mat crop = img(roi);

  cv::Mat gray;

  cv::cvtColor(crop, gray, cv::COLOR_BGR2GRAY);

  cv::Mat bin;

  cv::threshold(gray, bin, black_thresh_, 255, cv::THRESH_BINARY_INV);

  cv::morphologyEx(bin, bin, cv::MORPH_OPEN,

                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

  cv::morphologyEx(bin, bin, cv::MORPH_CLOSE,

                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));

  double black_ratio = (double)cv::countNonZero(bin) / bin.total();

  RCLCPP_INFO(get_logger(), "[LINE] black ratio=%.3f", black_ratio);

  if (black_ratio < 0.01) {
    RCLCPP_WARN(get_logger(), "[LINE] No black pixels");

    return false;
  }

  if (black_ratio > max_black_percent_) {
    RCLCPP_WARN(get_logger(), "[LINE] Too much black");

    return false;
  }

  std::vector<std::vector<cv::Point>> contours;

  cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  RCLCPP_INFO(get_logger(), "[LINE] contours=%ld", contours.size());

  if (contours.empty()) {
    RCLCPP_WARN(get_logger(), "[LINE] no contours");

    return false;
  }

  double best = 0;

  int idx = -1;

  for (size_t i = 0; i < contours.size(); i++) {
    double a = cv::contourArea(contours[i]);

    if (a > best) {
      best = a;

      idx = i;
    }
  }

  auto contour = contours[idx];

  cv::Rect box = cv::boundingRect(contour);

  double area = cv::contourArea(contour);

  double fill = area / (box.width * box.height);

  double aspect = (double)box.width / std::max(1, box.height);

  RCLCPP_INFO(get_logger(),

              "[LINE] area=%.0f width=%d fill=%.2f aspect=%.2f",

              area,

              box.width,

              fill,

              aspect);

  if (area < contour_min_) {
    RCLCPP_WARN(get_logger(), "[LINE] contour too small");

    return false;
  }

  if (fill < min_fill_ratio_) {
    RCLCPP_WARN(get_logger(), "[LINE] fill reject");

    return false;
  }

  if (box.width < min_tape_width_) {
    RCLCPP_WARN(get_logger(), "[LINE] tape width too small");

    return false;
  }

  width_ratio = (double)box.width / crop.cols;

  cv::Moments m = cv::moments(contour);

  double cx = m.m10 / m.m00;

  double err = cx - crop.cols / 2;

  cv::Vec4f line;

  cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);

  double vx = line[0];

  steer = (err + 100 * vx) * (1 - width_center_bonus_ * width_ratio);

  RCLCPP_INFO(get_logger(),

              "[LINE] FOUND cx=%.1f steer=%.1f width=%.2f",

              cx,

              steer,

              width_ratio);

  return true;
}

int LineFollower::servoFromError(double e) {
  int s = servo_center_ - kp_ * e;

  return std::max(servo_min_, std::min(servo_max_, s));
}

void LineFollower::imageCallback(sensor_msgs::msg::Image::SharedPtr msg) {
  auto img = cv_bridge::toCvCopy(msg, "bgr8")->image;

  //--------------------------------------------------
  // RED WAIT STATE
  //--------------------------------------------------

  if (waiting_) {
    publishCommand("STOP");

    double elapsed = (now() - stop_start_).seconds();

    if (elapsed >= stop_time_) {
      waiting_ = false;

      red_cooldown_ = true;

      red_resume_time_ = now();

      RCLCPP_INFO(get_logger(), "[RED] Resume -> ignoring red");
    }

    return;
  }

  //--------------------------------------------------
  // RED COOLDOWN
  //--------------------------------------------------

  if (red_cooldown_) {
    double dt = (now() - red_resume_time_).seconds();

    if (dt > red_ignore_time_) {
      red_cooldown_ = false;

      RCLCPP_INFO(get_logger(), "[RED] Red enabled");
    }
  }

  //--------------------------------------------------
  // RECOVERY ACTIVE STEP
  //--------------------------------------------------

  if (state_ == RobotState::RECOVERY_REVERSE && reverse_step_active_) {
    double total_dt = (now() - reverse_start_time_).seconds();

    if (total_dt > reverse_timeout_) {
      state_ = RobotState::STOPPED_NO_LINE;

      publishCommand("STOP");

      RCLCPP_WARN(get_logger(), "[RECOVERY] timeout");

      return;
    }

    double step_dt = (now() - reverse_step_start_).seconds();

    publishCommand("SERVO190");

    publishCommand("F40");

    if (step_dt >= reverse_step_time_) {
      reverse_step_active_ = false;

      publishCommand("STOP");

      RCLCPP_INFO(get_logger(), "[RECOVERY] step complete");
    }

    return;
  }

  //--------------------------------------------------
  // RED DETECTION
  //--------------------------------------------------

  if (!red_cooldown_ && detectRedStop(img)) {
    waiting_ = true;

    stop_start_ = now();

    publishCommand("STOP");

    publishCommand("SERVO190");

    publishRedTrigger();

    RCLCPP_WARN(get_logger(), "[RED] STOP detected -> waiting %.1fs",
                stop_time_);

    return;
  }

  //--------------------------------------------------
  // LINE DETECTION
  //--------------------------------------------------

  double steering = 0.0;

  double width = 0.0;

  bool found = detectLineContour(img, steering, width);

  //--------------------------------------------------
  // RECOVERY CHECK FRAME
  //--------------------------------------------------

  if (state_ == RobotState::RECOVERY_REVERSE && !reverse_step_active_) {
    if (found) {
      state_ = RobotState::FOLLOW_LINE;

      line_lost_time_ = now();

      RCLCPP_INFO(get_logger(), "[RECOVERY] line reacquired");

      return;
    }

    reverse_step_active_ = true;

    reverse_step_start_ = now();

    RCLCPP_INFO(get_logger(), "[RECOVERY] next reverse step");

    return;
  }

  //--------------------------------------------------
  // NORMAL LINE LOST
  //--------------------------------------------------

  if (!found) {
    if (state_ == RobotState::FOLLOW_LINE) {
      double dt = (now() - line_lost_time_).seconds();

      if (dt > line_loss_delay_) {
        state_ = RobotState::RECOVERY_REVERSE;

        reverse_start_time_ = now();

        reverse_step_active_ = true;

        reverse_step_start_ = now();

        RCLCPP_WARN(get_logger(), "[RECOVERY] starting reverse search");

        return;
      }

      publishCommand("STOP");

      return;
    }

    publishCommand("STOP");

    return;
  }

  //--------------------------------------------------
  // NORMAL FOLLOW
  //--------------------------------------------------

  line_lost_time_ = now();

  state_ = RobotState::FOLLOW_LINE;

  int servo = servoFromError(steering);

  publishCommand("SERVO1" + std::to_string(servo));

  if (width > 0.30) {
    publishCommand("B150");
  } else {
    publishCommand("B80");
  }
}
