#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <assignment2/msg/obstacle_info.hpp>
#include <assignment2/srv/set_threshold.hpp>

#include <cmath>
#include <limits>
#include <string>

class SafetyNode : public rclcpp::Node
{
public:
  SafetyNode() : Node("safety_node")
  {
    // Subscribers
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_user", 10,
      std::bind(&SafetyNode::cmd_callback, this, std::placeholders::_1));

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&SafetyNode::scan_callback, this, std::placeholders::_1));

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&SafetyNode::odom_callback, this, std::placeholders::_1));

    // Publishers
    pub_cmd_safe_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_obstacle_info_ =
      create_publisher<assignment2::msg::ObstacleInfo>("/closest_obstacle", 10);

    // Service
    srv_threshold_ = create_service<assignment2::srv::SetThreshold>(
      "/set_threshold",
      std::bind(&SafetyNode::set_threshold_callback,
                this, std::placeholders::_1, std::placeholders::_2));

    // Timer to publish obstacle info
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SafetyNode::publish_obstacle_info, this));

    RCLCPP_INFO(get_logger(), "Safety node started");
  }

private:

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double min_dist = std::numeric_limits<double>::infinity();
    int min_index = -1;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > 0.0 &&
          msg->ranges[i] < min_dist) {
        min_dist = msg->ranges[i];
        min_index = static_cast<int>(i);
      }
    }

    min_distance_ = min_dist;

    // Determine direction
    int n = msg->ranges.size();
    if (min_index < 0) {
      direction_ = "unknown";
    } else if (min_index < n * 0.33) {
      direction_ = "right";
    } else if (min_index > n * 0.66) {
      direction_ = "left";
    } else {
      direction_ = "front";
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_safe_x_ = msg->pose.pose.position.x;
    last_safe_y_ = msg->pose.pose.position.y;
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::Twist output;

    if (min_distance_ < threshold_) {
      // Too close → move backwards
      output.linear.x = -0.2;
      output.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Obstacle too close (%.2f m). Recovering.", min_distance_);
    } else {
      // Safe → forward user command
      output = *msg;
    }

    pub_cmd_safe_->publish(output);
  }

  void set_threshold_callback(
    const std::shared_ptr<assignment2::srv::SetThreshold::Request> request,
    std::shared_ptr<assignment2::srv::SetThreshold::Response> response)
  {
    if (request->threshold <= 0.0) {
      response->success = false;
      response->message = "Threshold must be positive";
      return;
    }

    threshold_ = request->threshold;
    response->success = true;
    response->message = "Threshold updated";
    RCLCPP_INFO(get_logger(), "New threshold: %.2f", threshold_);
  }

  void publish_obstacle_info()
  {
    assignment2::msg::ObstacleInfo msg;
    msg.distance = std::isfinite(min_distance_) ? min_distance_ : -1.0;
    msg.direction = direction_;
    msg.threshold = threshold_;
    pub_obstacle_info_->publish(msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_safe_;
  rclcpp::Publisher<assignment2::msg::ObstacleInfo>::SharedPtr pub_obstacle_info_;

  rclcpp::Service<assignment2::srv::SetThreshold>::SharedPtr srv_threshold_;
  rclcpp::TimerBase::SharedPtr timer_;

  double min_distance_{std::numeric_limits<double>::infinity()};
  double threshold_{0.6};
  std::string direction_{"unknown"};

  double last_safe_x_{0.0};
  double last_safe_y_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}