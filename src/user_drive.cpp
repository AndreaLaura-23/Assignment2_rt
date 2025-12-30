#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <assignment2/srv/get_avg_vel.hpp>

#include <deque>
#include <mutex>
#include <thread>
#include <iostream>
#include <sstream>
#include <atomic>

class UserDrive : public rclcpp::Node {
public:
  UserDrive() : Node("user_drive") {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_user", 10);

    srv_avg_ = this->create_service<assignment2::srv::GetAvgVel>(
      "/get_avg_vel",
      std::bind(&UserDrive::handle_get_avg, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(),
      "User drive ready: publishing on /cmd_vel_user. Service /get_avg_vel active.");

    input_thread_ = std::thread([this]() { this->input_loop(); });
  }

  ~UserDrive() override {
    running_ = false;
    if (input_thread_.joinable()) input_thread_.join();
  }

private:
  void handle_get_avg(
    const std::shared_ptr<assignment2::srv::GetAvgVel::Request> /*req*/,
    std::shared_ptr<assignment2::srv::GetAvgVel::Response> res)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const int n = static_cast<int>(last_cmds_.size());
    res->samples = n;

    if (n == 0) {
      res->avg_linear = 0.0f;
      res->avg_angular = 0.0f;
      return;
    }

    double sum_lin = 0.0, sum_ang = 0.0;
    for (auto &p : last_cmds_) {
      sum_lin += p.first;
      sum_ang += p.second;
    }
    res->avg_linear = static_cast<float>(sum_lin / n);
    res->avg_angular = static_cast<float>(sum_ang / n);
  }

  void input_loop() {
    std::string line;
    std::cout << "Enter: linear_vel angolar_vel (e.g., 0.2 0.8). Press Ctrl+C to exit.\n";

    while (running_ && rclcpp::ok()) {
      std::cout << "> " << std::flush;
      if (!std::getline(std::cin, line)) break;
      if (line.empty()) continue;

      std::istringstream iss(line);
      double v_lin, v_ang;
      if (!(iss >> v_lin >> v_ang)) {
        std::cout << "Invalid format. Example: 0.2 0.8\n";
        continue;
      }

      geometry_msgs::msg::Twist msg;
      msg.linear.x = v_lin;
      msg.angular.z = v_ang;
      pub_->publish(msg);

      {
        std::lock_guard<std::mutex> lk(mtx_);
        last_cmds_.push_back({v_lin, v_ang});
        if (last_cmds_.size() > 5) last_cmds_.pop_front();
      }
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Service<assignment2::srv::GetAvgVel>::SharedPtr srv_avg_;

  std::deque<std::pair<double,double>> last_cmds_;
  std::mutex mtx_;

  std::thread input_thread_;
  std::atomic<bool> running_{true};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UserDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
