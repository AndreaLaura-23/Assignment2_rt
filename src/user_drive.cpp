#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <assignment2/srv/get_avg_vel.hpp>

#include <deque>
#include <mutex>
#include <thread>
#include <iostream>
#include <sstream>
#include <atomic>
#include <utility>

class UserDrive : public rclcpp::Node {
public:
  UserDrive() : Node("user_drive") {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_user", 10);

    srv_avg_ = this->create_service<assignment2::srv::GetAvgVel>(
      "/get_avg_vel",
      std::bind(&UserDrive::handle_get_avg, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(),
       "=== ROBOT CONTROL INTERFACE ===\n"
      "Publishing on /cmd_vel_user\n"
      "Service /get_avg_vel active"
      "Type 'q' to quit.");

    input_thread_ = std::thread([this]() { this->input_loop(); });
  }

  ~UserDrive() override {
    running_ = false;
    
    geometry_msgs::msg::Twist stop;
    pub_->publish(stop);

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

    double sum_lin = 0.0;
    double sum_ang = 0.0;
    for (const auto &p : last_cmds_) {
      sum_lin += p.first;
      sum_ang += p.second;
    }
    res->avg_linear  = static_cast<float>(sum_lin / static_cast<double>(n));
    res->avg_angular = static_cast<float>(sum_ang / static_cast<double>(n));
  }

  void input_loop() {
     while (running_ && rclcpp::ok()) {
      double v_lin = 0.0;
      double v_ang = 0.0;
     
      // Linera velocity
      while (running_ && rclcpp::ok()) {
        std::cout << "\nAdd linear velocity: " << std::flush;
        std::string s;
        if (!std::getline(std::cin, s)) return;
        if (s == "q" || s == "quit") { running_ = false; return; }

        std::stringstream ss(s);
        if (ss >> v_lin) break;
        std::cout << "The value not valid, retry.\n";
    }
      // Angular Velocity
      while (running_ && rclcpp::ok()) {
        std::cout << "Add angular velocity: " << std::flush;
        std::string s;
        if (!std::getline(std::cin, s)) return;
        if (s == "q" || s == "quit") { running_ = false; return; }

        std::stringstream ss(s);
        if (ss >> v_ang) break;
        std::cout << "The value not valid, retry.\n";
      }
      
      {
        std::lock_guard<std::mutex> lk(mtx_);
        last_cmds_.push_back({v_lin, v_ang});
        if (last_cmds_.size() > 5) last_cmds_.pop_front();
      }

      geometry_msgs::msg::Twist msg;
      msg.linear.x = v_lin;
      msg.angular.z = v_ang;

      std::cout << "\nSend command for 10 second...\n";

      const auto start = this->now();
      rclcpp::Rate rate(10); // 10 Hz (0.1s)
      while (running_ && rclcpp::ok() && (this->now() - start).seconds() < 10.0) {
        pub_->publish(msg);
        rate.sleep();
      }

      geometry_msgs::msg::Twist stop;
      pub_->publish(stop);
      std::cout << "Robot stopped.\n";
      std::cout << "You can insert a new command now!\n";

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
