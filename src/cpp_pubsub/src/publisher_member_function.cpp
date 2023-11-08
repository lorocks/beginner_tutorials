/**
 * @file publisher_member_function.cpp
 * @author Lowell Lobo
 * @brief A minimal publisher node for ros2 that publishes the message "Lowell's message numeber: {iterative_number}"
 * @version 0.1
 * @date 2023-11-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief MinimalPublisher class that inherits form the Node class in rclcpp used to built a ros2 node
 * 
 */
class MinimalPublisher : public rclcpp::Node {
 public:
 /**
  * @brief Construct a new Minimal Publisher object
  *  Create a publisher and publish messages to the topic "/topic" at 500ms intervals
  */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    using std::literals::chrono_literals::operator""ms;
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
 /**
  * @brief A member function that runs based on set timer
  * 
  */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Lowell's message number " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing message: '%s'",
                message.data.c_str());
    publisher_->publish(message);
  }

  /**
   * @brief Create a timer shared pointer from rclcpp to be used in implementation
   * 
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Create a publisher shared pointer from rclcpp to be used in the implementation
   * 
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  /**
   * @brief Create a count variable to increment the message number in published message
   * 
   */
  size_t count_;
};

/**
 * @brief The main implementation of the class
 * 
 * @param argc Console input argument
 * @param argv Console input argument
 * @return int 
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
