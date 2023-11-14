/**
 * @file subscriber_member_function.cpp
 * @author Lowell Lobo
 * @brief A minimal subscriber node for ros2 that posts the messages received in
 * topic "/topic"
 * @version 0.1
 * @date 2023-11-07
 *
 * @copyright Copyright (c) Lowell Lobo 2023
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief MinimalSubscriber class that inherits form the Node class in rclcpp
 * used to built a ros2 node
 *
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object
   *  Create a subscriber and read messages from topic "/topic"
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscriber = this->create_subscription<std_msgs::msg::String>(
        "topic", 10,
        std::bind(&MinimalSubscriber::timer_callback, this,
                  std::placeholders::_1));
  }

 private:
  /**
   * @brief A member function that logs the message received in "/topic" topic
   *
   * @param msg Message from the topic "/topic"
   */
  void timer_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "Received message: %s", msg.data.c_str());
  }

  /**
   * @brief Create a subscriber shared pointer from rclcpp to be used in the
   * implementation
   *
   */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
};

/**
 * @brief The main implementaiton of the code
 *
 * @param argc Console input argument
 * @param argv Console input argument
 * @return int
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
