/**
 * @file publisher_member_function.cpp
 * @author Lowell Lobo
 * @brief A minimal publisher node for ros2 that publishes the message "Lowell's
 * message numeber: {iterative_number}" and also publishes tf2 frames
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

#include "cpp_service/srv/change_counter.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

/**
 * @brief MinimalPublisher class that inherits form the Node class in rclcpp
 * used to built a ros2 node
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   *  Create a publisher and publish messages to the topic "/topic" at 500ms
   * intervals
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    this->declare_parameter("pub_frequency", 750);
    int para_freq = this->get_parameter("pub_frequency").as_int();
    if (para_freq < 450) {
      RCLCPP_FATAL(this->get_logger(),
                   "Publish time too fast...\n Selecting 750ms");
      frequency = 750;
    } else if (para_freq > 3000) {
      RCLCPP_ERROR(this->get_logger(), "Publish time not optimal");
      frequency = para_freq;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Setting custom publish frequency");
      frequency = para_freq;
    }
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    service_ = this->create_service<cpp_service::srv::ChangeCounter>(
        "change_counter",
        std::bind(&MinimalPublisher::change_counter, this,
                  std::placeholders::_1, std::placeholders::_2));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(frequency),
        std::bind(&MinimalPublisher::timer_callback, this));
    tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->transform_publish();
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
   * @brief A member function that is called when the service is initiated.
   * The function changes the count number being publsihed by the talker
   * @param request Service request parameter that holds the parameters input to
   * the service
   * @param response Service response parameter that holds the output from the
   * service
   */
  void change_counter(
      const std::shared_ptr<cpp_service::srv::ChangeCounter::Request> request,
      std::shared_ptr<cpp_service::srv::ChangeCounter::Response> response) {
    count_ = request->number;
    RCLCPP_WARN(this->get_logger(), "Counter has been changed to %i",
                request->number);
    response->status = "Passed";
    RCLCPP_INFO(this->get_logger(), "Service response %s",
                response->status.c_str());
  }

  /**
   * @brief MinimalPublisher member function to create a tf2 frame and publish
   * it
   *
   */
  void transform_publish() {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "talk";

    transform.transform.translation.x = 2;
    transform.transform.translation.y = 3;
    transform.transform.translation.z = 1;

    tf2::Quaternion q;
    q.setRPY(0, 3.14, 1.57);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform);
  }

  /**
   * @brief Create a timer shared pointer from rclcpp to be used in
   * implementation
   *
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Create a publisher shared pointer from rclcpp to be used in the
   * implementation
   *
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  /**
   * @brief Create a service shared pointer from rclcpp to create a service that
   * changes the count value
   *
   */
  rclcpp::Service<cpp_service::srv::ChangeCounter>::SharedPtr service_;

  /**
   * @brief Create a shared pointer for static tf broadcasting
   *
   */
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  /**
   * @brief Create a count variable to increment the message number in published
   * message
   *
   */
  size_t count_;

  /**
   * @brief Create a frenquency variable from talker publish frequency
   *
   */
  int frequency;
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
