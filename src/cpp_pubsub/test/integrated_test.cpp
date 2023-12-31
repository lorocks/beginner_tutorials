/**
 * @file integrated_test.cpp
 * @author Lowell Lobo
 * @brief Level 2 tests for cpp_pubsub package
 * @version 0.1
 * @date 2023-11-20
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <gtest/gtest.h>
#include <stdlib.h>

#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

/**
 * @brief Test fixture to start and run tests on a chosen node
 *
 */
class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture()
      : node_(std::make_shared<rclcpp::Node>("integrated_tests")) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    // Setup things that should occur before every test instance should go here

    /*
     * 1.) Define any ros2 package and exectuable you want to test
     *  example: package name = cpp_pubsub, node name = minimal_publisher,
     * executable = talker
     */
    bool retVal = StartROSExec("cpp_pubsub", "minimal_publisher", "talker");
    ASSERT_TRUE(retVal);

    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override {
    // Tear things that should occur after every test instance should go here

    // Stop the running ros2 node, if any.
    bool retVal = StopROSExec();
    ASSERT_TRUE(retVal);

    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  std::stringstream cmd_ss, cmdInfo_ss, killCmd_ss;

  bool StartROSExec(const char* pkg_name, const char* node_name,
                    const char* exec_name) {
    // build command strings
    cmd_ss << "ros2 run " << pkg_name << " " << exec_name
           << " > /dev/null 2> /dev/null &";
    cmdInfo_ss << "ros2 node info "
               << "/" << node_name << " > /dev/null 2> /dev/null";
    char execName[16];
    snprintf(execName, sizeof(execName), "%s",
             exec_name);  // pkill uses exec name <= 15 char only
    killCmd_ss << "pkill --signal SIGINT " << execName
               << " > /dev/null 2> /dev/null";

    // First kill the ros2 node, in case it's still running.
    StopROSExec();

    // Start a ros2 node and wait for it to get ready:
    int retVal = system(cmd_ss.str().c_str());
    if (retVal != 0) return false;

    retVal = -1;
    while (retVal != 0) {
      retVal = system(cmdInfo_ss.str().c_str());
      sleep(1);
    }
    return true;
  }

  bool StopROSExec() {
    if (killCmd_ss.str().empty()) return true;

    int retVal = system(killCmd_ss.str().c_str());
    return retVal == 0;
  }
};

/**
 * @brief Construct a new test f object
 * Test for tf2 frames broadcasting
 */
TEST_F(TaskPlanningFixture, tf2Test) {
  std::cout << "tf2 publish test" << std::endl;
  // using namespace std::chrono_literals;

  auto buffer = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

  bool hasRotation = false;
  auto timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      [&]() {
        auto tf = buffer->lookupTransform("talk", "world", tf2::TimePointZero);
        if (tf.transform.rotation.x + tf.transform.rotation.y +
                tf.transform.rotation.z >
            0) {
          hasRotation = true;
        }
      });

  using timer = std::chrono::system_clock;

  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while (elapsed_time < std::chrono::seconds(15)) {
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  EXPECT_TRUE(hasRotation);
}

/**
 * @brief Construct a new test f object
 * Test for talker node publishing
 */
TEST_F(TaskPlanningFixture, publishTest) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  using std_msgs::msg::String;
  using SUBSCRIBER = rclcpp::Subscription<String>::SharedPtr;
  bool hasData = false;
  SUBSCRIBER subscription = node_->create_subscription<String>(
      "topic", 10,
      // Lambda expression begins
      [&](const std_msgs::msg::String& msg) {
        RCLCPP_DEBUG(node_->get_logger(), "I heard: '%s'", msg.data.c_str());
        hasData = true;
      });

  using timer = std::chrono::system_clock;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while (elapsed_time < std::chrono::seconds(5)) {
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  EXPECT_TRUE(hasData);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}
