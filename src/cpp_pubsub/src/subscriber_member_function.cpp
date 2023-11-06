#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class MinimalSubscriber : public rclcpp::Node{
    public:
        MinimalSubscriber()
        : Node("minimal_subscriber"){
            subscriber = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalSubscriber::timer_callback, this, std::placeholders::_1));
            
        }
    private:
        void timer_callback(const std_msgs::msg::String& msg ) const{
            RCLCPP_INFO(this->get_logger(), "Received message: %s", msg.data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
};


int main (int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}