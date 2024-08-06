#ifndef SIMULINK_WRAPPER_SUBSCRIBER_HPP_
#define SIMULINK_WRAPPER_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimulinkSubscriber : public rclcpp::Node {
public:
    SimulinkSubscriber();

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

#endif // SIMULINK_WRAPPER_SUBSCRIBER_HPP_
