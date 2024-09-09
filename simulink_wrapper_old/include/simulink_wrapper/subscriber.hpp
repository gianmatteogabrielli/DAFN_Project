#ifndef SIMULINK_WRAPPER_SUBSCRIBER_HPP_
#define SIMULINK_WRAPPER_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>

class SimulinkSubscriber : public rclcpp::Node {
public:
    SimulinkSubscriber();

private:
    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
};

#endif // SIMULINK_WRAPPER_SUBSCRIBER_HPP_
