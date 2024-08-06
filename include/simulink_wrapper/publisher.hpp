#ifndef SIMULINK_WRAPPER_PUBLISHER_HPP_
#define SIMULINK_WRAPPER_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimulinkPublisher : public rclcpp::Node {
public:
    SimulinkPublisher();

private:
    void timer_callback();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // SIMULINK_WRAPPER_PUBLISHER_HPP_
