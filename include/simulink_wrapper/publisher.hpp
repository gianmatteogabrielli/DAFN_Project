#ifndef SIMULINK_WRAPPER_PUBLISHER_HPP_
#define SIMULINK_WRAPPER_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>

class SimulinkPublisher : public rclcpp::Node {
public:
    SimulinkPublisher();

private:
    void timer_callback();

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // SIMULINK_WRAPPER_PUBLISHER_HPP_
