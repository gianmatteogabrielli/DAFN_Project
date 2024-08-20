#include <rclcpp/rclcpp.hpp>
#include "simulink_wrapper_cpp/publisher.hpp"

SimulinkPublisher::SimulinkPublisher()
: Node("simulink_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("simulink_input", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SimulinkPublisher::timer_callback, this));
}

void SimulinkPublisher::timer_callback() {
    auto message = std_msgs::msg::Float64MultiArray();
    message.data = {1.0, 2.0, 3.0, 4.0};  // Dati di esempio
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f, %f, %f, %f'", message.data[0], message.data[1], message.data[2], message.data[3]);
    publisher_->publish(message);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulinkPublisher>());
    rclcpp::shutdown();
    return 0;
}

