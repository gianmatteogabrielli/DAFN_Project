#include <rclcpp/rclcpp.hpp>
#include "simulink_wrapper/publisher.hpp"

SimulinkPublisher::SimulinkPublisher()
: Node("simulink_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("simulink_input", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SimulinkPublisher::timer_callback, this));
}

void SimulinkPublisher::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello from Simulink Publisher";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulinkPublisher>());
    rclcpp::shutdown();
    return 0;
}

