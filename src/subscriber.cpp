#include <rclcpp/rclcpp.hpp>
#include "simulink_wrapper/subscriber.hpp"

SimulinkSubscriber::SimulinkSubscriber()
: Node("simulink_subscriber") {
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "simulink_input", 10, std::bind(&SimulinkSubscriber::topic_callback, this, std::placeholders::_1));
}

void SimulinkSubscriber::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    // Logica per interagire con il modello Simulink
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulinkSubscriber>());
    rclcpp::shutdown();
    return 0;
}
