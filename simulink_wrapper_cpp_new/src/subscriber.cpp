#include <rclcpp/rclcpp.hpp>
#include "simulink_wrapper_cpp_new/subscriber.hpp"

SimulinkSubscriber::SimulinkSubscriber()
: Node("simulink_subscriber") {
    subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "simulink_output", 10, std::bind(&SimulinkSubscriber::topic_callback, this, std::placeholders::_1));
}

void SimulinkSubscriber::topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: '%f, %f, %f, %f'", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    // Logica per interagire con il modello Simulink
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulinkSubscriber>());
    rclcpp::shutdown();
    return 0;
}
