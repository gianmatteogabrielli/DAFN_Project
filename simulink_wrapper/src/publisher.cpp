#include <rclcpp/rclcpp.hpp>
#include <simulink_wrapper/publisher.hpp>

SimulinkPublisher::SimulinkPublisher()
: Node("simulink_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("simulink_input", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SimulinkPublisher::timer_callback, this));
}

void SimulinkPublisher::timer_callback() {
    auto message = std_msgs::msg::Float64MultiArray();
    message.data = {3.0, 2.0, 3.0, 4.0};  // Dati di esempio

    // Assicurati che il vettore dim abbia la dimensione corretta
    message.layout.dim.resize(2);

    message.layout.dim[0].label = "Rows";
    message.layout.dim[0].size = 2;

    message.layout.dim[1].label = "Columns";
    message.layout.dim[1].size = 2;

    RCLCPP_INFO(this->get_logger(), "Publishing: '%f, %f, %f, %f'", message.data[0], message.data[1], message.data[2], message.data[3]);
    publisher_->publish(message);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulinkPublisher>());
    rclcpp::shutdown();
    return 0;
}
