#include <perception/perception.h>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerceptionNode>();
    RCLCPP_INFO(node->get_logger(), "Perception node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
