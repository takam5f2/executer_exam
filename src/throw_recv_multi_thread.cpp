#include "executor_exam/receive_message_node.hpp"
#include "executor_exam/throw_message_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    rclcpp::executors::MultiThreadedExecutor exec;

    auto throw_message_node = std::make_shared<executor_test::ThrowMessageNode>(options);
    auto receive_message_node = std::make_shared<executor_test::ReceiveMessageNode>(options);

    exec.add_node(throw_message_node);
    exec.add_node(receive_message_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
