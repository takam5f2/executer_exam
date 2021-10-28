#include "executor_exam/callback_group_based_sample_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    rclcpp::executors::MultiThreadedExecutor exec;

    auto sample_node = std::make_shared<executor_test::CallbackGroupBasedSampleNode>(options);

    exec.add_node(sample_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
