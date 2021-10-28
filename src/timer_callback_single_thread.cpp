#include "executor_exam/sample_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    rclcpp::executors::SingleThreadedExecutor exec;

    auto sample_node = std::make_shared<executor_test::SampleNode>(options);

    exec.add_node(sample_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
