#include "executor_exam/sample_node.hpp"
#include "executor_exam/tiny_sample_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    rclcpp::executors::MultiThreadedExecutor exec;

    auto sample_node = std::make_shared<executor_test::SampleNode>(options);
    auto tiny_sample_node = std::make_shared<executor_test::TinySampleNode>(options);

    exec.add_node(sample_node);
    exec.add_node(tiny_sample_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
