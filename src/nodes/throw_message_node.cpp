#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include "executor_exam/throw_message_node.hpp"
#include <chrono>
#include <memory>
#include <thread>

using namespace std::literals::chrono_literals;
using std::placeholders::_1;

namespace executor_test
{

    ThrowMessageNode::ThrowMessageNode(const rclcpp::NodeOptions &options)
    : Node("throw_message_node", options)
    {
        publish_count_ = 0;
        // timer.
        timer_ = this->create_wall_timer(200ms, std::bind(&ThrowMessageNode::cyclic_send_message, this));

        // publisher.
        msg_pub_ = this->create_publisher<std_msgs::msg::String>("thrown_message", 10);
    }

    void ThrowMessageNode::cyclic_send_message()
    {
        rclcpp::Clock system_clock;
        rclcpp::Time now = system_clock.now();
        uint64_t sec = now.nanoseconds() / 1000000000;
        uint64_t nanosec = now.nanoseconds() % 1000000000;

        std_msgs::msg::String msg;
        msg.data = "Sent #" + std::to_string(publish_count_) \
                   + " from short timer: " \
                   + std::to_string(sec) \
                   + ","\
                   + std::to_string(nanosec);

        msg_pub_->publish(msg);
        publish_count_++;
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(executor_test::ThrowMessageNode)