#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include "executor_exam/tiny_sample_node.hpp"
#include <chrono>
#include <memory>
#include <thread>

using namespace std::literals::chrono_literals;
using std::placeholders::_1;

namespace executor_test
{

    TinySampleNode::TinySampleNode(const rclcpp::NodeOptions &options)
    : Node("tiny_sample_node", options)
    {
        counter_called_short_elapsed = 0;
        counter_called_subscriber = 0;
        // timer.
        short_timer_ = this->create_wall_timer(200ms, std::bind(&TinySampleNode::count_short_elapsed_time, this));
        // subscriber.
        msg_sub_ = this->create_subscription<std_msgs::msg::String>("tiny_node_message", 1, std::bind(&TinySampleNode::print_sub_message, this, _1));
        // publisher.
        msg_pub_ = this->create_publisher<std_msgs::msg::String>("tiny_node_message", 10);
    }

    void TinySampleNode::count_short_elapsed_time()
    {
        std_msgs::msg::String msg;
        msg.data = "Sent from short timer: " + std::to_string(counter_called_short_elapsed);
        print_count_common("Short timer", counter_called_short_elapsed);
        msg_pub_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    void TinySampleNode::print_sub_message(const std_msgs::msg::String::SharedPtr msg)
    {
        print_count_common("Subscriber", counter_called_subscriber);
        RCLCPP_INFO(this->get_logger(), "Subscribed Message: %s", msg->data.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    void TinySampleNode::print_count_common(const std::string & callback_name, 
                                        uint64_t & counter)
    {
        rclcpp::Clock system_clock;
        rclcpp::Time now = system_clock.now();
        uint64_t sec = now.nanoseconds() / 1000000000;
        uint64_t nanosec = now.nanoseconds() % 1000000000;
        RCLCPP_INFO(this->get_logger(), "%s called [%ld] at [%ld.%ld]",
                    callback_name.c_str(), counter, sec, nanosec);
        counter++;
        return;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(executor_test::TinySampleNode)