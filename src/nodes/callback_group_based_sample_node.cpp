#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include "executor_exam/callback_group_based_sample_node.hpp"
#include <chrono>
#include <memory>
#include <thread>

using namespace std::literals::chrono_literals;
using std::placeholders::_1;

namespace executor_test
{

    CallbackGroupBasedSampleNode::CallbackGroupBasedSampleNode(const rclcpp::NodeOptions &options)
    : Node("callback_group_based_sample_node", options)
    {
        counter_called_long_elapsed = 0;
        counter_called_short_elapsed = 0;
        counter_called_subscriber = 0;
        // callback group and option definition.
        callback_group_long_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        callback_group_short_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        auto short_opt = rclcpp::SubscriptionOptions();
        short_opt.callback_group = callback_group_long_;

        // timer.
        long_timer_ = this->create_wall_timer(1s, std::bind(&CallbackGroupBasedSampleNode::count_long_elapsed_time, this), callback_group_long_);
        short_timer_ = this->create_wall_timer(200ms, std::bind(&CallbackGroupBasedSampleNode::count_short_elapsed_time, this), callback_group_short_);
        // subscriber.
        msg_sub_ = this->create_subscription<std_msgs::msg::String>("cg_message", 1, std::bind(&CallbackGroupBasedSampleNode::print_sub_message, this, _1), short_opt);
        // publisher.
        msg_pub_ = this->create_publisher<std_msgs::msg::String>("cg_message", 10);
    }

    void CallbackGroupBasedSampleNode::count_long_elapsed_time()
    {
        print_count_common("Long timer", counter_called_long_elapsed);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return;
    }
    void CallbackGroupBasedSampleNode::count_short_elapsed_time()
    {
        std_msgs::msg::String msg;
        msg.data = "Sent from short timer: " + std::to_string(counter_called_short_elapsed);
        print_count_common("Short timer", counter_called_short_elapsed);
        msg_pub_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    void CallbackGroupBasedSampleNode::print_sub_message(const std_msgs::msg::String::SharedPtr msg)
    {
        print_count_common("Subscriber", counter_called_subscriber);
        RCLCPP_INFO(this->get_logger(), "Subscribed Message: %s", msg->data.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    void CallbackGroupBasedSampleNode::print_count_common(const std::string & callback_name, 
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
RCLCPP_COMPONENTS_REGISTER_NODE(executor_test::CallbackGroupBasedSampleNode)