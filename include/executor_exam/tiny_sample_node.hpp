#ifndef __TINY_SAMPLE_NODE_HPP__
#define __TINY_SAMPLE_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace executor_test
{
    class TinySampleNode : public rclcpp::Node
    {
        public:
            explicit TinySampleNode(const rclcpp::NodeOptions &options);

        private:
            rclcpp::TimerBase::SharedPtr short_timer_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr msg_sub_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub_;

            uint64_t counter_called_short_elapsed;
            uint64_t counter_called_subscriber;

            void count_short_elapsed_time();
            void print_sub_message(const std_msgs::msg::String::SharedPtr msg);
            void print_count_common(const std::string & callback_name,
                                    uint64_t & counter);
    };
}

#endif