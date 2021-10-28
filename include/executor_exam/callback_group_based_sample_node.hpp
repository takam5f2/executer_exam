#ifndef __CALLBACK_GROUP_BASED_SAMPLE_NODE_HPP__
#define __CALLBACK_GROUP_BASED_SAMPLE_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace executor_test
{
    class CallbackGroupBasedSampleNode : public rclcpp::Node
    {
        public:
            explicit CallbackGroupBasedSampleNode(const rclcpp::NodeOptions &options);

        private:
            rclcpp::CallbackGroup::SharedPtr callback_group_long_;
            rclcpp::CallbackGroup::SharedPtr callback_group_short_;
            rclcpp::TimerBase::SharedPtr long_timer_;
            rclcpp::TimerBase::SharedPtr short_timer_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr msg_sub_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub_;

            uint64_t counter_called_long_elapsed;
            uint64_t counter_called_short_elapsed;
            uint64_t counter_called_subscriber;

            void count_long_elapsed_time();
            void count_short_elapsed_time();
            void print_sub_message(const std_msgs::msg::String::SharedPtr msg);
            void print_count_common(const std::string & callback_name,
                                    uint64_t & counter);
    };
}

#endif