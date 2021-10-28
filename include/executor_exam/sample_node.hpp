#ifndef __SAMPLE_NODE_HPP__
#define __SAMPLE_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace executor_test
{
    class SampleNode : public rclcpp::Node
    {
        public:
            explicit SampleNode(const rclcpp::NodeOptions &options);

        private:
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