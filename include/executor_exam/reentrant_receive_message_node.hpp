#ifndef __REENTRANT_RECEIVE_MESSAGE_NODE_HPP__
#define __REENTRANT_RECEIVE_MESSAGE_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace executor_test
{
    class ReentrantReceiveMessageNode : public rclcpp::Node
    {
        public:
            explicit ReentrantReceiveMessageNode(const rclcpp::NodeOptions &options);

        private:
            rclcpp::CallbackGroup::SharedPtr callback_group_reentrant_;
            rclcpp::TimerBase::SharedPtr long_timer_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr msg_sub_;

            uint64_t counter_called_long_elapsed;
            uint64_t counter_called_subscriber;

            std::string received_message;

            void count_long_elapsed_time();
            void recv_message(const std_msgs::msg::String::SharedPtr msg);
            void print_count_common(const std::string & callback_name,
                                    uint64_t & counter);
    };
}

#endif