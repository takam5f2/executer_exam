#ifndef __RECEIVE_MESSAGE_NODE_HPP__
#define __RECEIVE_MESSAGE_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace executor_test
{
    class ReceiveMessageNode : public rclcpp::Node
    {
        public:
            explicit ReceiveMessageNode(const rclcpp::NodeOptions &options);

        private:
            rclcpp::TimerBase::SharedPtr long_timer_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr msg_sub_shallow;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr msg_sub;

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