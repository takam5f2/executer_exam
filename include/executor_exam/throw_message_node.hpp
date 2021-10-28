#ifndef __THROW_MESSAGE_NODE_HPP__
#define __THROW_MESSAGE_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace executor_test
{
    class ThrowMessageNode : public rclcpp::Node
    {
        public:
            explicit ThrowMessageNode(const rclcpp::NodeOptions &options);

        private:
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub_;

            uint64_t publish_count_;

            void cyclic_send_message();
    };
}

#endif