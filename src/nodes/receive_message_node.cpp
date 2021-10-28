#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include "executor_exam/receive_message_node.hpp"
#include <chrono>
#include <memory>
#include <thread>

using namespace std::literals::chrono_literals;
using std::placeholders::_1;

namespace executor_test
{

    ReceiveMessageNode::ReceiveMessageNode(const rclcpp::NodeOptions &options)
    : Node("receive_message_node", options)
    {
        counter_called_long_elapsed = 0;
        counter_called_subscriber = 0;
        // timer.
        long_timer_ = this->create_wall_timer(1s, std::bind(&ReceiveMessageNode::count_long_elapsed_time, this));
        // subscriber.
        msg_sub_ = this->create_subscription<std_msgs::msg::String>("thrown_message", rclcpp::SensorDataQoS().keep_last(1), std::bind(&ReceiveMessageNode::recv_message, this, _1));
    }

    void ReceiveMessageNode::count_long_elapsed_time()
    {
        print_count_common("Long timer", counter_called_long_elapsed);
        RCLCPP_INFO(this->get_logger(), "Long timer: %s", received_message.c_str());

        std::this_thread::sleep_for(std::chrono::milliseconds(900));
        return;
    }
    
    void ReceiveMessageNode::recv_message(const std_msgs::msg::String::SharedPtr msg)
    {
        print_count_common("Subscriber", counter_called_subscriber);
        received_message = msg->data;
        RCLCPP_INFO(this->get_logger(), "Subscriber: %s", received_message.c_str());
    }

    void ReceiveMessageNode::print_count_common(const std::string & callback_name, 
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
RCLCPP_COMPONENTS_REGISTER_NODE(executor_test::ReceiveMessageNode)