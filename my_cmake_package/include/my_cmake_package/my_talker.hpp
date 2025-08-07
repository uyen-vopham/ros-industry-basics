#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MyTalkerNode : public rclcpp::Node
{
    public:
    MyTalkerNode()

    private:
    void main_loop();
    void talker_callback();
    double counter_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rrclcpp::TimerBase::SharedPtr timer_
}