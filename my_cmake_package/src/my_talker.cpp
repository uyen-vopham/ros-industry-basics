#include "my_talker.hpp"

MyTalkerNode::MyTalkerNode() : node('my_talker_node'), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("/my_string_cpp", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MyTalkerNode::talker_callback, this)
    );

    
}

void MyTalkerNode::talker_callback(){
    count_ ++;
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

void MyTalkerNode::main_loop()
{


}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyTalkerNode>();

    // Instead of spin, use a custom executor
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
