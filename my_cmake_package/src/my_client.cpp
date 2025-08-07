#include "my_client.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
#include <functional> 

MyClient::MyClient():Node("client_cpp_node"), sending_(false)
{

    //-------------CLIENT-----------
    client_ = this->create_client<std_srvs::srv::SetBool>("offboard_service");
     while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    //-------------TIMER CALLBACK----
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                std::bind(&MyClient::main_loop, this));
}

void MyClient::send_request()
{
    if (sending_==true){return;}
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto result = client_->async_send_request(request);
    // Wait for the result.
    auto future_result = client_->async_send_request(request,
        std::bind(&MyClient::sendrequest_callback, this, std::placeholders::_1));
    
    sending_ = true;

}

void MyClient::sendrequest_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
{
    try
    {
        auto response = future.get();

        if (response->success){
            RCLCPP_INFO(this->get_logger(), "Service sucess!");
            // check_armed_=true;
            // pull_waypoint_srv_flag = true;
            // pull_waypoint();
        }
        else  {RCLCPP_WARN(this->get_logger(), "Failed to sending request to service!");}

    }catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    } 
}

void MyClient::main_loop(){

    send_request();
    sending_ = true;

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyClient>();

    // Instead of spin, use a custom executor
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}