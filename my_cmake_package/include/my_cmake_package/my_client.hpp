#pragma once
#include <unistd.h>

#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
// #include "rclcpp/service_options.hpp"
// #include "perception_mgss/srv/example_service_cpp.hpp"
#include "std_srvs/srv/set_bool.hpp"


using namespace std::chrono_literals;

class MyClient : public rclcpp::Node
{

    public:
        MyClient();
        void main_loop();

    private:
    //----Functions-----
    void send_request();
    void sendrequest_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future);

    //----Inherit object--------
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
    // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    // rclcpp::CallbackGroup::SharedPtr callback_group_;
    // rclcpp::SubscriptionOptions service_options_;

    //----Variables-------------
    rclcpp::TimerBase::SharedPtr timer_;
    bool sending_;
};