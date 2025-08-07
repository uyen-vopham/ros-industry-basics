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

class MyServer : public rclcpp::Node
{

    public:
        MyServer();

    private:
    //----Functions-----
    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response);

    //----Inherit object--------
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::SubscriptionOptions subscription_options_;
};