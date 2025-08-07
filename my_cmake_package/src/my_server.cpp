#include "my_server.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
#include <functional> 


MyServer::MyServer(): Node("server_cpp_node")
// , check_armed_(false),
//                                                     reach_attitude_(false),
//                                                     landing_flag_(false),
//                                                     pull_waypoint_srv_flag(false),
{
    //----------------Reentrant-----------------
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // service_options_.callback_group = callback_group_;
    
    
    
    //----------------Publisher-----------------



    //----------------Subriber------------------



    //-----------------Service server-----------
    service_ = this-> create_service <std_srvs::srv::SetBool>("offboard_service", std::bind(&MyServer::service_callback, this, _1, _2), rmw_qos_profile_services_default,callback_group_);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to send response");

}




void MyServer::service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
    response -> success = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response ");
}
// self.camera_srv = self.create_service(GetObjectLocation, '/find_objects', 
//     self.service_callback, callback_group=server_cb_group)


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyServer>();

    // Instead of spin, use a custom executor
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
