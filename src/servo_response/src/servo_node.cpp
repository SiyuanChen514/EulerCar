#include "servo_response/servo_node.hpp"
#include "servo_response/servo_util.hpp"
#include<string>

using std::placeholders::_1;
using std::placeholders::_2;



void ServoListener::lift(uint16_t time){
    run_servo(HORIZONTAL_SERVO_ID, 2500000);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    close_servo(HORIZONTAL_SERVO_ID);
}

void ServoListener::lay(uint16_t time){
    run_servo(HORIZONTAL_SERVO_ID, 500000);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    close_servo(HORIZONTAL_SERVO_ID);
}

ServoListener::ServoListener() : Node("servo_response_node")
{   
    // 创建服务
    lift_service_ = this->create_service<std_srvs::srv::SetBool>(
        "servo_response",
        std::bind(&ServoListener::service_callback, this, _1, _2)
    );
    
    RCLCPP_INFO(this->get_logger(), "Servo services initialized:");
}

void ServoListener::service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                         std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        try {
        lift();
        response->success = true;
        response->message = "Lift command executed successfully";
        RCLCPP_INFO(this->get_logger(), "Lift command executed successfully");
        } catch (const std::exception& e) {
        response->success = false;
        response->message = "Failed to execute lift command: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Failed to execute lift command: %s", e.what());
        }
    } else {
        try {
            lay();
            response->success = true;
            response->message = "Lay command executed successfully";
            RCLCPP_INFO(this->get_logger(), "Lay command executed successfully");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to execute lay command: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Failed to execute lay command: %s", e.what());
            }
    }
}

int main(int argc, char * argv[])
{
    setlocale(LC_ALL,"");
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ServoListener>());
    rclcpp::shutdown();
    return 0;
}