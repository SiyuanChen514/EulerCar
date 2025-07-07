#include "servo_response/servo_node.hpp"
#include "servo_response/servo_util.hpp"
#include<string>

using std::placeholders::_1;
using std::placeholders::_2;



void ServoListener::_close(){
    run_servo(HORIZONTAL_SERVO_ID, _max_duty_cycle);
    // std::this_thread::sleep_for(std::chrono::milliseconds(time));
    // close_servo(HORIZONTAL_SERVO_ID);
}

void ServoListener::open(){
    run_servo(HORIZONTAL_SERVO_ID, _min_duty_cycle);
    // std::this_thread::sleep_for(1std::chrono::milliseconds(time));
    // close_servo(HORIZONTAL_SERVO_ID);
}

ServoListener::ServoListener() : Node("servo_response_node")
{   
    this->declare_parameter("delay_time", 500);
    this->declare_parameter("max_duty_cycle", 1900000);
    this->declare_parameter("min_duty_cycle", 1200000);

    this->get_parameter("delay_time", _delay_time);
    this->get_parameter("max_duty_cycle", _max_duty_cycle);
    this->get_parameter("min_duty_cycle", _min_duty_cycle);


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
        _close();
        response->success = true;
        response->message = "Lift command executed successfully";
        stop_timer = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Servo Successfully!");
                close_servo(HORIZONTAL_SERVO_ID);
                stop_timer->cancel(); 
            }
        );
        RCLCPP_INFO(this->get_logger(), "Lift command executed successfully");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to execute lift command: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Failed to execute lift command: %s", e.what());
        }
    } else {
        try {
            open();
            response->success = true;
            response->message = "Lay command executed successfully";
            stop_timer = this->create_wall_timer(
                std::chrono::milliseconds(250),
                [this]() {
                    RCLCPP_INFO(this->get_logger(), "Servo Successfully!");
                    close_servo(HORIZONTAL_SERVO_ID);
                    stop_timer->cancel(); 
                }
        );
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