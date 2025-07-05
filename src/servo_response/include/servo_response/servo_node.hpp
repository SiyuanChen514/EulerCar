#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "servo_response/servo_util.hpp"
#include <std_srvs/srv/set_bool.hpp> 
#include <thread>


struct ServoState{
  bool h;
  bool v; 
};

class ServoListener : public rclcpp::Node
{
  public:
    ServoListener();
  private:
    short _delay_time;
    int _max_duty_cycle;
    int _min_duty_cycle;

    // 服务回调函数
    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    //void reset();
    void lift(uint16_t time = DELAY_TIME_MS);
    void lay(uint16_t time = DELAY_TIME_MS);

    
    // 服务对象
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr lift_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr lay_service_;
    ServoState state;
};