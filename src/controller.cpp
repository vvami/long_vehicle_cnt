#include <chrono>
#include <memory> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"
#include "lgsvl_msgs/msg/can_bus_data.hpp"
#include "PIDcnt.hpp"
using std::placeholders::_1;

PIDcnt pid(0.01, 0.001, 0.01, 0.001, 0, 1); 

double ref = 20;

using namespace std::chrono_literals;


class PubSub : public rclcpp::Node
{
public:
  PubSub()
  : Node("pub_sub"), speed(0)
  {
    publisher_ = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/lgsvl/vehicle_control_cmd", 10);
    pub_err = this->create_publisher<std_msgs::msg::Float32>("/speed_error", 10);
    timer_ = this->create_wall_timer(
      1ms, std::bind(&PubSub::timer_callback, this));
    subscription_ = this->create_subscription<lgsvl_msgs::msg::CanBusData>(
      "/lgsvl/state_report", 10, std::bind(&PubSub::topic_callback, this, _1));
  }

private:

  void timer_callback()
  {
    auto message = lgsvl_msgs::msg::VehicleControlData();
    message.acceleration_pct = pid.Compute(ref-speed);
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.acceleration_pct);
    publisher_->publish(message);
    
    auto err_msg = std_msgs::msg::Float32();
    err_msg.data = ref-speed;
    pub_err->publish(err_msg);
  }

  void topic_callback(const lgsvl_msgs::msg::CanBusData::SharedPtr msg) 
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->speed_mps);
    speed = msg->speed_mps;
  }
  
  rclcpp::Subscription<lgsvl_msgs::msg::CanBusData>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_err;
  double speed;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubSub>());
  rclcpp::shutdown();
  return 0;
}