#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "fqdemo_msgs/msg/num_pwr_result.hpp"
#include "fqdemo_msgs/msg/num_pwr_data.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class DemoSubPub : public rclcpp::Node
{
public:
  DemoSubPub()
  : Node("demo_sub_pub"), count_(0)
  {
    publisher_ = this->create_publisher<fqdemo_msgs::msg::NumPwrResult>("power_result", 10);
    subscriber_ = this->create_subscription<fqdemo_msgs::msg::NumPwrData>(
      "num_power", 10, std::bind(&DemoSubPub::topic_callback, this, _1));
    timer_ = this->create_wall_timer(
      500ms, std::bind(&DemoSubPub::timer_callback, this));
  }

private:
  void topic_callback(const fqdemo_msgs::msg::NumPwrData::SharedPtr msg)
  {
      RCLCPP_INFO(this->get_logger(), "I heard: '%li, %li'", msg->num, msg->power);
  }
  rclcpp::Subscription<fqdemo_msgs::msg::NumPwrData>::SharedPtr subscriber_;

  void timer_callback()
  {
    auto message = fqdemo_msgs::msg::NumPwrResult();
    message.to_root = 3;
    message.to_power = 4;
    RCLCPP_INFO(this->get_logger(), "Publishing NumPwrResult");
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<fqdemo_msgs::msg::NumPwrResult>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoSubPub>());
  rclcpp::shutdown();
  return 0;
}
