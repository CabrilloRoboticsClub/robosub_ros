#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class CppTest : public rclcpp::Node
{
public:
  CppTest()
  : Node("cpp_test")
  {
    publisher = this->create_publisher<std_msgs::msg::String>("test_topic", 10);
    timer = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CppTest::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "hello";
    publisher->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppTest>());
  rclcpp::shutdown();
  return 0;
}
