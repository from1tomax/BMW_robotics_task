#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class SineWaveSubscriber : public rclcpp::Node
{
public:
  SineWaveSubscriber() : Node("sine_wave_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "sine_wave", 10, std::bind(&SineWaveSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received sine wave value: %f", msg->data);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SineWaveSubscriber>());
  rclcpp::shutdown();
  return 0;
}
