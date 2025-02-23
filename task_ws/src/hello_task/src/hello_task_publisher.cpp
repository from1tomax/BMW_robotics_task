#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <hello_task/sine_wave_param.hpp>

class SineWavePublisher : public rclcpp::Node
{
public:
  SineWavePublisher() : Node("sine_wave_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("sine_wave", 10);
    param_listener_ = std::make_shared<sine_wave_param::ParamListener>(get_node_parameters_interface());
    int pub_frequency = param_listener_->get_params().pub_frequency;
    time_step_ = 1000 / pub_frequency;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                     std::bind(&SineWavePublisher::publish_sine_wave, this));
  }

private:
  void publish_sine_wave()
  {
    params_ = param_listener_->get_params();
    double ampl = params_.amplitude;
    double angle_freq = params_.angle_frequency;
    double pharse = params_.phase;

    auto msg = std_msgs::msg::Float64();
    // calculate sine value with parameters:  amplitude, angle_frequency and phase
    msg.data = ampl * std::sin(angle_freq * this->now().seconds() + pharse);
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<sine_wave_param::ParamListener> param_listener_;
  sine_wave_param::Params params_;
  int time_step_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SineWavePublisher>());
  rclcpp::shutdown();
  return 0;
}