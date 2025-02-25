#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <memory>
#include <chrono>
#include <cmath>

class SineWaveTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    subscription_ = node_->create_subscription<std_msgs::msg::Float64>(
        "sine_wave", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { last_received_msg_ = msg; });
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  std_msgs::msg::Float64::SharedPtr last_received_msg_;
};

TEST_F(SineWaveTest, SineWaveReceived)
{
  using namespace std::chrono_literals;

  auto start_time = std::chrono::steady_clock::now();
  while (!last_received_msg_ && std::chrono::steady_clock::now() - start_time < 2s)
  {
    rclcpp::spin_some(node_);
  }

  ASSERT_NE(last_received_msg_, nullptr);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
