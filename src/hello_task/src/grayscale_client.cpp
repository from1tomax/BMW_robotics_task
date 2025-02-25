#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "customized_interfaces/srv/image_process.hpp"

class GrayscaleClient : public rclcpp::Node
{
public:
  GrayscaleClient() : Node("grayscale_client")
  {
    client_ = this->create_client<customized_interfaces::srv::ImageProcess>("convert_to_grayscale");
  }

  void send_request(const std::string& file_path)
  {
    auto request = std::make_shared<customized_interfaces::srv::ImageProcess::Request>();
    request->file_path = file_path;

    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }

    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      display_image(future.get()->grayscale_image);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }
  }

private:
  void display_image(const sensor_msgs::msg::Image& image_msg)
  {
    cv_bridge::CvImagePtr cv_image;
    try
    {
      cv_image = cv_bridge::toCvCopy(image_msg, "mono8");
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::imshow("Grayscale Image", cv_image->image);
    cv::waitKey(0);
    cv::destroyAllWindows();
  }

  rclcpp::Client<customized_interfaces::srv::ImageProcess>::SharedPtr client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<GrayscaleClient>();

  std::string file_path = "src/hello_task/images/dog-png-30.png";  // 这里修改为你的图片路径
  client->send_request(file_path);

  rclcpp::shutdown();
  return 0;
}
