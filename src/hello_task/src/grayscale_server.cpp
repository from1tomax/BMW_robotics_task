#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "customized_interfaces/srv/image_process.hpp"

using namespace std::placeholders;

class GrayscaleServer : public rclcpp::Node
{
public:
  GrayscaleServer() : Node("grayscale_server")
  {
    service_ = this->create_service<customized_interfaces::srv::ImageProcess>(
        "convert_to_grayscale", std::bind(&GrayscaleServer::convert_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Grayscale conversion server is ready.");
  }

private:
  void convert_callback(const std::shared_ptr<customized_interfaces::srv::ImageProcess::Request> request,
                        std::shared_ptr<customized_interfaces::srv::ImageProcess::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to convert: %s", request->file_path.c_str());

    cv::Mat image = cv::imread(request->file_path, cv::IMREAD_COLOR);
    if (image.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load image.");
      return;
    }

    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    cv_bridge::CvImage cv_image;
    cv_image.encoding = "mono8";
    cv_image.image = gray_image;
    response->grayscale_image = *(cv_image.toImageMsg());

    RCLCPP_INFO(this->get_logger(), "Conversion successful.");
  }

  rclcpp::Service<customized_interfaces::srv::ImageProcess>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GrayscaleServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
