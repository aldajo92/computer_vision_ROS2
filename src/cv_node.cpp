#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using sensor_msgs::msg::Image;

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber_node")
  {
    // Create a subscription to the image topic
    // Quality of Service (QoS) is set to 'best effort' with a history depth of 1
    subscription_ = this->create_subscription<Image>(
      "/camera/image_raw", // Image topic name
      rclcpp::SensorDataQoS(), // Recommended QoS for sensor data
      std::bind(&ImageSubscriber::image_callback, this, _1));

    // Create a publisher for the processed (Sobel edge detection) image
    publisher_ = this->create_publisher<Image>(
      "/cv/sobel_image", // Output topic name
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "Image Subscriber Node Initialized.");
    RCLCPP_INFO(this->get_logger(), "  Subscribing to: /camera/image_raw");
    RCLCPP_INFO(this->get_logger(), "  Publishing to: /cv/sobel_image");
    RCLCPP_INFO(this->get_logger(), "  Processing: Sobel edge detection");
  }

private:
  void image_callback(const Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received image message.");

    // Convert ROS Image message to OpenCV Mat
    try
    {
      // The bridge handles the conversion. "bgr8" is a common format for color images.
      cv::Mat cv_image = cv_bridge::toCvShare(msg, msg->encoding)->image;

      if (!cv_image.empty())
      {
        // --- OpenCV Sobel Edge Detection ---
        RCLCPP_INFO(this->get_logger(),
          "Image received: Size = %dx%d, Type = %s",
          cv_image.cols,
          cv_image.rows,
          getCvType(cv_image.type()).c_str());

        // Convert to grayscale first (Sobel works on single channel)
        cv::Mat gray_image;
        cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);

        // Apply Sobel edge detection
        cv::Mat grad_x, grad_y;
        cv::Mat abs_grad_x, abs_grad_y;
        cv::Mat sobel_image;

        // Sobel gradient in X direction
        cv::Sobel(gray_image, grad_x, CV_16S, 1, 0, 3);
        cv::convertScaleAbs(grad_x, abs_grad_x);

        // Sobel gradient in Y direction
        cv::Sobel(gray_image, grad_y, CV_16S, 0, 1, 3);
        cv::convertScaleAbs(grad_y, abs_grad_y);

        // Combine gradients (approximation of gradient magnitude)
        cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sobel_image);

        // Convert processed image back to ROS Image message
        std_msgs::msg::Header header;
        header.stamp = msg->header.stamp; // Keep the same timestamp
        header.frame_id = msg->header.frame_id;

        // Create cv_bridge image and convert to ROS message
        cv_bridge::CvImage cv_bridge_image(header, "mono8", sobel_image);
        Image::SharedPtr sobel_msg = cv_bridge_image.toImageMsg();

        // Publish the processed image
        publisher_->publish(*sobel_msg);
        RCLCPP_INFO(this->get_logger(), "Published Sobel edge detection to /cv/sobel_image");
        // --- End of Processing ---
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Received an empty image!");
      }
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }

  // Helper function to get the string representation of an OpenCV Mat type
  std::string getCvType(int type) {
    std::string r;
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);
    switch ( depth ) {
      case CV_8U:  r = "8U"; break;
      case CV_8S:  r = "8S"; break;
      case CV_16U: r = "16U"; break;
      case CV_16S: r = "16S"; break;
      case CV_32S: r = "32S"; break;
      case CV_32F: r = "32F"; break;
      case CV_64F: r = "64F"; break;
      default:     r = "UNKNOWN"; break;
    }
    r += "C";
    r += (chans+'0');
    return r;
  }

  rclcpp::Subscription<Image>::SharedPtr subscription_;
  rclcpp::Publisher<Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
